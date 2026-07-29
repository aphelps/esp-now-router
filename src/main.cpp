// esp-now-router — headless ESP-NOW backbone router for the AMPWorks sensor-sync mesh.
//
// Relays SensorSync ("AMPS") frames multi-hop so edges out of direct ESP-NOW range still hear
// each other. Loop-free flood via the shared, host-tested ss_router_should_relay() (per-origin seq
// dedup + TTL). Headless: no LEDs, no UI.
//
// Wire format + RX ring are the SAME headers the WLED edges use (included from the sibling WLED
// submodule via the -I in platformio.ini) so the format cannot drift.
//
// This file: the relay data path, beacon-based leader election, and neighbor table. HTTP OTA is
// in ota.{h,cpp}. The elected leader owns the shared timebase reference.
//
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <QuickEspNow.h>
#include <esp_timer.h>

#include <Debug.h>                  // leveled serial debug (DEBUG_LEVEL: 1=errors+major default)

#include "sensor_sync_protocol.h"   // shared wire format: SensorSyncHeader, ss_is_our_frame, SS_*
#include "sensor_sync_ring.h"       // SpscByteRing (lock-free SPSC, same as the edge RX ring)
#include "router_relay.h"           // router logic: ss_router_should_relay, SensorRouterPeer, RouterBeacon
#include "router_election.h"        // pure leader-election state machine (host-tested)
#include "ota.h"                    // WiFi bring-up + POST /update route (server owned here)

#ifndef ROUTER_ESPNOW_CHANNEL
#define ROUTER_ESPNOW_CHANNEL 1
#endif
#ifndef ROUTER_BEACON_MS
#define ROUTER_BEACON_MS 1000       // beacon cadence (leader election)
#endif
#ifndef ROUTER_SERIAL_BAUD
#define ROUTER_SERIAL_BAUD 115200   // console baud; keep platformio.ini's monitor_speed in step
#endif
#define ROUTER_LEADER_TIMEOUT_MS 3500  // no better beacon within this -> we are leader (~3.5 beacons)
#define ROUTER_MAX_ORIGINS 64       // per-origin dedup table size
#define ROUTER_MAX_NEIGHBORS 16     // known peer routers
#define ROUTER_NEIGHBOR_TIMEOUT_MS 5000

static const uint8_t BCAST_ADDR[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static const int SLOT_LEN = sizeof(SensorSyncHeader) + 64;

static uint32_t          selfId = 0;
static SensorRouterPeer   relayTbl[ROUTER_MAX_ORIGINS];        // origin -> lastSeq dedup
static SpscByteRing<32, sizeof(SensorSyncHeader) + 64> rxRing; // ISR/RX-task -> loop() handoff

static WebServer httpServer(80);   // top-level HTTP server; ota.cpp registers /update on it

// Lifetime counters, surfaced by /debug.
static uint32_t rxFrames    = 0;   // our frames received
static uint32_t relayFrames = 0;   // frames re-broadcast
static uint32_t beaconFrames = 0;  // beacons received

// Neighbor (peer-router) table — populated by beacons.
struct RouterNeighbor { uint32_t deviceId; uint32_t lastSeenMs; bool used; };
static RouterNeighbor neighbors[ROUTER_MAX_NEIGHBORS];

// Leader-election state machine (pure logic in router_election.h so it is host-tested). Starts as
// a follower; asserts leadership only after a hold-down with no better beacon.
static RouterElection election = re_init();
static uint16_t       beaconSeq = 0;

// Monotonic seconds since boot — wrap-safe leadership metric (uint32 s = ~136 y; millis() would
// wrap at ~49.7 days and make the longest-lived leader lose every post-wrap election).
static uint32_t uptimeSecs() { return (uint32_t)(esp_timer_get_time() / 1000000); }

// FNV-1a over the 6-byte MAC — same derivation the edge uses, so ids are consistent fleet-wide.
static uint32_t deriveDeviceId() {
  uint8_t mac[6] = {0};
  WiFi.macAddress(mac);
  uint32_t h = 2166136261u;
  for (int i = 0; i < 6; i++) { h ^= mac[i]; h *= 16777619u; }
  return h ? h : 1;
}

// QuickEspNow RX callback — runs on QuickEspNow's dedicated task, NOT loop(). Keep it minimal:
// claim only our frames and hand them to the SPSC ring; all relay logic runs on loop().
static void onEspNowRx(uint8_t *mac, uint8_t *data, uint8_t len, signed int rssi, bool broadcast) {
  (void)mac; (void)rssi; (void)broadcast;
  if (ss_is_our_frame(data, (int)len)) rxRing.push(data, (int)len);
}

// Record that peer router `id` was heard now: refresh its slot, or take a free one (full -> drop).
static void noteNeighbor(uint32_t id, uint32_t now) {
  for (int i = 0; i < ROUTER_MAX_NEIGHBORS; i++)
    if (neighbors[i].used && neighbors[i].deviceId == id) { neighbors[i].lastSeenMs = now; return; }
  for (int i = 0; i < ROUTER_MAX_NEIGHBORS; i++)
    if (!neighbors[i].used) { neighbors[i] = RouterNeighbor{id, now, true}; return; }
}

// Expire peer routers we haven't heard from within ROUTER_NEIGHBOR_TIMEOUT_MS.
static void sweepNeighbors(uint32_t now) {
  for (int i = 0; i < ROUTER_MAX_NEIGHBORS; i++)
    if (neighbors[i].used && (now - neighbors[i].lastSeenMs) > ROUTER_NEIGHBOR_TIMEOUT_MS)
      neighbors[i] = RouterNeighbor{};
}

// Broadcast our election beacon: an AMPS frame, msgType=BEACON, carrying {uptime, term}. Single
// hop (ttl=1 so no router relays it; edges reject it via ss_parse_header's msgType check).
static void sendBeacon(uint32_t now) {
  uint8_t buf[sizeof(SensorSyncHeader) + sizeof(RouterBeacon)];
  SensorSyncHeader h{};
  h.magic[0] = 'A'; h.magic[1] = 'M'; h.magic[2] = 'P'; h.magic[3] = 'S';
  h.version = SENSOR_SYNC_VERSION;
  h.msgType = SENSOR_SYNC_MSG_BEACON;
  h.dataLen = sizeof(RouterBeacon);
  h.deviceId = selfId;
  h.seq = beaconSeq++;
  h.ttl = 1;               // never relayed
  h.timestamp = now;
  RouterBeacon b{ uptimeSecs(), election.term };
  memcpy(buf, &h, sizeof(h));
  memcpy(buf + sizeof(h), &b, sizeof(b));
  quickEspNow.send(BCAST_ADDR, buf, sizeof(buf));
}

// Parse a received beacon and feed it to the pure election state machine (router_election.h).
static void handleBeacon(const SensorSyncHeader &h, const uint8_t *data, int dataLen, uint32_t now) {
  if (dataLen < (int)sizeof(RouterBeacon)) return;
  RouterBeacon b; memcpy(&b, data, sizeof(b));
  re_on_beacon(election, h.deviceId, b.uptimeTicks, b.term, selfId, uptimeSecs(), now);
}

// GET / — human-readable hint listing the endpoints.
static void handleRoot() {
  httpServer.send(200, "text/plain",
                  "esp-now-router. Endpoints: POST /update (firmware), GET /info /routes /debug\n");
}

// GET /info — the router's configurable values as JSON. Read-only; exposed over HTTP rather than
// serial so the settings are inspectable on a deployed, headless node.
static void handleInfo() {
  String j = "{";
  j += "\"deviceId\":\"" + String(selfId, HEX) + "\",";
  j += "\"espnowChannel\":" + String(ROUTER_ESPNOW_CHANNEL) + ",";
  j += "\"beaconMs\":" + String(ROUTER_BEACON_MS) + ",";
  j += "\"leaderTimeoutMs\":" + String(ROUTER_LEADER_TIMEOUT_MS) + ",";
  j += "\"maxOrigins\":" + String(ROUTER_MAX_ORIGINS) + ",";
  j += "\"maxNeighbors\":" + String(ROUTER_MAX_NEIGHBORS) + ",";
  j += "\"defaultTtl\":" + String(SS_DEFAULT_TTL);
  j += "}";
  httpServer.send(200, "application/json", j);
}

// GET /routes — the routing state as JSON: per-origin dedup table (who we relay + last seq) and
// the peer-router neighbor table (with age).
static void handleRoutes() {
  uint32_t now = millis();
  String j = "{\"origins\":[";
  bool first = true;
  for (int i = 0; i < ROUTER_MAX_ORIGINS; i++) {
    if (!relayTbl[i].used) continue;
    if (!first) j += ",";
    first = false;
    j += "{\"id\":\"" + String(relayTbl[i].deviceId, HEX) + "\",\"lastSeq\":" + String(relayTbl[i].lastSeq) + "}";
  }
  j += "],\"routers\":[";
  first = true;
  for (int i = 0; i < ROUTER_MAX_NEIGHBORS; i++) {
    if (!neighbors[i].used) continue;
    if (!first) j += ",";
    first = false;
    j += "{\"id\":\"" + String(neighbors[i].deviceId, HEX) + "\",\"ageMs\":" + String(now - neighbors[i].lastSeenMs) + "}";
  }
  j += "]}";
  httpServer.send(200, "application/json", j);
}

// GET /debug — recent runtime state as JSON: leadership, counters, uptime, heap, WiFi.
static void handleDebug() {
  String j = "{";
  j += "\"uptimeS\":" + String(uptimeSecs()) + ",";
  j += "\"isLeader\":" + String(election.isLeader ? "true" : "false") + ",";
  j += "\"leaderId\":\"" + String(election.leaderId, HEX) + "\",";
  j += "\"term\":" + String(election.term) + ",";
  j += "\"rxFrames\":" + String(rxFrames) + ",";
  j += "\"relayFrames\":" + String(relayFrames) + ",";
  j += "\"beaconFrames\":" + String(beaconFrames) + ",";
  j += "\"freeHeap\":" + String(ESP.getFreeHeap()) + ",";
  j += "\"staWifiUp\":" + String(otaWifiUp() ? "true" : "false");
  j += "}";
  httpServer.send(200, "application/json", j);
}

void setup() {
  Serial.begin(ROUTER_SERIAL_BAUD);

  // Radio: STA mode is set here, but the association itself happens in otaWifiBegin() below —
  // that joins infra WiFi when creds are set (which also locks the radio to the AP channel),
  // else raises the fallback SoftAP.
  WiFi.mode(WIFI_STA);
  selfId = deriveDeviceId();
  quickEspNow.onDataRcvd(onEspNowRx);
  quickEspNow.begin(ROUTER_ESPNOW_CHANNEL);

  // Top-level web server: WiFi bring-up (ota), then register all routes here and start it.
  otaWifiBegin();
  httpServer.on("/", HTTP_GET, handleRoot);
  httpServer.on("/info", HTTP_GET, handleInfo);
  httpServer.on("/routes", HTTP_GET, handleRoutes);
  httpServer.on("/debug", HTTP_GET, handleDebug);
  otaRegisterUpdate(httpServer);   // POST /update
  httpServer.begin();

  DEBUG1_PRINT("esp-now-router up id=0x");
  DEBUG1_HEX(selfId);
  DEBUG1_VALUELN(" channel=", ROUTER_ESPNOW_CHANNEL);
}

static uint32_t lastBeaconMs = 0;

void loop() {
  uint32_t now = millis();
  httpServer.handleClient();   // service the HTTP endpoints (/update, /info, /routes, /debug)

  // 1. Drain the RX ring: relay SNAPSHOT frames; feed BEACON frames to the election (never relay
  //    beacons — they are single-hop router-to-router).
  uint8_t buf[SLOT_LEN];
  int n;
  while ((n = rxRing.pop(buf, sizeof(buf))) > 0) {
    if (n < (int)sizeof(SensorSyncHeader)) continue;
    SensorSyncHeader h;
    memcpy(&h, buf, sizeof(h));
    rxFrames++;
    if (h.msgType == SENSOR_SYNC_MSG_BEACON) {
      beaconFrames++;
      noteNeighbor(h.deviceId, now);                               // beacon senders are peer routers
      handleBeacon(h, buf + sizeof(h), n - (int)sizeof(h), now);
      continue;
    }

    // Backpressure: if the TX queue can't take a frame right now, skip WITHOUT deduping so a
    // re-heard copy (this is a flood — the same frame arrives from multiple neighbors) can still
    // relay it once the queue drains. Deduping-then-dropping would strand this seq forever.
    if (!quickEspNow.readyToSendData()) continue;
    uint8_t outTtl = 0;
    if (ss_router_should_relay(h, selfId, relayTbl, ROUTER_MAX_ORIGINS, &outTtl)) {
      buf[offsetof(SensorSyncHeader, ttl)]    = outTtl;              // stamp decremented hop budget
      buf[offsetof(SensorSyncHeader, flags)] |= SS_FLAG_RELAYED;     // mark as relayed (diagnostics)
      if (quickEspNow.send(BCAST_ADDR, buf, n) == 0) {
        relayFrames++;
      } else {
        DEBUG_ERR("relay: send failed");
      }
    }
  }

  // 2. Beacon tick: sweep dead neighbors, (re)assert leadership if no better beacon arrived within
  //    the timeout, then beacon our own metric. Degenerates cleanly to N=1 (a lone router hears no
  //    better beacon -> stays leader).
  if (now - lastBeaconMs >= ROUTER_BEACON_MS) {
    lastBeaconMs = now;
    sweepNeighbors(now);
    re_tick(election, selfId, uptimeSecs(), now, ROUTER_LEADER_TIMEOUT_MS);   // (re)assert leadership
    sendBeacon(now);
  }
}
