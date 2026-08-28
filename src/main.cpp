// esp-now-router — headless ESP-NOW backbone router for the AMPWorks sensor-sync mesh.
//
// Relays SensorSync ("AMPS") frames multi-hop so edges out of direct ESP-NOW range still hear
// each other. Loop-free flood via the shared, host-tested ss_router_should_relay() (per-origin seq
// dedup + TTL). Headless: no LEDs, no UI.
//
// Wire format + RX ring are the SAME headers the WLED edges use (included from the sibling WLED
// submodule via the -I in platformio.ini) so the format cannot drift.
//
// This file: the relay data path, beacon-based leader election, and neighbor table. The attach
// protocol (router advertisements, node membership, upstream selection) is in attach.{h,cpp} and
// HTTP OTA is in ota.{h,cpp}. The elected leader owns the shared timebase reference.
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
#include "attach.h"                 // attach protocol: router advertisements, membership, failover
#include "ota.h"
#ifdef WLED_COORDINATOR_ROLE
#include "coordinator.h"          // scan/join/push + discover/fan-out (radio glue)
#endif                    // WiFi bring-up + POST /update route (server owned here)

#ifndef ROUTER_ESPNOW_CHANNEL
#define ROUTER_ESPNOW_CHANNEL 1
#endif
#ifndef ROUTER_BEACON_MS
#define ROUTER_BEACON_MS 1000       // beacon cadence (leader election)
#endif
#ifndef ROUTER_CHANNEL_CHECK_MS
// How often to check that the ESP-NOW channel still matches the association. 1 s is far below any
// plausible roam rate and costs one WiFi.status()/WiFi.channel() pair.
#define ROUTER_CHANNEL_CHECK_MS 1000
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

// Live ESP-NOW channel state. Declared here because /info reports it — the whole point of the fix
// below is that the LIVE channel and the build-time default can differ.
static uint8_t  espnowChannel    = ROUTER_ESPNOW_CHANNEL;  // what the radio is actually on
static bool     espnowStarted    = false;  // quickEspNow.begin() has run (see startEspNowWhenReady)
#ifndef ROUTER_ESPNOW_JOIN_GRACE_MS
// How long to wait for the STA to associate before starting ESP-NOW anyway. Bounded so a router
// whose AP is absent still forms a standalone mesh instead of staying mute forever.
#define ROUTER_ESPNOW_JOIN_GRACE_MS 20000
#endif
static uint32_t lastChannelChkMs = 0;
static uint32_t channelFollows   = 0;                      // times we have had to follow the AP
static bool     followWifiChannel = false;                 // true when infra WiFi owns the channel

// Lifetime counters, surfaced by /debug.
static uint32_t rxFrames    = 0;   // our frames received
static uint32_t relayFrames = 0;   // frames re-broadcast
static uint32_t beaconFrames = 0;  // beacons received
static uint32_t controlFrames = 0; // attach-protocol frames received (advert / attach / ack)

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
// Per-origin RSSI of the last frame we heard directly, keyed by the sender's deviceId.
//
// This is the DEVICE-TO-DEVICE measurement, and it is the one worth having: /json/info's wifi.rssi
// is a device's signal to the ACCESS POINT, which places devices on a sphere around the router and
// says nothing about how far they are from each other. ESP-NOW frames arrive with the RSSI of the
// direct radio path between the two nodes, which is what a relative-position estimate actually
// needs. It was being discarded — the callback has always been handed it.
//
// Written from the ESP-NOW RX task and read by the HTTP handler on the Arduino task. Single writer,
// and each field is word-sized and independently meaningful, so a torn read costs at worst one
// stale sample of a value that is noisy by nature — not worth a lock in an interrupt-adjacent path.
struct OriginSignal { uint32_t deviceId; int32_t rssi; uint32_t lastMs; bool used; };
static volatile OriginSignal originSignal[ROUTER_MAX_ORIGINS];

static void noteOriginSignal(uint32_t id, int32_t rssi, uint32_t now) {
  for (int i = 0; i < ROUTER_MAX_ORIGINS; i++) {
    if (originSignal[i].used && originSignal[i].deviceId == id) {
      originSignal[i].rssi = rssi; originSignal[i].lastMs = now; return;
    }
  }
  for (int i = 0; i < ROUTER_MAX_ORIGINS; i++) {
    if (!originSignal[i].used) {
      originSignal[i].used = true; originSignal[i].deviceId = id;
      originSignal[i].rssi = rssi; originSignal[i].lastMs = now; return;
    }
  }
}

static void onEspNowRx(uint8_t *mac, uint8_t *data, uint8_t len, signed int rssi, bool broadcast) {
  (void)mac; (void)broadcast;
  if (!ss_is_our_frame(data, (int)len)) return;
  if (len >= (int)sizeof(SensorSyncHeader)) {
    SensorSyncHeader h;
    memcpy(&h, data, sizeof(h));
    noteOriginSignal(h.deviceId, (int32_t)rssi, millis());
  }
  rxRing.push(data, (int)len);
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
  ra_stamp_control_header(h, selfId, SENSOR_SYNC_MSG_BEACON, sizeof(RouterBeacon), beaconSeq++, now);
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
  j += "\"espnowChannel\":" + String(espnowChannel) + ",";           // LIVE, not the build-time default
  j += "\"espnowChannelConfigured\":" + String(ROUTER_ESPNOW_CHANNEL) + ",";
  j += "\"beaconMs\":" + String(ROUTER_BEACON_MS) + ",";
  j += "\"leaderTimeoutMs\":" + String(ROUTER_LEADER_TIMEOUT_MS) + ",";
  j += "\"maxOrigins\":" + String(ROUTER_MAX_ORIGINS) + ",";
  j += "\"maxNeighbors\":" + String(ROUTER_MAX_NEIGHBORS) + ",";
  j += "\"advertMs\":" + String(ROUTER_ADVERT_MS) + ",";
  j += "\"attachMs\":" + String(ROUTER_ATTACH_MS) + ",";
  j += "\"memberLeaseMs\":" + String(ROUTER_MEMBER_LEASE_MS) + ",";
  j += "\"routeTimeoutMs\":" + String(ROUTER_ROUTE_TIMEOUT_MS) + ",";
  j += "\"routeHoldDownMs\":" + String(ROUTER_ROUTE_HOLDDOWN_MS) + ",";
  j += "\"maxMembers\":" + String(ROUTER_MAX_MEMBERS) + ",";
  j += "\"defaultTtl\":" + String(SS_DEFAULT_TTL);
  j += "}";
  httpServer.send(200, "application/json", j);
}

// GET /routes — the routing state as JSON: per-origin dedup table (who we relay + last seq), the
// peer-router neighbor table (with age), the upstream router we are attached to, and the nodes
// attached to us.
static void handleRoutes() {
  uint32_t now = millis();
  String j = "{\"origins\":[";
  bool first = true;
  for (int i = 0; i < ROUTER_MAX_ORIGINS; i++) {
    if (!relayTbl[i].used) continue;
    if (!first) j += ",";
    first = false;
    j += "{\"id\":\"" + String(relayTbl[i].deviceId, HEX) + "\",\"lastSeq\":" + String(relayTbl[i].lastSeq);
    for (int k = 0; k < ROUTER_MAX_ORIGINS; k++) {
      if (originSignal[k].used && originSignal[k].deviceId == relayTbl[i].deviceId) {
        // rssiDirect: this node's radio path to that origin. NOT the same thing as the device's own
        // wifi.rssi, which is its path to the AP.
        j += ",\"rssiDirect\":" + String((int)originSignal[k].rssi);
        j += ",\"heardMsAgo\":" + String(now - originSignal[k].lastMs);
        break;
      }
    }
    j += "}";
  }
  j += "],\"heard\":[";
  // Every origin we have heard DIRECTLY, whether or not it ever reached the relay table. Kept
  // separate from "origins" on purpose: a node can be audible and still never relay (wrong msgType,
  // deduped, TTL spent), and for distance estimation "did we hear it, and how strongly" is the
  // question — not "did we forward it".
  {
    bool f2 = true;
    for (int i = 0; i < ROUTER_MAX_ORIGINS; i++) {
      if (!originSignal[i].used) continue;
      if (!f2) j += ",";
      f2 = false;
      j += "{\"id\":\"" + String((uint32_t)originSignal[i].deviceId, HEX) + "\"";
      j += ",\"rssiDirect\":" + String((int)originSignal[i].rssi);
      j += ",\"heardMsAgo\":" + String(now - (uint32_t)originSignal[i].lastMs) + "}";
    }
  }
  j += "],\"routers\":[";
  first = true;
  for (int i = 0; i < ROUTER_MAX_NEIGHBORS; i++) {
    if (!neighbors[i].used) continue;
    if (!first) j += ",";
    first = false;
    j += "{\"id\":\"" + String(neighbors[i].deviceId, HEX) + "\",\"ageMs\":" + String(now - neighbors[i].lastSeenMs) + "}";
  }
  j += "],\"upstream\":" + attachRouteJson();
  j += ",\"members\":" + attachMembersJson();
  j += "}";
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
  j += "\"controlFrames\":" + String(controlFrames) + ",";
  j += "\"memberCount\":" + String((unsigned)attachMemberCount()) + ",";
  j += "\"freeHeap\":" + String(ESP.getFreeHeap()) + ",";
  j += "\"espnowChannel\":" + String(espnowChannel) + ",";
  j += "\"channelFollows\":" + String(channelFollows) + ",";
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
  attachBegin(selfId);
  quickEspNow.onDataRcvd(onEspNowRx);
  // ASYNCHRONOUS send (third arg false) — this is a correctness fix, not a tuning choice.
  //
  // QuickEspNow's synchronous path is an UNBOUNDED spin with no timeout:
  //
  //     waitingForConfirmation = true;
  //     while (waitingForConfirmation) { taskYIELD (); }
  //
  // (QuickEspNow_esp32.cpp). `waitingForConfirmation` is cleared by the ESP-NOW TX callback, so if
  // the radio cannot transmit — the case reproduced on the bench 2026-08-22, where the ESP-NOW
  // channel disagreed with the AP the STA was associated to — that callback never arrives and the
  // caller never returns. Since the first thing loop() does after attach is beacon, the whole main
  // loop dies there: HTTP (so /routes, /debug AND POST /update), relaying, beaconing and election
  // all stop. lwIP keeps answering ICMP from its own task, so ping and a port-open check both still
  // say the device is healthy — and OTA, the one recovery path that does not need a cable, is
  // exactly what is broken. Observed directly: "beacon: send enter" with no matching "send exit".
  //
  // Asynchronous send makes an untransmittable frame a DROPPED FRAME instead of a hung CPU, which
  // is the right failure for a broadcast mesh beacon. The relay path already gates on
  // readyToSendData(), which is the async-shaped check, so nothing else has to change.
  // CHANNEL: when we have infra creds, do NOT pin one. The AP owns the channel and a single radio
  // cannot hold two — pinning it does not merely degrade the link, it stops the STA associating at
  // all (bench 2026-08-22: WiFi.status() stuck at WL_DISCONNECTED with ip 0.0.0.0 forever, while
  // WiFi.channel() correctly reported the AP's channel). CURRENT_WIFI_CHANNEL means "leave the
  // channel to WiFi", which is the same rule WLED's own ESP-NOW follows once joined, so edges and
  // routers converge without coordination. ROUTER_ESPNOW_CHANNEL keeps its meaning for the no-infra
  // case (SoftAP / standalone mesh), where nothing else defines a channel.
  //
  // ORDER MATTERS, and getting it wrong is silent. begin(CURRENT_WIFI_CHANNEL) does NOT mean
  // "track the STA": QuickEspNow resolves it ONCE via esp_wifi_get_channel() and then PINS it with
  // setChannel() (QuickEspNow_esp32.cpp:37-44). Called here — before otaWifiBegin() has associated
  // — it pins the BOOT channel. The STA then joins the AP on a different one, the ESP-NOW TX
  // confirm callback stops arriving, and every send returns -3 (COMMS_SEND_QUEUE_FULL_ERROR) once
  // the queue backs up. Measured on the bench 2026-08-27 against Lightbringer (AP on ch 2, router
  // pinned to ch 1): 100% of adverts failed, continuously, while the beacon path — which never
  // checks its return value — hid it. Re-pinning later does not work either: setChannel() fails
  // while associated, exactly as reconcileEspNowChannel() documents (verified: "follow setChannel
  // failed", 172/172 adverts still lost). So the only fix is to start ESP-NOW AFTER association.
  const bool haveInfraCreds = otaHaveInfraCreds();
  followWifiChannel = haveInfraCreds;
  espnowChannel     = haveInfraCreds ? 0 : ROUTER_ESPNOW_CHANNEL;   // 0 = "not known yet"
  if (!haveInfraCreds) {
    // Standalone: nothing else owns the channel, so start immediately.
    quickEspNow.begin(ROUTER_ESPNOW_CHANNEL, 0, /*synchronousSend=*/false);
    espnowStarted = true;
  }
  // Infra: deferred to startEspNowWhenReady() in loop(), so boot stays non-blocking.

  // Top-level web server: WiFi bring-up (ota), then register all routes here and start it.
  otaWifiBegin();
  httpServer.on("/", HTTP_GET, handleRoot);
  httpServer.on("/info", HTTP_GET, handleInfo);
  httpServer.on("/routes", HTTP_GET, handleRoutes);
  httpServer.on("/debug", HTTP_GET, handleDebug);
#ifdef WLED_COORDINATOR_ROLE
  httpServer.on("/coordinator", HTTP_GET, [](){
    CoordinatorStatus c = coordinatorStatus();
    String j = "{";
    j += "\"phase\":\"" + String(c.phase) + "\",";
    j += "\"target\":[" + String(c.targetR) + "," + String(c.targetG) + "," + String(c.targetB) + "],";
    j += "\"onboarded\":" + String(c.onboarded) + ",";
    j += "\"discovered\":" + String(c.discovered) + ",";
    j += "\"synced\":" + String(c.synced) + ",";
    j += "\"hops\":" + String(c.hops) + ",";
    j += "\"hopFailures\":" + String(c.hopFailures) + ",";
    j += "\"homeUp\":" + String(c.homeUp ? "true" : "false");
    // selfHost is the single address the sweep skips. Exposed because a wrong value here silently
    // removes exactly one device from discovery, which is indistinguishable from that device being
    // offline unless you can see the number.
    j += ",\"selfHost\":" + String(c.selfHost);
    j += ",\"sweepAt\":" + String(c.sweepAt);
    j += ",\"sweepFoundNow\":" + String(c.sweepFoundNow);
    j += ",\"retryHits\":" + String(c.retryHits);
    // Policy state. A refused or failed hop without attribution is undiagnosable from outside the
    // device -- this is the gap a bench session hit when it logged 7 hops and 7 failures.
    j += ",\"mode\":\"" + String(c.mode == 1 ? "operate" : "onboard") + "\"";
    j += ",\"lastRefusal\":" + String(c.lastRefusal);
    j += ",\"blacklisted\":" + String(c.blacklisted);
    j += ",\"blacklistFull\":" + String(c.blacklistFull ? "true" : "false");
    j += "}";
    httpServer.send(200, "application/json", j);
  });
  // Set the colour the fleet is driven to. Query args rather than a JSON body so it is one curl.
  // The device registry. Everything here was learned from the sweep that already visits each
  // device, so reading it costs nothing extra.
  httpServer.on("/devices", HTTP_GET, [](){
    uint8_t n = 0;
    const CoordDevice *d = coordinatorDevices(&n);
    uint32_t now = millis();
    String j = "{\"count\":" + String(n) + ",\"devices\":[";
    bool first = true;
    for (uint8_t i = 0; i < COORD_MAX_DEVICES; i++) {
      if (!d[i].used) continue;
      if (!first) j += ",";
      first = false;
      j += "{\"host\":" + String(d[i].host);
      j += ",\"mac\":\"" + String(d[i].mac) + "\"";
      j += ",\"name\":\"" + String(d[i].name) + "\"";
      j += ",\"ver\":\"" + String(d[i].ver) + "\"";
      j += ",\"leds\":" + String(d[i].leds);
      j += ",\"matrix\":" + String(d[i].matrix ? "true" : "false");
      // Named apRssi, not rssi: this is the device's signal to the ACCESS POINT, which is a
      // different measurement from heard[].rssiDirect in /routes (this node's radio path to it).
      // Conflating the two would put every device on a sphere around the router.
      j += ",\"apRssi\":" + String(d[i].apRssi);
      j += ",\"apSignalPct\":" + String(d[i].apSignalPct);
      j += ",\"on\":" + String(d[i].on ? "true" : "false");
      j += ",\"bri\":" + String(d[i].bri);
      j += ",\"fx\":" + String(d[i].fx);
      j += ",\"pal\":" + String(d[i].pal);
      j += ",\"col\":[" + String(d[i].r) + "," + String(d[i].g) + "," + String(d[i].b) + "]";
      j += ",\"seenMsAgo\":" + String(now - d[i].lastSeenMs) + "}";
    }
    j += "]}";
    httpServer.send(200, "application/json", j);
  });
  httpServer.on("/target", HTTP_POST, [](){
    long r = httpServer.arg("r").toInt(), g = httpServer.arg("g").toInt(), b = httpServer.arg("b").toInt();
    if (r < 0 || r > 255 || g < 0 || g > 255 || b < 0 || b > 255) {
      httpServer.send(400, "application/json", "{\"error\":\"r,g,b must each be 0-255\"}");
      return;
    }
    coordinatorSetTarget((uint8_t)r, (uint8_t)g, (uint8_t)b);
    httpServer.send(200, "application/json", "{\"ok\":true}");
  });
#endif
  otaRegisterUpdate(httpServer);   // POST /update
  httpServer.begin();

#ifdef WLED_COORDINATOR_ROLE
  coordinatorSetup();
#endif
  DEBUG1_PRINT("esp-now-router up id=0x");
  DEBUG1_HEX(selfId);
  DEBUG1_VALUELN(" channel=", ROUTER_ESPNOW_CHANNEL);
}

static uint32_t lastBeaconMs = 0;

// Start ESP-NOW once the STA has associated, so begin(CURRENT_WIFI_CHANNEL) resolves and pins the
// AP's channel rather than the boot channel. Non-blocking: loop() keeps serving HTTP meanwhile.
static void startEspNowWhenReady(uint32_t now) {
  if (espnowStarted) return;
  const bool connected = (WiFi.status() == WL_CONNECTED);
  if (!connected && now < ROUTER_ESPNOW_JOIN_GRACE_MS) return;   // still waiting; keep trying
  quickEspNow.begin(connected ? CURRENT_WIFI_CHANNEL : ROUTER_ESPNOW_CHANNEL,
                    0, /*synchronousSend=*/false);
  espnowChannel = connected ? (uint8_t)WiFi.channel() : ROUTER_ESPNOW_CHANNEL;
  espnowStarted = true;
  DEBUG1_VALUE("espnow: started, associated=", (int)connected);
  DEBUG1_VALUELN(" channel=", (int)espnowChannel);
}

// --- ESP-NOW channel must FOLLOW the WiFi association -------------------------------------------
//
// One radio cannot hold two channels. `quickEspNow.begin(ROUTER_ESPNOW_CHANNEL)` runs in setup(),
// but `WiFi.begin()` is non-blocking, so at that moment we are NOT yet associated and the AP's
// channel is unknowable. If the AP then lands on a different channel, the association survives in
// name only: DHCP completes, ARP resolves and ICMP replies, but TCP connects and never answers —
// so /routes, /debug and POST /update are all dead while every cheap health check says the device
// is fine. A router in that state cannot be recovered over the air, which is exactly what
// "OTA one-and-done" exists to prevent. Reproduced on the bench 2026-08-22 (AP on ch 9, default
// ROUTER_ESPNOW_CHANNEL 1).
//
// So the channel is reconciled at RUNTIME rather than pinned at boot: whenever we are associated
// and the radio is not on the AP's channel, follow it. ROUTER_ESPNOW_CHANNEL keeps its meaning for
// the no-infra case (SoftAP / standalone mesh), where nothing else defines the channel.
//
// Note this makes the mesh channel a property of the AP, which is the same rule WLED's own ESP-NOW
// follows once joined — so edges and routers converge on one channel without coordination.
static void reconcileEspNowChannel(uint32_t now) {
  if (!espnowStarted) return;   // nothing to reconcile until the radio is up
  if (now - lastChannelChkMs < ROUTER_CHANNEL_CHECK_MS) return;
  lastChannelChkMs = now;
#ifdef ROUTER_CHANNEL_DIAG
  {
    static uint32_t lastDiag = 0;
    if (now - lastDiag >= 3000) {
      lastDiag = now;
      DEBUG1_VALUE("diag: WiFi.status=", (int)WiFi.status());
      DEBUG1_VALUE(" WiFi.channel=", (int)WiFi.channel());
      DEBUG1_VALUE(" espnowChannel=", (int)espnowChannel);
      DEBUG1_VALUE(" rssi=", (int)WiFi.RSSI());
      DEBUG1_VALUELN(" ip=", WiFi.localIP().toString().c_str());
    }
  }
#endif
  // Deliberately NOT gated on WL_CONNECTED. Gating on it deadlocks: a wrong pinned channel is
  // exactly what stops the association, so a repair that waits for the association never runs.
  // WiFi.channel() reports the channel the STA is working with even while it is still associating.
  // Ignore the reading entirely while a scan is running: WiFi.channel() then reports whichever
  // channel the scan is currently sitting on, not the AP's. Acting on it makes the coordinator
  // "follow" a walk through channels 2, 8, 12, 14 — observed on the bench — and in standalone mode
  // would actively drag the radio around with setChannel().
  if (WiFi.scanComplete() == WIFI_SCAN_RUNNING) return;
  uint8_t apCh = (uint8_t)WiFi.channel();
  if (apCh == 0 || apCh == espnowChannel) return;

  // Two different situations, and calling setChannel() in the wrong one is itself a bug (it fails
  // every time and retries forever, which is what the first cut of this did):
  //
  //   infra mode  — the AP owns the channel and the radio is already on it, because we asked
  //                 QuickEspNow for CURRENT_WIFI_CHANNEL. There is nothing to set; we only record
  //                 what the channel turned out to be so /info can report it. Trying to set it
  //                 while associated fails by design.
  //   standalone  — no infra creds, so WE own the channel. Only here is setChannel() meaningful,
  //                 and only if something has moved us off it.
  if (followWifiChannel) {
    DEBUG1_VALUE("espnow: on AP channel ", apCh);
    DEBUG1_VALUELN(" (was ", espnowChannel);
    espnowChannel = apCh;
    channelFollows++;
    return;
  }
  DEBUG1_VALUE("espnow: re-pinning channel ", apCh);
  DEBUG1_VALUELN(" (was ", espnowChannel);
  if (quickEspNow.setChannel(apCh)) {
    espnowChannel = apCh;
    channelFollows++;
  } else {
    DEBUG_ERR("espnow: setChannel failed");
  }
}

void loop() {
  uint32_t now = millis();
#ifdef ROUTER_CHANNEL_DIAG
  { static uint32_t hb=0; static uint32_t n=0;
    if (now - hb >= 2000) { hb=now; DEBUG1_VALUE("hb ", n++); DEBUG1_VALUE(" ms=", now);
      DEBUG1_VALUE(" wifi=", (int)WiFi.status()); DEBUG1_VALUELN(" ch=", (int)WiFi.channel()); } }
#endif
  // Before anything radio-dependent: start ESP-NOW if the association has landed, then follow the
  // channel if we have associated (or roamed) onto a different one. Cheap — the follow check is
  // gated to once per ROUTER_CHANNEL_CHECK_MS.
  startEspNowWhenReady(now);
  reconcileEspNowChannel(now);
#ifdef WLED_COORDINATOR_ROLE
  coordinatorLoop(now);
#endif
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

    // Attach-protocol frames (advertisement, attach, ack) are consumed by that module; anything
    // it does not claim falls through to the relay path below.
    if (attachHandleFrame(h, buf + sizeof(h), n - (int)sizeof(h), now)) {
      controlFrames++;
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
#ifdef ROUTER_CHANNEL_DIAG
    DEBUG1_PRINTLN("beacon: send enter");
#endif
    sendBeacon(now);
#ifdef ROUTER_CHANNEL_DIAG
    DEBUG1_PRINTLN("beacon: send exit");
#endif
  }

  // 3. Attach tick: release members whose lease ran out, drop a silent upstream router, and send
  //    our advertisement + attach keepalive. Its own intervals gate the sends, so it runs every
  //    loop and reacts to a lost route without waiting for the slower beacon tick.
  attachTick(election, now);
}
