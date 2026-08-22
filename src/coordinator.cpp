// coordinator.cpp — see coordinator.h. Only the radio-facing half lives here.
#ifdef WLED_COORDINATOR_ROLE

#include "coordinator.h"

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <Debug.h>

#include "wled_candidate.h"
#include "wled_onboard_cfg.h"
#include "wled_translate.h"

#ifndef COORD_HOME_SSID
#define COORD_HOME_SSID ""
#endif
#ifndef COORD_HOME_PASS
#define COORD_HOME_PASS ""
#endif
// One hop must be bounded or a target that half-answers strands us off the home network. Sized
// above a worst-case join+push+verify, well below anything a human would wait through.
#ifndef COORD_HOP_BUDGET_MS
#define COORD_HOP_BUDGET_MS 45000
#endif
// How long orphans get to converge onto the home network after a push, before we sweep for them.
// A WLED device retries its STA every 18 s (wled.cpp), so anything under that guarantees a miss.
#ifndef COORD_SETTLE_MS
// 30 s, matching what coordinator_phase.h computes. They were 25000 here and 30000 there — separate
// translation units, no warning, and both satisfied that header's static_assert, so the two could
// drift indefinitely without anything complaining. A pushed device retries its STA every 18 s
// (wled.cpp), so the window must clear that with margin.
#define COORD_SETTLE_MS 30000
#endif
#ifndef COORD_SYNC_PERIOD_MS
#define COORD_SYNC_PERIOD_MS 30000
#endif
#ifndef COORD_SCAN_PERIOD_MS
#define COORD_SCAN_PERIOD_MS 20000
#endif
#ifndef COORD_HTTP_TIMEOUT_MS
#define COORD_HTTP_TIMEOUT_MS 4000
#endif
#ifndef COORD_TARGET_BRI
#define COORD_TARGET_BRI 200
#endif

static const char *NVS_NS   = "coord";
static const char *NVS_SSID = "home_ssid";
static const char *NVS_PASS = "home_pass";

// NOTE: there is deliberately no PH_HOP. The hop runs inline and blocking inside the PH_SCAN case
// (bounded by COORD_HOP_BUDGET_MS), so the real transition is PH_SCAN -> PH_SETTLE. An enumerator
// that can never be entered would misdescribe the machine to anyone reading /coordinator.
enum Phase : uint8_t { PH_BOOT = 0, PH_SCAN, PH_SETTLE, PH_SYNC };
static const char *phaseName(Phase p) {
  switch (p) {
    case PH_BOOT:   return "boot";
    case PH_SCAN:   return "scan";
    case PH_SETTLE: return "settle";
    default:        return "sync";
  }
}

static Phase       phase        = PH_BOOT;
static uint32_t    phaseSince   = 0;
static uint32_t    lastScanMs   = 0;
static uint32_t    lastSyncMs   = 0;
static WledPushLog pushed;
static WledColour  target       = { 255, 0, 0 };   // Adam's acceptance shape: solid red by default
static CoordinatorStatus st     = {};
static char        homeSsid[33] = {0};
static char        homePass[65] = {0};

// --- home config -------------------------------------------------------------------------------
// Persisted BEFORE the first hop, not after: a watchdog reset mid-hop must come back knowing where
// home is, and a value that only exists in RAM is exactly what a reset destroys.
static void loadHome() {
  Preferences p;
  if (p.begin(NVS_NS, true)) {
    String s = p.getString(NVS_SSID, ""), k = p.getString(NVS_PASS, "");
    p.end();
    if (s.length()) { strncpy(homeSsid, s.c_str(), sizeof(homeSsid) - 1); strncpy(homePass, k.c_str(), sizeof(homePass) - 1); return; }
  }
  strncpy(homeSsid, COORD_HOME_SSID, sizeof(homeSsid) - 1);
  strncpy(homePass, COORD_HOME_PASS, sizeof(homePass) - 1);
}

static void storeHome() {
  Preferences p;
  if (!p.begin(NVS_NS, false)) return;
  p.putString(NVS_SSID, homeSsid);
  p.putString(NVS_PASS, homePass);
  p.end();
}

static bool joinHome(uint32_t timeoutMs) {
  WiFi.disconnect(true, true);
  delay(80);
  WiFi.mode(WIFI_STA);
  WiFi.begin(homeSsid, homePass);
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    if (WiFi.status() == WL_CONNECTED) { st.homeUp = true; return true; }
    delay(150);
  }
  st.homeUp = false;
  return false;
}

// --- HTTP helpers ------------------------------------------------------------------------------
static int httpPost(const char *host, const char *path, const char *body, String *out) {
  HTTPClient http;
  String url = String("http://") + host + path;
  if (!http.begin(url)) return -1;
  http.setTimeout(COORD_HTTP_TIMEOUT_MS);
  http.addHeader("Content-Type", "application/json");
  int code = http.POST((uint8_t *)body, strlen(body));
  if (out && code > 0) *out = http.getString();
  http.end();
  return code;
}

static int httpGetT(const char *host, const char *path, String *out, uint16_t timeoutMs) {
  HTTPClient http;
  String url = String("http://") + host + path;
  if (!http.begin(url)) return -1;
  http.setConnectTimeout(timeoutMs);
  http.setTimeout(timeoutMs);
  int code = http.GET();
  if (out && code > 0) *out = http.getString();
  http.end();
  return code;
}

static int httpGet(const char *host, const char *path, String *out) {
  return httpGetT(host, path, out, COORD_HTTP_TIMEOUT_MS);
}

// A stock WLED device in AP-fallback always answers on 4.3.2.1 — its own captive-portal address.
static const char *AP_HOST = "4.3.2.1";

// --- onboarding one target ---------------------------------------------------------------------
// Returns true if the target accepted our credentials. Leaves the radio detached from the target AP
// either way: staying associated backs the target's own STA retry off from 18 s to 300 s
// (wled.cpp), which delays the exact join we are waiting for.
static bool onboardOne(const WledScanResult &cand) {
  st.hops++;
  DEBUG1_VALUELN("coord: hopping to ", cand.ssid);
  uint32_t deadline = millis() + COORD_HOP_BUDGET_MS;

  // Every exit from here on goes through `done:`. The detach is not a courtesy — an early return
  // while still associated to the target's setup AP strands us there, and it is invisible to the
  // settle phase, which only asks WiFi.status() != WL_CONNECTED and gets "connected" (to the wrong
  // network). It also backs the TARGET's own STA retry off from 18 s to 300 s (wled.cpp), delaying
  // the exact join we are waiting for. Two failure paths used to return without detaching.
  bool ok = false;

  WiFi.disconnect(true, true);
  delay(80);
  WiFi.mode(WIFI_STA);
  WiFi.begin(cand.ssid, WLED_SETUP_AP_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    if ((int32_t)(millis() - deadline) >= 0) {
      DEBUG_ERR("coord: join timed out");
      goto done;                      // never associated, but detach anyway to clear WiFi state
    }
    delay(150);
  }

  {
    // Confirm it really is WLED before handing over credentials. The OUI pre-filter cannot do this
    // — it only says the silicon is Espressif, not that the firmware is WLED.
    String info;
    if (httpGet(AP_HOST, "/json/info", &info) <= 0 || info.indexOf("\"brand\":\"WLED\"") < 0) {
      DEBUG_ERR("coord: target did not identify as WLED — not pushing credentials");
      goto done;
    }

    WledOnboardCfg cfg = {};
    cfg.ssid = homeSsid;
    cfg.psk  = homePass;
    uint8_t mac[6]; WiFi.macAddress(mac);
    memcpy(cfg.coordinator_mac.b, mac, 6);
    cfg.enable_espnow      = true;    // the radio itself; the MVP needs this and only this
    cfg.enable_espnow_sync = false;   // NOT WLED's own sync master — we push state ourselves

    char body[512]; size_t wrote = 0;
    if (wled_build_onboard_cfg(cfg, body, sizeof(body), &wrote) != WLED_CFG_OK) {
      DEBUG_ERR("coord: could not build onboarding cfg");
      goto done;
    }

    int code = httpPost(AP_HOST, "/json/cfg", body, nullptr);
    ok = (code >= 200 && code < 300);
    DEBUG1_VALUELN("coord: cfg push http=", code);
    if (ok) {
      // A full push log means every slot is taken and we can no longer remember that this target
      // was done — which is precisely the condition that makes the own-AP re-candidacy loop run
      // forever. Say so rather than discarding the answer.
      if (!wled_pushlog_add(&pushed, cand.bssid))
        DEBUG_ERR("coord: push log FULL — this target may be re-pushed on the next scan");
      st.onboarded++;
    }
  }

done:
  if (!ok) st.hopFailures++;
  WiFi.disconnect(true, true);        // detach and STAY off, on every path
  delay(120);
  return ok;
}


// --- minimal JSON field extraction ----------------------------------------------------------------
// Deliberately not a JSON parser. WLED's /json/info is several kB and a real parser would cost more
// heap than the whole registry; these pull single well-known fields out of a response we already
// hold as a String. `scope` optionally narrows the search to the object after a given key, so
// "rssi" inside "wifi" cannot be confused with an "rssi" elsewhere in the document.
static int jsonFindScope(const String &doc, const char *scopeKey) {
  if (!scopeKey) return 0;
  String k = String("\"") + scopeKey + "\":";
  int at = doc.indexOf(k);
  return at < 0 ? -1 : at + k.length();
}

static bool jsonInt(const String &doc, const char *key, long *out, const char *scopeKey = nullptr) {
  int from = jsonFindScope(doc, scopeKey);
  if (from < 0) return false;
  String k = String("\"") + key + "\":";
  int at = doc.indexOf(k, from);
  if (at < 0) return false;
  at += k.length();
  while (at < (int)doc.length() && doc[at] == ' ') at++;
  int end = at;
  if (end < (int)doc.length() && (doc[end] == '-' || doc[end] == '+')) end++;
  int digits = end;
  while (end < (int)doc.length() && isdigit((unsigned char)doc[end])) end++;
  if (end == digits) return false;                 // null / true / a string: not an int
  *out = doc.substring(at, end).toInt();
  return true;
}

static bool jsonStr(const String &doc, const char *key, char *out, size_t cap,
                    const char *scopeKey = nullptr) {
  int from = jsonFindScope(doc, scopeKey);
  if (from < 0) return false;
  String k = String("\"") + key + "\":\"";
  int at = doc.indexOf(k, from);
  if (at < 0) return false;
  at += k.length();
  int end = doc.indexOf('"', at);
  if (end < 0) return false;
  size_t n = (size_t)(end - at);
  if (n >= cap) n = cap - 1;
  memcpy(out, doc.c_str() + at, n);
  out[n] = 0;
  return true;
}

// True when the key is present AND not null — how leds.matrix signals a 2D device.
static bool jsonPresentNotNull(const String &doc, const char *key) {
  String k = String("\"") + key + "\":";
  int at = doc.indexOf(k);
  if (at < 0) return false;
  at += k.length();
  while (at < (int)doc.length() && doc[at] == ' ') at++;
  return doc.indexOf("null", at) != at;
}

// --- the registry ---------------------------------------------------------------------------------
static CoordDevice devices[COORD_MAX_DEVICES];

// How long a device may go unseen before its record is treated as stale. Two sweeps' worth plus
// slack: a device that misses one sweep to power-save should not vanish from the registry, but one
// that has actually left must not linger forever claiming to be present — a stale record is worse
// than a missing one, because callers act on it.
#ifndef COORD_DEVICE_STALE_MS
#define COORD_DEVICE_STALE_MS (COORD_SYNC_PERIOD_MS * 3)
#endif

static CoordDevice *deviceSlot(uint8_t host, uint32_t now) {
  for (uint8_t i = 0; i < COORD_MAX_DEVICES; i++)
    if (devices[i].used && devices[i].host == host) return &devices[i];
  for (uint8_t i = 0; i < COORD_MAX_DEVICES; i++)
    if (!devices[i].used) { memset(&devices[i], 0, sizeof(devices[i])); devices[i].used = true; devices[i].host = host; return &devices[i]; }
  // Full. Reclaim the most stale entry rather than refusing: refusing means a fleet that has churned
  // past COORD_MAX_DEVICES addresses can never see a new device again, which is a worse failure than
  // forgetting one that has not answered in three sweeps.
  int oldest = -1; uint32_t oldestAge = 0;
  for (uint8_t i = 0; i < COORD_MAX_DEVICES; i++) {
    uint32_t age = now - devices[i].lastSeenMs;
    if (age > oldestAge) { oldestAge = age; oldest = i; }
  }
  if (oldest >= 0 && oldestAge >= COORD_DEVICE_STALE_MS) {
    memset(&devices[oldest], 0, sizeof(devices[oldest]));
    devices[oldest].used = true; devices[oldest].host = host;
    return &devices[oldest];
  }
  return nullptr;   // everything is fresh: keep what we have
}

// Drop records for devices that have not answered in a while, so /devices reports what is actually
// there. Called once per completed sweep rather than per address.
static void registryExpire(uint32_t now) {
  for (uint8_t i = 0; i < COORD_MAX_DEVICES; i++) {
    if (!devices[i].used) continue;
    if ((uint32_t)(now - devices[i].lastSeenMs) >= COORD_DEVICE_STALE_MS) {
      DEBUG1_VALUELN("coord: registry expiring host ", devices[i].host);
      memset(&devices[i], 0, sizeof(devices[i]));
    }
  }
}

// Record everything we can learn about one device from the two documents we already fetch.
static void registryNote(uint8_t host, const String &info, const String &state, uint32_t now) {
  CoordDevice *d = deviceSlot(host, now);
  if (!d) return;
  jsonStr(info, "mac",  d->mac,  sizeof(d->mac));
  jsonStr(info, "name", d->name, sizeof(d->name));
  jsonStr(info, "ver",  d->ver,  sizeof(d->ver));
  long v;
  if (jsonInt(info, "count",  &v, "leds")) d->leds        = (uint16_t)v;
  if (jsonInt(info, "rssi",   &v, "wifi")) d->apRssi      = (int8_t)v;
  if (jsonInt(info, "signal", &v, "wifi")) d->apSignalPct = (uint8_t)v;
  d->matrix = jsonPresentNotNull(info, "matrix");
  if (jsonInt(state, "bri", &v)) d->bri = (uint8_t)v;
  d->on = state.indexOf("\"on\":true") >= 0;
  if (jsonInt(state, "fx",  &v)) d->fx  = (uint8_t)v;
  if (jsonInt(state, "pal", &v)) d->pal = (uint8_t)v;
  // col is [[r,g,b],...]; take the first triple.
  int c = state.indexOf("\"col\":[[");
  if (c >= 0) {
    c += 8;
    long rgb[3] = {0,0,0}; int idx = 0;
    while (idx < 3 && c < (int)state.length()) {
      while (c < (int)state.length() && (state[c] == ' ' || state[c] == ',')) c++;
      int st2 = c;
      while (c < (int)state.length() && isdigit((unsigned char)state[c])) c++;
      if (c == st2) break;
      rgb[idx++] = state.substring(st2, c).toInt();
    }
    d->r = (uint8_t)rgb[0]; d->g = (uint8_t)rgb[1]; d->b = (uint8_t)rgb[2];
  }
  d->lastSeenMs = now;
}

const CoordDevice *coordinatorDevices(uint8_t *countOut) {
  if (countOut) {
    uint8_t n = 0;
    for (uint8_t i = 0; i < COORD_MAX_DEVICES; i++) if (devices[i].used) n++;
    *countOut = n;
  }
  return devices;
}

// --- discovery + fan-out -------------------------------------------------------------------------
// Sweeps the home /24 for anything answering /json/info as WLED, then sets each one's colour.
// Deliberately the same shape as the Mac-side wled_sync.py, which is this firmware's oracle.
//
// INCREMENTAL, and that is not an optimisation. A straight loop over 254 addresses at the push
// timeout is up to ~17 minutes inside one loop() call, during which handleClient() never runs — so
// /coordinator, /debug and POST /update all go dark and the device looks bricked. That is precisely
// the failure just fixed in the router's beacon path, and it is just as easy to reintroduce here.
// So the sweep advances a few addresses per pass and yields.
#ifndef COORD_SWEEP_PER_PASS
// Two, not six. This bounds how long ONE loop() pass can be stuck in synchronous HTTP: each probe
// costs up to COORD_DISCOVER_TIMEOUT_MS against a silent address, and 6 x 1200 ms is 7.2 s during
// which handleClient() does not run and the device looks dead to anything talking HTTP. Measured:
// at 6 per pass, 3 of 12 status polls timed out; the sweep is background work and must never be
// the reason an OTA or a status request fails.
#define COORD_SWEEP_PER_PASS 2
#endif
// Discovery timeout is deliberately much shorter than the push timeout: a WLED device already on
// the LAN answers /json/info in milliseconds, and 248 of the 254 addresses are silence we are
// paying for. The push keeps the longer budget because a target mid-reboot legitimately stalls.
// Attempts per address. TWO SHORT tries beat one long one, and this was measured rather than
// reasoned: with a single 1200 ms probe, a sweep that had walked past all four devices on the LAN
// had found two, and a fast-fail-only retry recovered zero (retryHits stayed 0) — so the misses
// were full timeouts, not quick errors.
//
// The measurement that misled me first time was taken from the WRONG VANTAGE POINT: the host answers
// every device in under 100 ms, 12/12, which says nothing about ESP32-to-ESP32 HTTP over the same
// AP, where a peer in WiFi power-save is far less prompt. The coordinator's view is the only one
// that matters here.
//
// Two attempts at COORD_DISCOVER_TIMEOUT_MS cost about the same wall clock as the single longer
// probe they replace, but give two independent chances at a device that simply did not answer the
// first time.
// Gap between probes, and between the two attempts on one address. See the pacing note in
// sweepStep for why this exists.
#ifndef COORD_PROBE_GAP_MS
#define COORD_PROBE_GAP_MS 60
#endif
#ifndef COORD_PROBE_ATTEMPTS
#define COORD_PROBE_ATTEMPTS 2
#endif
#ifndef COORD_DISCOVER_TIMEOUT_MS
// Per-ATTEMPT timeout; see COORD_PROBE_ATTEMPTS above for why there are two of them. 700 ms x 2 is
// roughly the wall clock of the single 1200 ms probe this replaces.
#define COORD_DISCOVER_TIMEOUT_MS 700
#endif

static uint8_t  sweepHost   = 0;      // next address to probe; 0 = sweep not running
static uint16_t sweepFound  = 0;
static uint16_t sweepSet    = 0;
static uint16_t sweepRetryHits = 0;   // devices found ONLY because of the retry
static uint8_t  knownIdx    = 0;      // cursor over already-known hosts, run before the full sweep

static void sweepBegin() { sweepHost = 1; sweepFound = 0; sweepSet = 0; knownIdx = 0; }

// Drive the devices we ALREADY know about, before walking the whole subnet.
//
// A full /24 sweep is ~127 passes at COORD_SWEEP_PER_PASS, and 250 of the 254 addresses are silence
// paid for at the discovery timeout — minutes before a newly-onboarded device gets its colour, even
// though the coordinator learned its address the moment it joined. Registry entries are exactly the
// list of hosts worth talking to, so they get driven first and the exhaustive sweep continues
// behind them purely to FIND devices we have not seen.
//
// Returns true when the known set has been walked for this cycle.
static bool syncKnownStep(const char *bodyBuf) {
  IPAddress me = WiFi.localIP();
  for (int i = 0; i < COORD_SWEEP_PER_PASS && knownIdx < COORD_MAX_DEVICES; i++, knownIdx++) {
    const CoordDevice &d = devices[knownIdx];
    if (!d.used || d.host == me[3]) continue;
    char ip[16];
    snprintf(ip, sizeof(ip), "%u.%u.%u.%u", me[0], me[1], me[2], d.host);
    httpPost(ip, "/json/state", bodyBuf, nullptr);
  }
  return knownIdx >= COORD_MAX_DEVICES;
}

// Returns true when the sweep has finished this pass over the subnet.
// DISCOVERY: the cause of the missed devices was found, and it was none of the first three things
// it looked like. Kept in full because the wrong answers are the useful part.
//
// A full /24 HTTP sweep from the ESP32 misses devices that are demonstrably present. Measured on a
// LAN with four WLED devices (.27 .52 .64 .65):
//
//   1 x 1200 ms probe   sweep past all four -> found 2, retryHits 0 (so they were full TIMEOUTS,
//                       not fast errors)
//   2 x  700 ms probes  retryHits 1, so the second attempt genuinely recovers devices — but the
//                       sweep still had found only 1 by address 99
//
// What is NOT the cause, each checked and ruled out:
//   - the devices being slow: they answer the HOST in 55-99 ms, 12/12
//   - the self-skip: /coordinator reports selfHost 66, correct
//   - the cursor skipping addresses: it advances two per pass, so odd-only snapshots are expected
//
// ACTUAL CAUSE — the async WiFi scan and the HTTP sweep were sharing one radio. A scan HOPS across
// every channel, so for most of its duration the ESP32 is not on the AP's channel at all: probes
// time out against healthy devices, ESP-NOW adverts fail, and WiFi.channel() returns the SCAN's
// channel. The serial log made it plain — "espnow: on AP channel 2 (was 9", then 8, then 12, then
// 14 — the reconciliation faithfully following a scan around the band.
//
// Fixed by making the two take turns (see the guard at the top of sweepStep, and the matching one
// in reconcileEspNowChannel). Result: a sweep that had been finding 2 of 4 devices now finds 4 of 4.
//
// CORRECTION to an earlier claim in this file's history: I wrote that "pacing changed nothing".
// That was wrong, and the completed measurement says so. Full picture, one LAN, four devices:
//
//   1 probe,  no pacing                  2/4 found,  retryHits 0
//   2 probes, no pacing                  3/4 found,  retryHits 1-2
//   2 probes + pacing                    4/4 found,  retryHits 4   <- every device needed the retry
//   2 probes + pacing + serialisation    4/4 found,  retryHits 1
//
// So all three changes contribute, but they are not equals. Serialising the scan and the sweep is
// the one that removes the CAUSE — with it, retries go from carrying every device to carrying
// almost none. Pacing and retrying were compensating for a radio that kept wandering off-channel;
// they raised the hit rate without ever explaining it, which is exactly how a workaround looks when
// you mistake it for a fix.
//
// Note the irony worth remembering — making the scan asynchronous earlier (a correct fix, since a
// blocking scan took HTTP down every 20 s) is what created the overlap that caused this. The
// blocking version hid it by never running concurrently with anything.
//
// Practical impact today: devices already in the registry are driven promptly by the known-host
// fast path, so a running installation stays in sync. It is a NEWLY onboarded device that can wait
// through a sweep or two before it is first driven. Onboarding itself is unaffected.
static bool sweepStep() {
  if (WiFi.status() != WL_CONNECTED) return true;
  // NEVER sweep while a scan is in flight. This is the real cause of the missed devices, and it is
  // not congestion: an async WiFi scan HOPS THE RADIO across every channel, so for most of the scan
  // the ESP32 is simply not on the AP's channel. HTTP probes issued in that window time out against
  // devices that are perfectly healthy, ESP-NOW sends fail ("attach: advert send failed"), and
  // WiFi.channel() returns the SCAN's current channel — which is why the channel reconciliation was
  // logging a walk through channels 2, 8, 12, 14. One radio cannot scan and hold a connection at
  // the same time; the two phases have to take turns.
  if (WiFi.scanComplete() == WIFI_SCAN_RUNNING) return false;
  IPAddress me = WiFi.localIP();
  char bodyBuf[WLED_TRANSLATE_MAX_BODY];
  size_t n = wled_body_solid(target, COORD_TARGET_BRI, bodyBuf, sizeof(bodyBuf));
  if (n == 0) { DEBUG_ERR("coord: colour body would not fit"); return true; }

  // Known devices first — they are the ones a user is waiting to see change.
  if (!syncKnownStep(bodyBuf)) return false;

  for (int i = 0; i < COORD_SWEEP_PER_PASS && sweepHost <= 254; i++, sweepHost++) {
    if (sweepHost == me[3]) continue;
    char ip[16];
    snprintf(ip, sizeof(ip), "%u.%u.%u.%u", me[0], me[1], me[2], sweepHost);
    String info;

    // One probe per address per sweep is too fragile. Measured: with four devices demonstrably on
    // the LAN — all answering the host in under 100 ms, 12/12 — consecutive sweeps found 2, then 3.
    // A device missed here waits a whole sweep (~7.5 min) for another chance, and a NEWLY onboarded
    // one is not in the registry yet so the fast path cannot cover it either.
    //
    // Retry, but only when the first attempt failed FAST. A genuinely empty address burns the full
    // COORD_DISCOVER_TIMEOUT_MS and retrying it would double the cost of the 250 addresses that are
    // silence; a transient failure (ARP miss, TCP retry, WiFi contention) typically fails in a
    // fraction of that. So the elapsed time discriminates "nothing there" from "something went
    // wrong", and the retry is paid for only in the second case.
    int rc = 0;
    for (int a = 0; a < COORD_PROBE_ATTEMPTS; a++) {
      rc = httpGetT(ip, "/json/info", &info, COORD_DISCOVER_TIMEOUT_MS);
      if (rc > 0) { if (a > 0) sweepRetryHits++; break; }
      delay(COORD_PROBE_GAP_MS);
    }
    // Pace the sweep. Back-to-back TCP connects from the ESP32 appear to starve its own WiFi task:
    // during an unpaced sweep the coordinator's ping latency rose to 380-590 ms and its own HTTP
    // went intermittent, while devices that answer the host in under 100 ms were being missed. A
    // short gap costs a few seconds across a /24 and is the cheapest thing to try before reaching
    // for longer timeouts.
    delay(COORD_PROBE_GAP_MS);
    if (rc <= 0) continue;
    if (info.indexOf("\"brand\":\"WLED\"") < 0) continue;
    sweepFound++;
    if (httpPost(ip, "/json/state", bodyBuf, nullptr) > 0) sweepSet++;
    // Read the look back AFTER setting it, so the registry records what the device actually holds
    // rather than what we asked for — the same assert-the-effect rule the tests follow.
    String stateDoc;
    if (httpGetT(ip, "/json/state", &stateDoc, COORD_DISCOVER_TIMEOUT_MS) > 0)
      registryNote(sweepHost, info, stateDoc, millis());
  }
  if (sweepHost > 254) {
    registryExpire(millis());          // a full pass has just visited every address
    st.discovered = sweepFound;
    st.synced     = sweepSet;
    DEBUG1_VALUE("coord: sync found ", sweepFound);
    DEBUG1_VALUELN(" set ", sweepSet);
    return true;
  }
  return false;
}

// --- phases ---------------------------------------------------------------------------------------
static void enter(Phase p, uint32_t now) { phase = p; phaseSince = now; st.phase = phaseName(p); }

void coordinatorSetup() {
  loadHome();
  storeHome();                 // commit BEFORE any hop can happen
  wled_pushlog_reset(&pushed);
  st.phase = phaseName(PH_BOOT);
  st.targetR = target.r; st.targetG = target.g; st.targetB = target.b;
  DEBUG1_VALUELN("coord: home ssid=", homeSsid);
  joinHome(20000);
  enter(PH_SCAN, millis());
}

void coordinatorLoop(uint32_t now) {
  switch (phase) {
    case PH_SCAN: {
      if (now - lastScanMs < COORD_SCAN_PERIOD_MS && lastScanMs != 0) { enter(PH_SYNC, now); break; }

      // ASYNCHRONOUS scan. A blocking WiFi.scanNetworks() takes seconds and stalls loop(), so
      // handleClient() does not run and every HTTP endpoint — including POST /update — goes dead
      // for the duration, every COORD_SCAN_PERIOD_MS. Observed directly: with no candidate APs in
      // range and hops=0, /coordinator answered on alternating polls and a 600 kB OTA upload could
      // not complete. That is the same failure as the router's synchronous ESP-NOW send and as the
      // original blocking discovery sweep: a long call in loop() is indistinguishable from a dead
      // device to anything talking HTTP.
      int found = WiFi.scanComplete();
      if (found == WIFI_SCAN_RUNNING) break;             // still scanning: yield, stay responsive
      if (found == WIFI_SCAN_FAILED) {                   // not started yet (or the last one failed)
        WiFi.scanNetworks(/*async=*/true, /*show_hidden=*/true);
        break;
      }
      lastScanMs = now;
      int hopped = 0;
      for (int i = 0; i < found; i++) {
        WledScanResult r;
        String ssid = WiFi.SSID(i);
        r.ssid = ssid.c_str();
        memcpy(r.bssid, WiFi.BSSID(i), 6);
        r.rssi = (int8_t)WiFi.RSSI(i);
        r.channel = (uint8_t)WiFi.channel(i);
        if (wled_is_candidate(&r, &pushed) != WLED_CAND_YES) continue;
        WiFi.scanDelete();
        if (onboardOne(r)) {
          // Force the next sync to be a FRESH sweep rather than waiting out COORD_SYNC_PERIOD_MS.
          // The settle phase exists to give this device time to join; finishing that wait and then
          // NOT looking for it wastes the whole point. Observed: a device onboarded at t+65 s was
          // missed by the sweep that completed at t+120 s, because that sweep had already walked
          // past its address before it appeared, and it then sat at factory defaults for another
          // full sweep period.
          lastSyncMs = 0;
          sweepHost  = 0;
        }
        hopped++;
        break;                 // one hop per pass: re-scan after, the AP list has changed
      }
      if (!hopped) WiFi.scanDelete();
      enter(hopped ? PH_SETTLE : PH_SYNC, now);
      break;
    }
    case PH_SETTLE: {
      // Return home and hold still so pushed devices can find the network.
      if (WiFi.status() != WL_CONNECTED) joinHome(15000);
      if (now - phaseSince >= COORD_SETTLE_MS) enter(PH_SYNC, now);
      break;
    }
    case PH_SYNC: {
      if (WiFi.status() != WL_CONNECTED) { joinHome(15000); break; }
      if (sweepHost == 0) {                       // not sweeping: is one due?
        if (lastSyncMs != 0 && now - lastSyncMs < COORD_SYNC_PERIOD_MS) { enter(PH_SCAN, now); break; }
        lastSyncMs = now;
        sweepBegin();
      }
      if (sweepStep()) {                          // finished this pass over the subnet
        sweepHost = 0;
        enter(PH_SCAN, now);
        break;
      }
      // A sweep must not delay ONBOARDING. Discovery walks 254 addresses, most of them silent, at
      // COORD_DISCOVER_TIMEOUT_MS each — minutes of wall clock. Scanning only between completed
      // sweeps meant a factory-fresh device sat in AP mode that whole time (observed: hops=0 with a
      // waiting target). So a due scan preempts the sweep; sweepHost is left where it is and the
      // sweep resumes on the next pass through PH_SYNC rather than restarting.
      if (now - lastScanMs >= COORD_SCAN_PERIOD_MS) enter(PH_SCAN, now);
      break;                                       // still sweeping: yield, keep HTTP alive
    }
    default: enter(PH_SCAN, now); break;
  }
}

CoordinatorStatus coordinatorStatus() {
  st.homeUp  = (WiFi.status() == WL_CONNECTED);
  st.selfHost = WiFi.localIP()[3];
  st.sweepAt  = sweepHost;
  st.sweepFoundNow = sweepFound;
  st.retryHits     = sweepRetryHits;
  st.targetR = target.r; st.targetG = target.g; st.targetB = target.b;
  return st;
}

void coordinatorSetTarget(uint8_t r, uint8_t g, uint8_t b) {
  target.r = r; target.g = g; target.b = b;
  lastSyncMs = 0;              // apply on the next pass rather than waiting out the period
}

#endif  // WLED_COORDINATOR_ROLE
