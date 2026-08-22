#pragma once
//
// wled_candidate.h — decide whether a scan result is a WLED setup AP worth hopping to, and track
// which targets have already been pushed. Pure, host-testable.
//
// Values read from WLED (wled00/const.h, wled00/wled.h), not assumed:
//
//   DEFAULT_AP_SSID  WLED_BRAND "-AP" -> "WLED-AP"   const.h:47, :39
//   DEFAULT_AP_PASS  "wled1234"                       const.h:48
//   apChannel        _INIT(6)                         wled.h:368
//
// Two of those contradict what the plan assumed, and both would have cost bench time:
//
//   * The setup AP is PASSWORD-PROTECTED, not open. Filtering candidates on "open auth" finds
//     nothing at all. The join must supply "wled1234".
//   * apChannel defaults to 6, not 1. The channel-drag consequence is identical either way, but a
//     hard-coded 1 would look for the AP on the wrong channel.
//
#include <stdint.h>
#include <stddef.h>

#define WLED_SETUP_AP_PREFIX "WLED-AP"
#define WLED_SETUP_AP_PASS   "wled1234"
#define WLED_DEFAULT_AP_CHANNEL 6

struct WledScanResult {
  const char *ssid;
  uint8_t     bssid[6];
  int8_t      rssi;
  uint8_t     channel;
};

// Espressif OUI prefixes, used only as a PRE-FILTER to avoid hopping to something that merely
// shares the name. It is not an identity check: the real confirmation is reading brand == "WLED"
// from /json/info after joining, which this cannot do from a scan.
static inline bool wled_oui_is_espressif(const uint8_t *bssid) {
  if (!bssid) return false;
  static const uint8_t kOui[][3] = {
    {0x24,0x0a,0xc4},{0x24,0x62,0xab},{0x24,0x6f,0x28},{0x24,0xb2,0xde},
    {0x2c,0xbc,0xbb},{0x30,0xae,0xa4},{0x3c,0x61,0x05},{0x3c,0x71,0xbf},
    {0x48,0x3f,0xda},{0x4c,0x11,0xae},{0x54,0x43,0xb2},{0x58,0xbf,0x25},
    {0x5c,0xcf,0x7f},{0x60,0x01,0x94},{0x68,0xc6,0x3a},{0x7c,0x9e,0xbd},
    {0x7c,0xdf,0xa1},{0x80,0x7d,0x3a},{0x84,0x0d,0x8e},{0x84,0xcc,0xa8},
    {0x8c,0xaa,0xb5},{0x90,0x38,0x0c},{0x94,0xb5,0x55},{0x98,0xf4,0xab},
    {0xa0,0x20,0xa6},{0xa4,0x7b,0x9d},{0xa4,0xcf,0x12},{0xac,0x67,0xb2},
    {0xb4,0xe6,0x2d},{0xbc,0xdd,0xc2},{0xc4,0x4f,0x33},{0xc4,0xdd,0x57},
    {0xcc,0x50,0xe3},{0xd8,0xa0,0x1d},{0xdc,0x4f,0x22},{0xe0,0x98,0x06},
    {0xe8,0xdb,0x84},{0xec,0xfa,0xbc},{0xf0,0x08,0xd1},{0xfc,0xf5,0xc4},
  };
  for (size_t i = 0; i < sizeof(kOui) / sizeof(kOui[0]); i++) {
    if (bssid[0] == kOui[i][0] && bssid[1] == kOui[i][1] && bssid[2] == kOui[i][2]) return true;
  }
  return false;
}

static inline bool wled_str_starts_with(const char *s, const char *prefix) {
  if (!s || !prefix) return false;
  for (size_t i = 0; prefix[i]; i++) if (s[i] != prefix[i]) return false;
  return true;
}

enum wled_candidate_verdict_t {
  WLED_CAND_YES = 0,
  WLED_CAND_NO_SSID,        // not named like a WLED setup AP
  WLED_CAND_NOT_ESPRESSIF,  // right name, wrong silicon — someone else's AP
  WLED_CAND_ALREADY_PUSHED, // we have already onboarded this one this round
};

// Bookkeeping of targets already pushed, so own-AP mode's re-candidacy loop terminates.
//
// The loop it prevents: the coordinator hops to a target, pushes credentials, and returns home. The
// target reboots, cannot find the coordinator's AP yet (it is down during the hop phase), and raises
// its OWN fallback AP again — which looks exactly like a fresh candidate on the next scan. Without
// this, the coordinator re-pushes the same device forever and never reaches the settle phase.
#ifndef WLED_MAX_PUSHED
#define WLED_MAX_PUSHED 16
#endif

struct WledPushLog {
  uint8_t macs[WLED_MAX_PUSHED][6];
  uint8_t count;
};

static inline void wled_pushlog_reset(WledPushLog *l) { if (l) l->count = 0; }

static inline bool wled_pushlog_contains(const WledPushLog *l, const uint8_t *bssid) {
  if (!l || !bssid) return false;
  for (uint8_t i = 0; i < l->count; i++) {
    bool same = true;
    for (size_t b = 0; b < 6; b++) if (l->macs[i][b] != bssid[b]) { same = false; break; }
    if (same) return true;
  }
  return false;
}

// Returns false when full. Full is reported, not ignored: silently forgetting a push is what
// restarts the re-candidacy loop this log exists to break.
static inline bool wled_pushlog_add(WledPushLog *l, const uint8_t *bssid) {
  if (!l || !bssid) return false;
  if (wled_pushlog_contains(l, bssid)) return true;
  if (l->count >= WLED_MAX_PUSHED) return false;
  for (size_t b = 0; b < 6; b++) l->macs[l->count][b] = bssid[b];
  l->count++;
  return true;
}

static inline wled_candidate_verdict_t wled_is_candidate(const WledScanResult *r,
                                                         const WledPushLog *pushed) {
  if (!r || !r->ssid) return WLED_CAND_NO_SSID;
  if (!wled_str_starts_with(r->ssid, WLED_SETUP_AP_PREFIX)) return WLED_CAND_NO_SSID;
  if (!wled_oui_is_espressif(r->bssid)) return WLED_CAND_NOT_ESPRESSIF;
  if (wled_pushlog_contains(pushed, r->bssid)) return WLED_CAND_ALREADY_PUSHED;
  return WLED_CAND_YES;
}
