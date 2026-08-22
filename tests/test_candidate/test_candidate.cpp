// Host unit test for wled_candidate.h — scan-result matching and push bookkeeping.
#include "../../src/wled_candidate.h"
#include <cstdio>
#include <cstring>

static int g_fail = 0;
#define CHECK(cond, msg) do { \
  if (!(cond)) { printf("FAIL: %s\n", msg); g_fail = 1; } \
} while (0)

static WledScanResult mk(const char *ssid, const uint8_t *mac, uint8_t ch) {
  WledScanResult r; r.ssid = ssid; r.rssi = -50; r.channel = ch;
  memcpy(r.bssid, mac, 6); return r;
}

int main() {
  // A real Espressif MAC observed on this bench, and a non-Espressif one.
  const uint8_t esp[6]   = {0xc4,0xdd,0x57,0x67,0x16,0x78};
  const uint8_t esp2[6]  = {0x2c,0xbc,0xbb,0xd9,0x89,0x60};
  const uint8_t other[6] = {0x00,0x1a,0x2b,0x3c,0x4d,0x5e};

  WledPushLog log; wled_pushlog_reset(&log);

  // --- The happy path, on the ACTUAL default channel ---------------------------------------------
  {
    WledScanResult r = mk("WLED-AP", esp, WLED_DEFAULT_AP_CHANNEL);
    CHECK(wled_is_candidate(&r, &log) == WLED_CAND_YES, "WLED-AP on Espressif silicon is a candidate");
    CHECK(WLED_DEFAULT_AP_CHANNEL == 6, "apChannel default is 6 (wled.h:368), NOT 1");
  }

  // --- The setup AP is password-protected; the password must be the real one ---------------------
  CHECK(strcmp(WLED_SETUP_AP_PASS, "wled1234") == 0, "join password matches DEFAULT_AP_PASS");
  CHECK(strcmp(WLED_SETUP_AP_PREFIX, "WLED-AP") == 0, "prefix matches DEFAULT_AP_SSID");

  // --- Named right, wrong silicon: someone else's AP called WLED-AP -------------------------------
  {
    WledScanResult r = mk("WLED-AP", other, 6);
    CHECK(wled_is_candidate(&r, &log) == WLED_CAND_NOT_ESPRESSIF, "non-Espressif OUI rejected");
  }

  // --- Wrong name entirely ------------------------------------------------------------------------
  {
    WledScanResult r = mk("Acropolis", esp, 6);
    CHECK(wled_is_candidate(&r, &log) == WLED_CAND_NO_SSID, "unrelated SSID rejected");
    WledScanResult r2 = mk("WLED", esp, 6);
    CHECK(wled_is_candidate(&r2, &log) == WLED_CAND_NO_SSID, "'WLED' alone is not a setup AP");
    WledScanResult r3 = mk("WLED-AP-1a2b", esp, 6);
    CHECK(wled_is_candidate(&r3, &log) == WLED_CAND_YES, "a suffixed WLED-AP still matches");
  }

  // --- The re-candidacy loop: a pushed device reappears and must NOT be re-pushed -----------------
  // In own-AP mode the target reboots, cannot find our AP (it is down during HOP) and raises its own
  // fallback AP again — indistinguishable from a fresh candidate without this log.
  {
    WledScanResult r = mk("WLED-AP", esp, 6);
    CHECK(wled_is_candidate(&r, &log) == WLED_CAND_YES, "first sighting is a candidate");
    CHECK(wled_pushlog_add(&log, esp), "recorded after pushing");
    CHECK(wled_is_candidate(&r, &log) == WLED_CAND_ALREADY_PUSHED, "reappearance is NOT re-pushed");

    WledScanResult r2 = mk("WLED-AP", esp2, 6);
    CHECK(wled_is_candidate(&r2, &log) == WLED_CAND_YES, "a DIFFERENT device is still a candidate");
    CHECK(wled_pushlog_add(&log, esp2), "recorded too");
    CHECK(log.count == 2, "two distinct entries");
    CHECK(wled_pushlog_add(&log, esp), "re-adding is idempotent");
    CHECK(log.count == 2, "no duplicate");
  }

  // --- A full log REPORTS failure; silently forgetting restarts the loop it exists to break -------
  {
    WledPushLog full; wled_pushlog_reset(&full);
    uint8_t m[6] = {0xc4,0xdd,0x57,0,0,0};
    for (uint8_t i = 0; i < WLED_MAX_PUSHED; i++) { m[5] = i; CHECK(wled_pushlog_add(&full, m), "fills"); }
    m[5] = 0xFF;
    CHECK(!wled_pushlog_add(&full, m), "full log reports failure");
    WledScanResult r = mk("WLED-AP", m, 6);
    CHECK(wled_is_candidate(&r, &full) == WLED_CAND_YES,
          "and the unrecordable target still reads as a candidate — caller must handle the false");
  }

  // --- A reset clears the round -------------------------------------------------------------------
  {
    wled_pushlog_reset(&log);
    WledScanResult r = mk("WLED-AP", esp, 6);
    CHECK(wled_is_candidate(&r, &log) == WLED_CAND_YES, "a new round re-admits a previous target");
  }

  // --- Degenerate ---------------------------------------------------------------------------------
  CHECK(wled_is_candidate(0, &log) == WLED_CAND_NO_SSID, "null scan result");
  { WledScanResult r = mk("WLED-AP", esp, 6); r.ssid = 0;
    CHECK(wled_is_candidate(&r, &log) == WLED_CAND_NO_SSID, "null ssid"); }
  CHECK(!wled_oui_is_espressif(0), "null bssid is not Espressif");

  printf(g_fail ? "TESTS FAILED\n" : "ALL TESTS PASSED\n");
  return g_fail;
}
