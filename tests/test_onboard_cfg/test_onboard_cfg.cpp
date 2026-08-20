// Host unit test for wled_onboard_cfg.h — the /json/cfg onboarding body.
//
//   c++ -std=c++11 -Wall -o /tmp/enr_onboard test_onboard_cfg.cpp && /tmp/enr_onboard
//
#include "../../src/wled_onboard_cfg.h"
#include <cstdio>
#include <cstring>

static int g_fail = 0;
#define CHECK(cond, msg) do { \
  if (!(cond)) { printf("FAIL: %s\n", msg); g_fail = 1; } \
} while (0)

int main() {
  char body[512];
  size_t n = 0;
  WledMac mac = {{0xc4, 0xdd, 0x57, 0x67, 0x16, 0x78}};

  // --- MAC formatting is exactly what linked_remotes is compared against ------------------------
  // udp.cpp:908 formats the sender with %02x...; :921 requires strlen()==12 and strcmp equality.
  {
    char m[13];
    CHECK(wled_mac_to_linked(mac, m, sizeof(m)), "formats");
    CHECK(strcmp(m, "c4dd57671678") == 0, "12 lowercase hex, no separators");
    CHECK(strlen(m) == 12, "exactly 12 chars, as the strlen check demands");
    char tiny[12];
    CHECK(!wled_mac_to_linked(mac, tiny, sizeof(tiny)), "refuses a 12-byte buffer (needs 13)");
  }

  // --- The happy path ----------------------------------------------------------------------------
  {
    WledOnboardCfg c = { "Acropolis", "hunter2hunter2", mac, 0, 0, true, true };
    CHECK(wled_build_onboard_cfg(c, body, sizeof(body), &n) == WLED_CFG_OK, "builds");
    CHECK(strstr(body, "\"psk\":\"hunter2hunter2\"") != 0,
          "uses psk (the WRITE key), never pskl");
    CHECK(strstr(body, "pskl") == 0, "never emits pskl — it is read-only and would be ignored");
    CHECK(strstr(body, "\"ssid\":\"Acropolis\"") != 0, "ssid");
    CHECK(strstr(body, "\"espnow\":true") != 0, "nw.espnow on");
    CHECK(strstr(body, "\"linked_remote\":[\"c4dd57671678\"]") != 0, "our MAC authorised");
    CHECK(strstr(body, "\"if\":{\"sync\":{\"espnow\":true}}") != 0, "if.sync.espnow on");
    CHECK(strlen(body) == n, "returned length matches");
  }

  // --- An empty psk is REFUSED, not sent ---------------------------------------------------------
  // cfg.cpp:107 keeps the old password when psk is absent or empty, so sending "" would leave the
  // device on its previous credentials and fail to join with no error anywhere.
  {
    WledOnboardCfg c = { "Acropolis", "", mac, 0, 0, true, true };
    CHECK(wled_build_onboard_cfg(c, body, sizeof(body), &n) == WLED_CFG_BAD_PSK, "empty psk refused");
    CHECK(n == 0, "and nothing was produced");
  }
  {
    WledOnboardCfg c = { "", "pw12345678", mac, 0, 0, true, true };
    CHECK(wled_build_onboard_cfg(c, body, sizeof(body), &n) == WLED_CFG_BAD_SSID, "empty ssid refused");
  }

  // --- linked_remote APPENDS; replacing would unpair whatever was already authorised -------------
  {
    const char *existing[] = { "aabbccddee01", "aabbccddee02" };
    WledOnboardCfg c = { "Net", "password1", mac, existing, 2, true, false };
    CHECK(wled_build_onboard_cfg(c, body, sizeof(body), &n) == WLED_CFG_OK, "builds with existing");
    CHECK(strstr(body, "\"aabbccddee01\"") != 0, "existing entry 1 preserved");
    CHECK(strstr(body, "\"aabbccddee02\"") != 0, "existing entry 2 preserved");
    CHECK(strstr(body, "\"c4dd57671678\"") != 0, "ours appended");
    CHECK(strstr(body, "\"espnow\":false}}") != 0, "sync flag independently off");
  }

  // --- Malformed existing entries are skipped, not emitted ---------------------------------------
  // A wrong-length entry can never match at the far end, and emitting it would silently bloat the
  // list with something that looks authorised but is not.
  {
    const char *existing[] = { "AA:BB:CC:DD:EE:01", "short", "aabbccddee03" };
    WledOnboardCfg c = { "Net", "password1", mac, existing, 3, true, true };
    CHECK(wled_build_onboard_cfg(c, body, sizeof(body), &n) == WLED_CFG_OK, "builds");
    CHECK(strstr(body, "AA:BB") == 0, "colon-form entry dropped");
    CHECK(strstr(body, "short") == 0, "wrong-length entry dropped");
    CHECK(strstr(body, "\"aabbccddee03\"") != 0, "the valid one survives");
  }

  // --- All-or-nothing on overflow ---------------------------------------------------------------
  {
    WledOnboardCfg c = { "Acropolis", "hunter2hunter2", mac, 0, 0, true, true };
    char small[40];
    memset(small, 'X', sizeof(small));
    CHECK(wled_build_onboard_cfg(c, small, sizeof(small), &n) == WLED_CFG_NO_ROOM, "refuses");
    CHECK(n == 0, "reports nothing written");
    CHECK(small[sizeof(small) - 1] == 'X', "tail untouched — no partial cfg body");
  }

  printf(g_fail ? "TESTS FAILED\n" : "ALL TESTS PASSED\n");
  return g_fail;
}
