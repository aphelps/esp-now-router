// Host unit test for wled_translate.h — HMTL colour -> WLED /json/state body.
//
// Builds with a normal host compiler (no Arduino/WLED):
//   c++ -std=c++11 -Wall -o /tmp/enr_translate test_translate.cpp && /tmp/enr_translate
// Exits 0 on success, 1 on the first failed assertion.
//
#include "../../src/wled_translate.h"
#include <cstdio>
#include <cstring>

static int g_fail = 0;
#define CHECK(cond, msg) do { \
  if (!(cond)) { printf("FAIL: %s\n", msg); g_fail = 1; } \
} while (0)

int main() {
  char buf[WLED_TRANSLATE_MAX_BODY];

  // --- The acceptance case: solid red -------------------------------------------------------
  {
    WledColour red = {255, 0, 0};
    size_t n = wled_body_solid(red, 255, buf, sizeof(buf));
    CHECK(n > 0, "red: emitted something");
    CHECK(strcmp(buf, "{\"on\":true,\"bri\":255,\"seg\":[{\"fx\":0,\"col\":[[255,0,0]]}]}") == 0,
          "red: exact body");
    CHECK(strlen(buf) == n, "red: returned length matches strlen");
  }

  // --- fx:0 is the whole reason this needs no per-device lookup -------------------------------
  {
    WledColour c = {1, 2, 3};
    wled_body_solid(c, 128, buf, sizeof(buf));
    CHECK(strstr(buf, "\"fx\":0") != 0, "always selects effect 0 (Solid)");
  }

  // --- Brightness is explicit, not inherited ---------------------------------------------------
  // A device sitting at bri 0 shows nothing whatever the colour is.
  {
    WledColour c = {255, 255, 255};
    wled_body_solid(c, 7, buf, sizeof(buf));
    CHECK(strstr(buf, "\"bri\":7") != 0, "brightness is sent explicitly");
  }

  // --- Digit widths: 1, 2 and 3 digit channels all render ------------------------------------
  {
    WledColour c = {0, 10, 200};
    wled_body_solid(c, 9, buf, sizeof(buf));
    CHECK(strstr(buf, "[[0,10,200]]") != 0, "1/2/3-digit channels");
  }

  // --- The declared maximum really is enough --------------------------------------------------
  {
    WledColour c = {255, 255, 255};
    size_t n = wled_body_solid(c, 255, buf, sizeof(buf));
    CHECK(n > 0, "worst case fits in WLED_TRANSLATE_MAX_BODY");
    CHECK(n < WLED_TRANSLATE_MAX_BODY, "worst case leaves room for the NUL");
  }

  // --- Too-small buffer emits NOTHING, never a truncated body ---------------------------------
  // A truncated JSON body would be applied as a partial state rather than rejected, so the
  // contract is 0-or-complete. Assert the buffer is untouched, not merely that 0 came back.
  {
    WledColour c = {255, 255, 255};
    char small[20];
    memset(small, 'X', sizeof(small));
    size_t n = wled_body_solid(c, 255, small, sizeof(small));
    CHECK(n == 0, "too-small buffer returns 0");
    bool untouched = true;
    for (size_t i = 0; i < sizeof(small); i++) if (small[i] != 'X') untouched = false;
    CHECK(untouched, "too-small buffer is left untouched, no partial write");
  }

  // --- Degenerate arguments --------------------------------------------------------------------
  {
    WledColour c = {1, 1, 1};
    CHECK(wled_body_solid(c, 1, 0, 10) == 0, "null out returns 0");
    CHECK(wled_body_solid(c, 1, buf, 0) == 0, "zero cap returns 0");
  }

  printf(g_fail ? "TESTS FAILED\n" : "ALL TESTS PASSED\n");
  return g_fail;
}
