// Host unit test for hmtl_to_wled.h — HMTL OUTPUT wire bytes -> colour.
//
//   c++ -std=c++11 -Wall -o /tmp/enr_hmtl2wled test_hmtl_to_wled.cpp && /tmp/enr_hmtl2wled
//
// The frames here are hand-assembled from the layout in HMTLWireFormat.h rather than produced by
// the parser's own constants, so a wrong offset in the header fails instead of agreeing with itself.
//
#include "../../src/hmtl_to_wled.h"
#include "../../src/wled_translate.h"
#include <cstdio>
#include <cstring>

static int g_fail = 0;
#define CHECK(cond, msg) do { \
  if (!(cond)) { printf("FAIL: %s\n", msg); g_fail = 1; } \
} while (0)

// A well-formed 13-byte RGB OUTPUT frame, written out byte by byte.
static void make_rgb(uint8_t *f, uint16_t addr, uint8_t output,
                     uint8_t r, uint8_t g, uint8_t b) {
  f[0] = 0xFC;                 // startcode
  f[1] = 0x00;                 // crc (not checked here)
  f[2] = 2;                    // version
  f[3] = 13;                   // length == whole frame
  f[4] = 0x01;                 // MSG_TYPE_OUTPUT
  f[5] = 0x00;                 // flags
  f[6] = (uint8_t)(addr & 0xFF);
  f[7] = (uint8_t)(addr >> 8);
  f[8] = 0x02;                 // HMTL_OUTPUT_RGB
  f[9] = output;
  f[10] = r; f[11] = g; f[12] = b;
}

int main() {
  uint8_t f[13];
  hmtl_rgb_msg_t m;

  // --- The happy path, including little-endian address ------------------------------------------
  {
    make_rgb(f, 0x1234, HMTL_WIRE_ALL_OUTPUTS, 255, 0, 0);
    CHECK(hmtl_parse_rgb(f, sizeof(f), &m) == HMTL_PARSE_OK, "well-formed frame parses");
    CHECK(m.address == 0x1234, "address is little-endian");
    CHECK(m.output == HMTL_WIRE_ALL_OUTPUTS, "output sentinel preserved");
    CHECK(m.r == 255 && m.g == 0 && m.b == 0, "colour extracted");
    CHECK(hmtl_rgb_is_fleet_wide(&m), "ALL_OUTPUTS is fleet-wide");
  }

  // --- End to end: wire bytes -> the body actually sent -----------------------------------------
  {
    make_rgb(f, 129, HMTL_WIRE_ALL_OUTPUTS, 0, 128, 64);
    CHECK(hmtl_parse_rgb(f, sizeof(f), &m) == HMTL_PARSE_OK, "e2e parses");
    WledColour c = { m.r, m.g, m.b };
    char body[WLED_TRANSLATE_MAX_BODY];
    CHECK(wled_body_solid(c, 255, body, sizeof(body)) > 0, "e2e renders");
    CHECK(strcmp(body, "{\"on\":true,\"bri\":255,\"seg\":[{\"fx\":0,\"col\":[[0,128,64]]}]}") == 0,
          "e2e body carries the HMTL colour");
  }

  // --- A specific output is NOT fleet-wide -------------------------------------------------------
  // Guessing here would turn one module's cue into a whole-installation colour change.
  {
    make_rgb(f, 1, 3, 10, 20, 30);
    CHECK(hmtl_parse_rgb(f, sizeof(f), &m) == HMTL_PARSE_OK, "specific-output frame still parses");
    CHECK(!hmtl_rgb_is_fleet_wide(&m), "output 3 is not fleet-wide");
    make_rgb(f, 1, HMTL_WIRE_NO_OUTPUT, 10, 20, 30);
    hmtl_parse_rgb(f, sizeof(f), &m);
    CHECK(!hmtl_rgb_is_fleet_wide(&m), "NO_OUTPUT is not fleet-wide either");
  }

  // --- Malformed frames are reported, never silently black ---------------------------------------
  {
    make_rgb(f, 1, HMTL_WIRE_ALL_OUTPUTS, 1, 2, 3);
    uint8_t bad[13];

    memcpy(bad, f, 13); bad[0] = 0xAB;
    CHECK(hmtl_parse_rgb(bad, 13, &m) == HMTL_PARSE_BAD_START, "bad startcode");

    memcpy(bad, f, 13); bad[2] = 99;
    CHECK(hmtl_parse_rgb(bad, 13, &m) == HMTL_PARSE_BAD_VERSION, "bad version");

    memcpy(bad, f, 13); bad[4] = 0x02;   // MSG_TYPE_POLL
    CHECK(hmtl_parse_rgb(bad, 13, &m) == HMTL_PARSE_NOT_OUTPUT, "not an OUTPUT message");

    memcpy(bad, f, 13); bad[8] = 0x01;   // HMTL_OUTPUT_VALUE
    CHECK(hmtl_parse_rgb(bad, 13, &m) == HMTL_PARSE_NOT_RGB, "OUTPUT but not RGB");

    // A header claiming a different length than the buffer: without this check a short frame
    // would be accepted and stale tail bytes read as colour.
    memcpy(bad, f, 13); bad[3] = 9;
    CHECK(hmtl_parse_rgb(bad, 13, &m) == HMTL_PARSE_LENGTH_MISMATCH, "header length disagrees");
  }

  // --- Truncation at every length must never read past the buffer --------------------------------
  {
    make_rgb(f, 1, HMTL_WIRE_ALL_OUTPUTS, 1, 2, 3);
    for (size_t n = 0; n < 13; n++) {
      hmtl_rgb_msg_t sink;
      hmtl_parse_result_t r = hmtl_parse_rgb(f, n, &sink);
      CHECK(r != HMTL_PARSE_OK, "truncated frame never parses OK");
    }
  }

  // --- Degenerate arguments ----------------------------------------------------------------------
  CHECK(hmtl_parse_rgb(0, 13, &m) == HMTL_PARSE_TOO_SHORT, "null buffer");
  CHECK(hmtl_parse_rgb(f, 13, 0) == HMTL_PARSE_TOO_SHORT, "null out");
  CHECK(!hmtl_rgb_is_fleet_wide(0), "null message is not fleet-wide");

  printf(g_fail ? "TESTS FAILED\n" : "ALL TESTS PASSED\n");
  return g_fail;
}
