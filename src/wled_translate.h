#pragma once
//
// wled_translate.h — HMTL colour/program -> WLED JSON body. Pure, host-testable, no Arduino.
//
// This is the MVP of the WLED-coordinator role: a colour arrives from the HMTL side and has to
// become a /json/state body that stock WLED devices will apply.
//
// Why this is a fixed string and not a name lookup: WLED effect 0 is Solid, as a compile-time
// constant (WLED/wled00/FX.h:157, `#define FX_MODE_STATIC 0`), so it needs no per-device
// resolution. That is NOT true of effects and palettes in general — their ids depend on which
// usermods a device carries (usermod palettes are not in /json/pal at all; they live at 255-j and
// are named only in info.umpalnames). Any future capability that relays a NAMED effect or palette
// must resolve per target against that device's own tables, the way scripts/wled_sync.py does.
// Solid is the one case that escapes that, which is exactly why the MVP is built on it.
//
#include <stdint.h>
#include <stddef.h>

// A colour as it arrives from the HMTL side.
struct WledColour {
  uint8_t r, g, b;
};

// Longest body this can emit, including the NUL. The fixed skeleton is 44 chars and each channel
// is at most 3 digits, so 64 is comfortable; asserted by the tests rather than eyeballed.
#define WLED_TRANSLATE_MAX_BODY 64

// Render `c` as a /json/state body setting the whole device to that solid colour.
//
// Writes at most `cap` bytes including the NUL and returns the length written, or 0 if the buffer
// is too small — 0 means "emit nothing", never a truncated body, because a truncated JSON body
// would be applied as a partial state rather than rejected.
//
// `bri` is sent explicitly: a device left at brightness 0 shows nothing whatever the colour is, and
// "I set it to red and nothing happened" is the failure this avoids.
static inline size_t wled_body_solid(const WledColour &c, uint8_t bri, char *out, size_t cap) {
  if (out == 0 || cap == 0) return 0;

  // {"on":true,"bri":B,"seg":[{"fx":0,"col":[[R,G,B]]}]}
  static const char kPrefix[] = "{\"on\":true,\"bri\":";
  static const char kMid[]    = ",\"seg\":[{\"fx\":0,\"col\":[[";
  static const char kSuffix[] = "]]}]}";

  // Local uint->decimal so this stays free of stdio on the target.
  struct Emit {
    static size_t u8(uint8_t v, char *d) {
      size_t k = 0;
      if (v >= 100) d[k++] = (char)('0' + (v / 100));
      if (v >= 10)  d[k++] = (char)('0' + ((v / 10) % 10));
      d[k++] = (char)('0' + (v % 10));
      return k;
    }
  };

  char db[4], dr[4], dg[4], dbl[4];
  const size_t nb  = Emit::u8(bri, db);
  const size_t nr  = Emit::u8(c.r, dr);
  const size_t ng  = Emit::u8(c.g, dg);
  const size_t nbl = Emit::u8(c.b, dbl);

  // Size the whole body BEFORE writing any of it. Checking capacity as we go would leave a
  // partial body behind on failure, and a truncated JSON body gets applied as a partial state
  // rather than rejected — so the contract is all-or-nothing and this is what enforces it.
  const size_t need = (sizeof(kPrefix) - 1) + nb
                    + (sizeof(kMid) - 1) + nr + 1 + ng + 1 + nbl
                    + (sizeof(kSuffix) - 1);
  if (need + 1 > cap) return 0;   // +1 for the NUL

  size_t n = 0;
  const char *parts[] = { kPrefix, db, kMid, dr, ",", dg, ",", dbl, kSuffix };
  const size_t lens[] = { sizeof(kPrefix) - 1, nb, sizeof(kMid) - 1, nr, 1, ng, 1, nbl,
                          sizeof(kSuffix) - 1 };
  for (size_t p = 0; p < sizeof(lens) / sizeof(lens[0]); p++) {
    for (size_t i = 0; i < lens[p]; i++) out[n + i] = parts[p][i];
    n += lens[p];
  }
  out[n] = '\0';
  return n;
}
