#pragma once
//
// wled_onboard_cfg.h — build the /json/cfg body that onboards a stock WLED device. Pure, testable.
//
// Every key below was read from WLED (wled00/cfg.cpp) AND confirmed present on a live WLED 16.0.1
// via GET /json/cfg, because three of them have silent failure modes:
//
//  * "psk" is the WRITE key; a read returns "pskl", the password LENGTH only (cfg.cpp:99 vs :870).
//    Sending "pskl" is silently ignored and cfg.cpp:107 then KEEPS THE OLD PASSWORD — the device
//    reboots, fails to join, and nothing anywhere reports an error.
//  * An empty psk is also silently ignored, same line ("keep old password intact if not present").
//    So a blank password is not "no password", it is "leave it alone".
//  * linked_remote entries must be exactly 12 lowercase hex chars, no separators: the receive path
//    formats the sender with "%02x%02x%02x%02x%02x%02x" and compares with strcmp after a
//    strlen()==12 check (udp.cpp:908,921). "AA:BB:.." or uppercase silently never matches, so
//    every frame we later send is dropped as an unlinked sender.
//
#include <stdint.h>
#include <stddef.h>

struct WledMac { uint8_t b[6]; };

// Render a MAC in exactly the form linked_remotes is compared against. `out` needs 13 bytes.
static inline bool wled_mac_to_linked(const WledMac &m, char *out, size_t cap) {
  if (out == 0 || cap < 13) return false;
  static const char hx[] = "0123456789abcdef";   /* lowercase: strcmp, not case-insensitive */
  for (size_t i = 0; i < 6; i++) {
    out[i * 2]     = hx[(m.b[i] >> 4) & 0xF];
    out[i * 2 + 1] = hx[m.b[i] & 0xF];
  }
  out[12] = '\0';
  return true;
}

enum wled_cfg_result_t {
  WLED_CFG_OK = 0,
  WLED_CFG_NO_ROOM,
  WLED_CFG_BAD_SSID,      // empty: there is nothing to join
  WLED_CFG_BAD_PSK,       // empty: would silently keep the device's existing password
};

struct WledOnboardCfg {
  const char *ssid;
  const char *psk;
  WledMac     coordinator_mac;
  // Existing linked_remote entries, so we APPEND rather than replace. Replacing would silently
  // unpair whatever was already authorised on the device.
  const char *const *existing_linked;
  size_t             existing_linked_count;
  bool enable_espnow;       // nw.espnow      -> the radio itself
  bool enable_espnow_sync;  // if.sync.espnow -> follow a WLED ESP-NOW master
};

static inline size_t wled_cfg_len(const char *s) {
  size_t n = 0; if (!s) return 0; while (s[n]) n++; return n;
}

// Build the onboarding cfg body. Returns WLED_CFG_OK and writes a NUL-terminated body, or a reason.
// Nothing is written unless the whole body fits — a truncated cfg would be applied as a partial
// config, which on the network path means a device that half-joins and is then unreachable.
static inline wled_cfg_result_t wled_build_onboard_cfg(const WledOnboardCfg &c,
                                                       char *out, size_t cap,
                                                       size_t *written) {
  if (written) *written = 0;
  if (out == 0 || cap == 0) return WLED_CFG_NO_ROOM;
  if (wled_cfg_len(c.ssid) == 0) return WLED_CFG_BAD_SSID;
  if (wled_cfg_len(c.psk) == 0)  return WLED_CFG_BAD_PSK;

  char mac[13];
  if (!wled_mac_to_linked(c.coordinator_mac, mac, sizeof(mac))) return WLED_CFG_NO_ROOM;

  // Assemble into a local cursor that refuses to overflow, then commit only on success.
  size_t n = 0;
  struct P {
    static bool s(const char *t, char *o, size_t cap, size_t *n) {
      for (size_t i = 0; t[i]; i++) { if (*n + 1 >= cap) return false; o[(*n)++] = t[i]; }
      return true;
    }
  };
  bool ok = true;
  ok = ok && P::s("{\"nw\":{\"ins\":[{\"ssid\":\"", out, cap, &n);
  ok = ok && P::s(c.ssid, out, cap, &n);
  ok = ok && P::s("\",\"psk\":\"", out, cap, &n);
  ok = ok && P::s(c.psk, out, cap, &n);
  ok = ok && P::s("\"}],\"espnow\":", out, cap, &n);
  ok = ok && P::s(c.enable_espnow ? "true" : "false", out, cap, &n);
  ok = ok && P::s(",\"linked_remote\":[", out, cap, &n);
  for (size_t i = 0; ok && i < c.existing_linked_count; i++) {
    const char *e = c.existing_linked ? c.existing_linked[i] : 0;
    if (!e || wled_cfg_len(e) != 12) continue;      /* skip junk rather than emit it */
    ok = ok && P::s("\"", out, cap, &n);
    ok = ok && P::s(e, out, cap, &n);
    ok = ok && P::s("\",", out, cap, &n);
  }
  ok = ok && P::s("\"", out, cap, &n);
  ok = ok && P::s(mac, out, cap, &n);
  ok = ok && P::s("\"]},\"if\":{\"sync\":{\"espnow\":", out, cap, &n);
  ok = ok && P::s(c.enable_espnow_sync ? "true" : "false", out, cap, &n);
  ok = ok && P::s("}}}", out, cap, &n);
  if (!ok) return WLED_CFG_NO_ROOM;

  out[n] = '\0';
  if (written) *written = n;
  return WLED_CFG_OK;
}
