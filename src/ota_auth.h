#pragma once
// ota_auth.h — minimal sender authentication for OTA and commands.
//
// The goal Adam set: stop this mechanism being trivially usable by someone who should not have it.
// Explicitly NOT internet-grade security. What it must actually achieve, and the two traps in the
// obvious design:
//
// 1. THE MAC IS NOT A SECRET. It is broadcast in every frame and is settable in software, so it
//    contributes nothing an attacker cannot supply. It is an identifier. All of the strength here
//    comes from the shared password, and the scheme is designed assuming every MAC is known.
//
// 2. A SIGNATURE OVER A FIXED INPUT IS REPLAYABLE. Sign only the password (or the password and the
//    MAC) and the resulting token is a bearer credential forever: capture one upload and it can be
//    replayed at will. So the signature must cover something that changes per request AND something
//    that identifies the payload.
//
// The construction:
//
//    signature = HMAC-SHA256(secret, nonce || ":" || sha256_hex(payload))
//
//  - `nonce` is issued by the DEVICE, single-use, short-lived. It is what makes a captured
//    signature worthless the second time.
//  - `sha256_hex(payload)` binds the signature to those exact bytes, so a valid signature cannot be
//    lifted onto a different firmware image. This matters more than authenticating the connection:
//    it survives the upload being proxied, retried or resumed.
//  - The secret is never transmitted.
//
// Comparison is CONSTANT TIME. A byte-at-a-time early-exit compare leaks the correct prefix through
// timing, which turns a 256-bit search into a 32-step one.
//
// This header is pure: hex helpers, the nonce lifecycle and the comparison. The HMAC itself is
// supplied by the caller (mbedtls on the ESP32, a test double on the host), so all of the policy
// here is host-testable without a crypto library.
#include <stdint.h>
#include <string.h>

#define OTA_AUTH_NONCE_LEN     16          // bytes of randomness, hex-encoded to 32 chars
#define OTA_AUTH_NONCE_HEX     (OTA_AUTH_NONCE_LEN * 2 + 1)
#define OTA_AUTH_SIG_LEN       32          // HMAC-SHA256 output
#define OTA_AUTH_SIG_HEX       (OTA_AUTH_SIG_LEN * 2 + 1)
#ifndef OTA_AUTH_NONCE_TTL_MS
#define OTA_AUTH_NONCE_TTL_MS  60000       // a minute is ample to push an image, short enough that
#endif                                     // a captured nonce is useless by the time it is reused

static inline void ota_auth_hex(const uint8_t *in, size_t n, char *out) {
  static const char *H = "0123456789abcdef";
  for (size_t i = 0; i < n; i++) { out[2*i] = H[(in[i] >> 4) & 0xF]; out[2*i+1] = H[in[i] & 0xF]; }
  out[2*n] = 0;
}

// Constant-time equality over `n` bytes. Accumulates differences instead of returning early, so the
// time taken does not depend on WHERE the first mismatch is. Without this, an attacker times
// responses to learn the correct signature one byte at a time, turning a 2^256 search into 32
// sequential ones.
//
// NOTE FOR REVIEWERS, because the test suite cannot help you here: this property is INVISIBLE to
// functional tests. An early-returning memcmp is functionally identical and passes every assertion
// in test_ota_auth (confirmed by mutation — that mutant survives). The only defences are reading
// this function and not "optimising" the loop into a short-circuit.
static inline bool ota_auth_equal_ct(const char *a, const char *b, size_t n) {
  if (!a || !b) return false;
  uint8_t diff = 0;
  for (size_t i = 0; i < n; i++) diff |= (uint8_t)(a[i] ^ b[i]);
  return diff == 0;
}

// One outstanding challenge. A single slot rather than a pool: concurrent uploads to one device are
// not a case worth supporting, and every extra live nonce is another window an attacker can race.
struct OtaAuthNonce {
  char     hex[OTA_AUTH_NONCE_HEX];
  uint32_t issuedMs;
  bool     live;
};

static inline void ota_auth_nonce_clear(OtaAuthNonce *n) { if (n) { n->live = false; n->hex[0] = 0; } }

static inline void ota_auth_nonce_issue(OtaAuthNonce *n, const uint8_t *random16, uint32_t nowMs) {
  if (!n || !random16) return;
  ota_auth_hex(random16, OTA_AUTH_NONCE_LEN, n->hex);
  n->issuedMs = nowMs;
  n->live     = true;
}

enum ota_auth_result_t {
  OTA_AUTH_OK = 0,
  OTA_AUTH_NO_NONCE,      // nothing outstanding: the caller never asked for a challenge
  OTA_AUTH_EXPIRED,       // took too long — issue a fresh one
  OTA_AUTH_MISMATCH,      // wrong secret, or the payload is not the one that was signed
  OTA_AUTH_MALFORMED,     // missing/short signature
};

// Verify a presented signature against the expected one, then BURN the nonce.
//
// The nonce is consumed on every outcome, not just success. Leaving it live after a failure turns
// one challenge into an unlimited number of guesses against the same target string, which is the
// whole reason single-use matters.
static inline ota_auth_result_t ota_auth_verify(OtaAuthNonce *n, const char *presentedHex,
                                                const char *expectedHex, uint32_t nowMs) {
  if (!n || !n->live)                       return OTA_AUTH_NO_NONCE;
  if (!presentedHex || !expectedHex)      { ota_auth_nonce_clear(n); return OTA_AUTH_MALFORMED; }
  if (strlen(presentedHex) != OTA_AUTH_SIG_LEN * 2) { ota_auth_nonce_clear(n); return OTA_AUTH_MALFORMED; }
  // Unsigned modular subtraction, which is the correct idiom for millis() deltas: uint32_t
  // arithmetic wraps, so (now - issued) is the true elapsed time even across the ~49.7-day
  // rollover. (An earlier version cast both sides to int32_t with a comment claiming unsigned
  // would break here. That was wrong — mutation testing showed the two behave identically — and
  // the signed form is actually the worse of the two, since it misbehaves once the delta exceeds
  // 2^31 where the unsigned form does not.)
  if ((uint32_t)(nowMs - n->issuedMs) >= (uint32_t)OTA_AUTH_NONCE_TTL_MS) {
    ota_auth_nonce_clear(n);
    return OTA_AUTH_EXPIRED;
  }
  bool ok = ota_auth_equal_ct(presentedHex, expectedHex, OTA_AUTH_SIG_LEN * 2);
  ota_auth_nonce_clear(n);
  return ok ? OTA_AUTH_OK : OTA_AUTH_MISMATCH;
}

// The exact string that gets HMAC'd, assembled in one place so the device and any client tool
// cannot disagree about it. Returns the length, or 0 if it would not fit.
static inline size_t ota_auth_message(const char *nonceHex, const char *payloadSha256Hex,
                                      char *out, size_t cap) {
  if (!nonceHex || !payloadSha256Hex || !out) return 0;
  size_t a = strlen(nonceHex), b = strlen(payloadSha256Hex);
  if (a + 1 + b + 1 > cap) return 0;
  memcpy(out, nonceHex, a);
  out[a] = ':';
  memcpy(out + a + 1, payloadSha256Hex, b);
  out[a + 1 + b] = 0;
  return a + 1 + b;
}
