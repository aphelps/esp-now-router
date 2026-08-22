// Host test for the OTA/command authentication policy (ota_auth.h). No crypto library needed: the
// HMAC is the caller's job, so everything decided here — nonce lifecycle, replay, expiry, the
// constant-time compare and the signed message construction — is testable on the host.
//
//   c++ -std=c++11 -Wall -Wextra -o /tmp/ota_auth_test tests/test_ota_auth/test_ota_auth.cpp
//
#include "../../src/ota_auth.h"
#include <cstdio>
#include <cstring>
#include <string>

static int g_fail = 0;
#define CHECK(cond, msg) do { \
  if (!(cond)) { printf("FAIL: %s (line %d)\n", msg, __LINE__); g_fail = 1; } \
} while (0)

static const char *SIG_A = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"; // 64
static const char *SIG_B = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaab";

int main() {
  uint8_t rnd[OTA_AUTH_NONCE_LEN];
  for (int i = 0; i < OTA_AUTH_NONCE_LEN; i++) rnd[i] = (uint8_t)(i * 7 + 1);

  // --- the property the whole design exists for: a signature cannot be replayed ---------------
  {
    OtaAuthNonce n{}; ota_auth_nonce_issue(&n, rnd, 1000);
    CHECK(ota_auth_verify(&n, SIG_A, SIG_A, 1100) == OTA_AUTH_OK, "a correct signature verifies");
    // Same signature, immediately again. If this ever passes, capturing one upload gives an
    // attacker an unlimited licence.
    CHECK(ota_auth_verify(&n, SIG_A, SIG_A, 1101) == OTA_AUTH_NO_NONCE,
          "the SAME signature is refused the second time — the nonce was burned");
  }

  // --- a failed attempt must also burn the nonce ------------------------------------------------
  {
    OtaAuthNonce n{}; ota_auth_nonce_issue(&n, rnd, 1000);
    CHECK(ota_auth_verify(&n, SIG_B, SIG_A, 1100) == OTA_AUTH_MISMATCH, "wrong signature rejected");
    // Leaving the nonce live after a failure turns one challenge into unlimited guesses against a
    // fixed target string. That is the entire value of single-use.
    CHECK(ota_auth_verify(&n, SIG_A, SIG_A, 1101) == OTA_AUTH_NO_NONCE,
          "a WRONG guess also burns the nonce, so guesses cannot be stacked");
  }

  // --- expiry, including the millis() wrap ------------------------------------------------------
  {
    OtaAuthNonce n{}; ota_auth_nonce_issue(&n, rnd, 1000);
    CHECK(ota_auth_verify(&n, SIG_A, SIG_A, 1000 + OTA_AUTH_NONCE_TTL_MS - 1) == OTA_AUTH_OK,
          "valid one ms before expiry");
    ota_auth_nonce_issue(&n, rnd, 1000);
    CHECK(ota_auth_verify(&n, SIG_A, SIG_A, 1000 + OTA_AUTH_NONCE_TTL_MS) == OTA_AUTH_EXPIRED,
          "expired exactly on the deadline");

    // These confirm expiry still works across the millis() rollover. They do NOT distinguish signed
    // from unsigned arithmetic — mutation showed both pass — because uint32_t subtraction already
    // wraps correctly. Kept because the rollover is still worth pinning; the earlier comment here
    // claiming unsigned would break was wrong.
    OtaAuthNonce w{}; const uint32_t nearMax = 0xFFFFFF00u;
    ota_auth_nonce_issue(&w, rnd, nearMax);
    CHECK(ota_auth_verify(&w, SIG_A, SIG_A, nearMax + 100) == OTA_AUTH_OK, "fresh across the wrap");
    ota_auth_nonce_issue(&w, rnd, nearMax);
    CHECK(ota_auth_verify(&w, SIG_A, SIG_A, nearMax + OTA_AUTH_NONCE_TTL_MS) == OTA_AUTH_EXPIRED,
          "and expires correctly across the wrap");
  }

  // --- nothing is accepted without a challenge --------------------------------------------------
  {
    OtaAuthNonce n{};
    CHECK(ota_auth_verify(&n, SIG_A, SIG_A, 1000) == OTA_AUTH_NO_NONCE,
          "a signature with no outstanding nonce is refused");
  }

  // --- malformed input --------------------------------------------------------------------------
  {
    OtaAuthNonce n{}; ota_auth_nonce_issue(&n, rnd, 1000);
    CHECK(ota_auth_verify(&n, "short", SIG_A, 1100) == OTA_AUTH_MALFORMED, "a short signature is refused");
    ota_auth_nonce_issue(&n, rnd, 1000);
    CHECK(ota_auth_verify(&n, nullptr, SIG_A, 1100) == OTA_AUTH_MALFORMED, "a missing signature is refused");
  }

  // --- constant-time compare --------------------------------------------------------------------
  {
    // These check CORRECTNESS only. Be clear about what is not covered: the constant-time property
    // is invisible to this suite — replacing the accumulating loop with an early-returning memcmp
    // passes every assertion below (verified by mutation). Nothing here defends the timing
    // behaviour; only code review does. Recorded so a future reader does not mistake a green suite
    // for evidence that the compare is still constant-time.
    CHECK(ota_auth_equal_ct(SIG_A, SIG_A, 64), "identical strings compare equal");
    CHECK(!ota_auth_equal_ct(SIG_A, SIG_B, 64), "a difference in the LAST byte is caught");
    std::string early = SIG_A; early[0] = 'b';
    CHECK(!ota_auth_equal_ct(early.c_str(), SIG_A, 64), "a difference in the FIRST byte is caught");
  }

  // --- the signed message binds nonce AND payload ------------------------------------------------
  {
    char msg[128];
    size_t n = ota_auth_message("abc123", "deadbeef", msg, sizeof(msg));
    CHECK(n == 15, "message length is nonce + ':' + hash");
    CHECK(strcmp(msg, "abc123:deadbeef") == 0, "message is nonce:payloadhash");

    // The payload hash must be in there. Without it a signature authenticates the SENDER but not
    // the BYTES, so a valid signature could be lifted onto a different firmware image.
    char other[128];
    ota_auth_message("abc123", "cafebabe", other, sizeof(other));
    CHECK(strcmp(msg, other) != 0, "a different payload gives a different signed message");

    // And the nonce must be in there, or every request for the same image signs identically.
    char n2[128];
    ota_auth_message("zzz999", "deadbeef", n2, sizeof(n2));
    CHECK(strcmp(msg, n2) != 0, "a different nonce gives a different signed message");

    char tiny[8];
    CHECK(ota_auth_message("abc123", "deadbeef", tiny, sizeof(tiny)) == 0,
          "refuses to truncate rather than signing a partial message");
  }

  // --- the nonce is actually random-derived, not a counter ---------------------------------------
  {
    OtaAuthNonce a{}, b{};
    uint8_t r2[OTA_AUTH_NONCE_LEN];
    for (int i = 0; i < OTA_AUTH_NONCE_LEN; i++) r2[i] = (uint8_t)(255 - i);
    ota_auth_nonce_issue(&a, rnd, 1000);
    ota_auth_nonce_issue(&b, r2, 1000);
    CHECK(strcmp(a.hex, b.hex) != 0, "different entropy gives different nonces");
    CHECK(strlen(a.hex) == OTA_AUTH_NONCE_LEN * 2, "nonce is fully hex-encoded");
  }

  printf(g_fail ? "SOME TESTS FAILED\n" : "ALL TESTS PASSED\n");
  return g_fail;
}
