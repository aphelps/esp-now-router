// Host unit test for coordinator_mode.h — the hop-permission rules.
//
//   c++ -std=c++11 -Wall -o /tmp/enr_coord_mode test_coord_mode.cpp && /tmp/enr_coord_mode
//
// Each rule is asserted through coord_may_hop()/coord_hop_begin() — the decisions the radio glue
// actually consults — not through the struct fields, so a refactor that keeps the fields but stops
// consulting them fails here.
//
#include "../../src/coordinator_mode.h"
#include <cstdio>

static int g_fail = 0;
#define CHECK(cond, msg) do { \
  if (!(cond)) { printf("FAIL: %s\n", msg); g_fail = 1; } \
} while (0)

int main() {
  const uint64_t A = 0xAABBCCDDEE01ULL, B = 0xAABBCCDDEE02ULL;

  // --- Rule 1: never hop while operating. The load-bearing one. -------------------------------
  {
    coord_state_t s; coord_init(&s, 30000);
    CHECK(coord_may_hop(&s, A) == COORD_HOP_PERMIT, "onboard mode permits a hop");
    coord_enter_operate(&s);
    CHECK(coord_may_hop(&s, A) == COORD_HOP_REFUSE_OPERATING, "operate mode refuses");
    CHECK(!coord_hop_begin(&s, A, 1000), "hop_begin refuses in operate mode");
    // Refuse for EVERY target, not just the one asked about first.
    CHECK(coord_may_hop(&s, B) == COORD_HOP_REFUSE_OPERATING, "operate refuses any target");
    coord_enter_onboard(&s);
    CHECK(coord_may_hop(&s, A) == COORD_HOP_PERMIT, "back to onboard permits again");
  }

  // --- Entering operate must clear a hop in flight ----------------------------------------------
  // Otherwise hop_in_flight stays set forever and every later onboard-mode hop is refused.
  {
    coord_state_t s; coord_init(&s, 30000);
    CHECK(coord_hop_begin(&s, A, 100), "hop begins");
    coord_enter_operate(&s);
    coord_enter_onboard(&s);
    CHECK(coord_may_hop(&s, A) == COORD_HOP_PERMIT, "mode round-trip does not strand hop_in_flight");
  }

  // --- Rule 2: hops are bounded --------------------------------------------------------------
  {
    coord_state_t s; coord_init(&s, 30000);
    coord_hop_begin(&s, A, 1000);
    CHECK(!coord_hop_expired(&s, 1000), "not expired at t=0");
    CHECK(!coord_hop_expired(&s, 30999), "not expired just under budget");
    CHECK(coord_hop_expired(&s, 31000), "expired exactly at budget");
    CHECK(coord_hop_expired(&s, 99999), "still expired later");
    coord_hop_end(&s);
    CHECK(!coord_hop_expired(&s, 99999), "no hop in flight cannot expire");
  }

  // --- millis() rollover must not grant an unbounded hop ---------------------------------------
  // Start just before the ~49-day wrap and confirm the deadline still lands.
  {
    coord_state_t s; coord_init(&s, 30000);
    const uint32_t near_wrap = 0xFFFFF000u;
    coord_hop_begin(&s, A, near_wrap);
    CHECK(!coord_hop_expired(&s, near_wrap + 100), "pre-wrap: not yet expired");
    CHECK(coord_hop_expired(&s, (uint32_t)(near_wrap + 30000)), "expires correctly ACROSS the wrap");
  }

  // --- Rule 3: a target that wedged us is not retried -------------------------------------------
  {
    coord_state_t s; coord_init(&s, 30000);
    CHECK(coord_blacklist_add(&s, A), "blacklist A");
    CHECK(coord_may_hop(&s, A) == COORD_HOP_REFUSE_BLACKLISTED, "A refused");
    CHECK(coord_may_hop(&s, B) == COORD_HOP_PERMIT, "B unaffected");
    CHECK(coord_blacklist_add(&s, A), "re-adding A is idempotent");
    CHECK(s.blacklist_len == 1, "no duplicate entry");
  }

  // --- A full blacklist reports failure rather than silently resuming retries -------------------
  {
    coord_state_t s; coord_init(&s, 30000);
    for (uint8_t i = 0; i < COORD_MAX_BLACKLIST; i++) {
      CHECK(coord_blacklist_add(&s, 0x100ULL + i), "fills");
    }
    CHECK(!coord_blacklist_add(&s, 0xDEADULL), "full blacklist reports failure");
    CHECK(coord_may_hop(&s, 0xDEADULL) == COORD_HOP_PERMIT,
          "and the un-blacklistable target is still permitted — caller must handle the false");
  }

  // --- One hop at a time -----------------------------------------------------------------------
  {
    coord_state_t s; coord_init(&s, 30000);
    CHECK(coord_hop_begin(&s, A, 0), "first hop starts");
    CHECK(coord_may_hop(&s, B) == COORD_HOP_REFUSE_IN_FLIGHT, "second refused while in flight");
    coord_hop_end(&s);
    CHECK(coord_may_hop(&s, B) == COORD_HOP_PERMIT, "permitted once the first ends");
  }

  // --- Fail closed on a null state --------------------------------------------------------------
  CHECK(coord_may_hop(0, A) == COORD_HOP_REFUSE_OPERATING, "null state refuses");

  printf(g_fail ? "TESTS FAILED\n" : "ALL TESTS PASSED\n");
  return g_fail;
}
