// Host unit test for coordinator_phase.h — own-AP onboarding phase sequencing.
//
//   c++ -std=c++11 -Wall -o /tmp/enr_coord_phase test_coord_phase.cpp && /tmp/enr_coord_phase
//
#include "../../src/coordinator_phase.h"
#include <cstdio>

static int g_fail = 0;
#define CHECK(cond, msg) do { \
  if (!(cond)) { printf("FAIL: %s\n", msg); g_fail = 1; } \
} while (0)

int main() {
  // --- The AP must be DOWN while hopping, and only while hopping --------------------------------
  // Hopping with the AP up is the one combination that drops already-onboarded clients.
  {
    coord_phase_state_t s; coord_phase_init(&s, COORD_SETTLE_MS);
    coord_phase_enter(&s, COORD_PHASE_HOP, 0);
    CHECK(!coord_ap_should_be_up(&s), "AP down during HOP");
    coord_phase_enter(&s, COORD_PHASE_SETTLE, 0);
    CHECK(coord_ap_should_be_up(&s), "AP up during SETTLE");
    coord_phase_enter(&s, COORD_PHASE_SYNC, 0);
    CHECK(coord_ap_should_be_up(&s), "AP up during SYNC");
    coord_phase_enter(&s, COORD_PHASE_IDLE, 0);
    CHECK(coord_ap_should_be_up(&s), "AP up when idle");
  }

  // --- Settle must span at least one full WLED retry interval -----------------------------------
  // Under-running it is SILENT: discovery just finds fewer devices.
  {
    coord_phase_state_t s; coord_phase_init(&s, COORD_SETTLE_MS);
    coord_phase_enter(&s, COORD_PHASE_SETTLE, 1000);
    CHECK(!coord_settle_complete(&s, 1000), "not settled at t=0");
    CHECK(!coord_settle_complete(&s, 1000 + WLED_STA_RETRY_MS - 1),
          "not settled before one retry interval has passed");
    CHECK(coord_settle_complete(&s, 1000 + COORD_SETTLE_MS), "settled at the configured window");
    CHECK(COORD_SETTLE_MS >= WLED_STA_RETRY_MS,
          "the default settle window is at least one retry interval");
  }

  // --- A too-short settle is clamped, not honoured ----------------------------------------------
  // The caller cannot accidentally configure a window in which no orphan can possibly knock.
  {
    coord_phase_state_t s; coord_phase_init(&s, 500);
    CHECK(s.settle_ms == WLED_STA_RETRY_MS, "sub-retry settle is clamped up to the retry interval");
    coord_phase_enter(&s, COORD_PHASE_SETTLE, 0);
    CHECK(!coord_settle_complete(&s, 500), "and the clamp is actually in force");
    CHECK(coord_settle_complete(&s, WLED_STA_RETRY_MS), "settled once the clamped window elapses");
  }

  // --- Discovery is gated on the settle window --------------------------------------------------
  {
    coord_phase_state_t s; coord_phase_init(&s, COORD_SETTLE_MS);
    coord_phase_enter(&s, COORD_PHASE_HOP, 0);
    CHECK(!coord_may_discover(&s, 999999), "never discover during HOP, however long it has run");
    coord_phase_enter(&s, COORD_PHASE_SETTLE, 0);
    CHECK(!coord_may_discover(&s, COORD_SETTLE_MS - 1), "not during an incomplete settle");
    CHECK(coord_may_discover(&s, COORD_SETTLE_MS), "yes once settle completes");
    coord_phase_enter(&s, COORD_PHASE_SYNC, 0);
    CHECK(coord_may_discover(&s, 0), "yes in SYNC regardless of elapsed");
  }

  // --- settle_complete is phase-scoped, not just a timer ----------------------------------------
  // A stale timestamp from a previous SETTLE must not make HOP look settled.
  {
    coord_phase_state_t s; coord_phase_init(&s, COORD_SETTLE_MS);
    coord_phase_enter(&s, COORD_PHASE_SETTLE, 0);
    CHECK(coord_settle_complete(&s, COORD_SETTLE_MS), "settled");
    coord_phase_enter(&s, COORD_PHASE_HOP, COORD_SETTLE_MS);
    CHECK(!coord_settle_complete(&s, COORD_SETTLE_MS * 4), "HOP is never 'settled'");
  }

  // --- Wrap-safety across the ~49-day millis() rollover -----------------------------------------
  {
    coord_phase_state_t s; coord_phase_init(&s, COORD_SETTLE_MS);
    const uint32_t near_wrap = 0xFFFFF000u;
    coord_phase_enter(&s, COORD_PHASE_SETTLE, near_wrap);
    CHECK(!coord_settle_complete(&s, near_wrap + 100), "pre-wrap: not settled");
    CHECK(coord_settle_complete(&s, (uint32_t)(near_wrap + COORD_SETTLE_MS)),
          "settles correctly ACROSS the wrap");
  }

  // --- The constants match WLED, and are not drifting quietly -----------------------------------
  CHECK(WLED_STA_RETRY_MS == 18000u,   "STA retry matches wled.cpp:947");
  CHECK(WLED_STA_RETRY_STAC == 300000u,"associated-station backoff matches wled.cpp:947");
  CHECK(WLED_AP_FALLBACK_MS == 12000u, "AP fallback matches wled.cpp:953");

  // --- Null state fails closed ------------------------------------------------------------------
  CHECK(!coord_may_discover(0, 0), "null: no discovery");
  CHECK(!coord_settle_complete(0, 0), "null: not settled");
  CHECK(!coord_ap_should_be_up(0), "null: AP not asserted up");

  printf(g_fail ? "TESTS FAILED\n" : "ALL TESTS PASSED\n");
  return g_fail;
}
