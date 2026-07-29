// Host unit test for the router leader-election state machine (router_election.h) — pure logic,
// no Arduino. Build from inside WLED_dev so the sibling WLED protocol header resolves:
//   c++ -std=c++11 -Wall -Wextra -I../../WLED/usermods/ampworks -o /tmp/el election_test.cpp && /tmp/el
// Exits 0 on success, 1 on the first failed assertion.
//
// Covers: fresh follower, boot hold-down, promotion to leader (term bump), step-down on a better
// beacon, best-heard (not latest) tracking, not-better beacons ignored, stale-term rejection,
// same-leader liveness refresh, and failover (tracked leader goes silent -> reassert).
#include "../../src/router_election.h"
#include <cstdio>

static int g_fail = 0;
#define CHECK(cond, msg) do { \
  if (!(cond)) { printf("FAIL: %s (line %d)\n", msg, __LINE__); g_fail = 1; } \
} while (0)

static const uint32_t TMO = 3500;   // leader timeout / boot hold-down
static const uint32_t SELF = 100;   // our deviceId

// ---- 1. Fresh state + boot hold-down + promotion ----
static void test_boot_and_promote() {
  RouterElection e = re_init();
  CHECK(!e.isLeader, "fresh state is a follower");

  // Within the hold-down (no beacon), we do NOT promote.
  CHECK(!re_tick(e, SELF, /*uptime*/1, /*now*/1000, TMO), "no promotion during boot hold-down");
  CHECK(!e.isLeader, "still follower mid hold-down");

  // After the hold-down with no better beacon -> take leadership, term bumps to 1.
  CHECK(re_tick(e, SELF, /*uptime*/4, /*now*/4000, TMO), "promotes after hold-down");
  CHECK(e.isLeader && e.term == 1 && e.leaderId == SELF, "leader with fresh term 1, self is leader");

  // Idempotent: already leader -> tick doesn't re-promote.
  CHECK(!re_tick(e, SELF, 5, 5000, TMO), "already leader: no re-promote");
  CHECK(e.term == 1, "term unchanged while already leader");
}

// ---- 2. N=1 degeneracy: a lone router just becomes leader ----
static void test_n1() {
  RouterElection e = re_init();
  re_tick(e, SELF, 4, 4000, TMO);
  CHECK(e.isLeader, "lone router (no beacons) becomes leader");
}

// ---- 3. Step down on a strictly-better beacon; ignore not-better ----
static void test_stepdown_and_ignore() {
  RouterElection e = re_init();
  re_tick(e, SELF, 4, 4000, TMO);            // we are leader (uptime 4)

  // A beacon from a longer-lived router (uptime 10) outranks us -> step down.
  re_on_beacon(e, /*sender*/200, /*uptime*/10, /*term*/1, SELF, /*selfUptime*/5, /*now*/5000);
  CHECK(!e.isLeader && e.leaderId == 200, "step down to a better (higher-uptime) router");

  // A beacon that does NOT outrank us (lower uptime) is ignored.
  re_on_beacon(e, /*sender*/300, /*uptime*/2, /*term*/1, SELF, /*selfUptime*/6, /*now*/5500);
  CHECK(e.leaderId == 200, "lower-uptime beacon ignored (leader unchanged)");
}

// ---- 4. Best-heard, not latest ----
static void test_best_not_latest() {
  RouterElection e = re_init();
  re_tick(e, SELF, 4, 4000, TMO);
  re_on_beacon(e, 200, /*uptime*/10, 1, SELF, 5, 5000);   // adopt 200
  re_on_beacon(e, 300, /*uptime*/20, 1, SELF, 5, 5100);   // 300 is even better -> adopt 300
  CHECK(e.leaderId == 300, "adopt the strictly-better router (300)");
  re_on_beacon(e, 200, /*uptime*/11, 1, SELF, 5, 5200);   // 200 again, still worse than 300
  CHECK(e.leaderId == 300, "keep the BEST leader, not the most-recent (still 300)");
}

// ---- 5. Same-leader liveness refresh + stale-term rejection ----
static void test_term_and_liveness() {
  RouterElection e = re_init();
  re_tick(e, SELF, 4, 4000, TMO);
  re_on_beacon(e, 200, 10, /*term*/5, SELF, 5, 5000);     // adopt 200 @ term 5
  CHECK(e.leaderTerm == 5 && e.lastBetterMs == 5000, "tracked leader term 5, heard @5000");

  // A stale (older-term) beacon from our leader must NOT refresh liveness.
  re_on_beacon(e, 200, 10, /*term*/3, SELF, 6, 6000);
  CHECK(e.lastBetterMs == 5000, "stale-term beacon from leader does not refresh liveness");

  // A newer-term beacon from the same leader DOES refresh.
  re_on_beacon(e, 200, 12, /*term*/6, SELF, 7, 7000);
  CHECK(e.leaderTerm == 6 && e.lastBetterMs == 7000, "newer-term beacon refreshes liveness");
}

// ---- 6. Failover: the tracked leader goes silent -> we reassert with a bumped term ----
static void test_failover() {
  RouterElection e = re_init();
  re_tick(e, SELF, 4, 4000, TMO);                 // leader, term 1
  re_on_beacon(e, 200, 10, 1, SELF, 5, 5000);     // step down to 200
  CHECK(!e.isLeader, "follower under leader 200");

  // Not yet timed out -> stay follower.
  CHECK(!re_tick(e, SELF, 6, 5000 + TMO, TMO), "within timeout: stay follower");

  // 200 goes silent past the timeout -> reassert leadership, term bumps to 2.
  CHECK(re_tick(e, SELF, 7, 5000 + TMO + 1, TMO), "reassert after leader goes silent");
  CHECK(e.isLeader && e.leaderId == SELF && e.term == 2, "self leader again with term 2");
}

int main() {
  test_boot_and_promote();
  test_n1();
  test_stepdown_and_ignore();
  test_best_not_latest();
  test_term_and_liveness();
  test_failover();
  if (g_fail) { printf("SOME TESTS FAILED\n"); return 1; }
  printf("ALL TESTS PASSED\n");
  return 0;
}
