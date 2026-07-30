// Host unit test for the attach protocol (router_attach.h) — pure logic, no Arduino. Run it with
// `make -C tests test` (or `pio test -e native`) from inside WLED_dev, so the sibling WLED wire
// header resolves. Exits 0 on success, 1 on the first failed assertion.
//
// Covers: control-frame header stamping; control frames never relayed; membership add / refresh /
// expire / full table; the router-selection metric; adoption, hold-down and preemption by a
// shorter path; unreachable routers never selected and dropped when they were ours; the ack
// handshake; heartbeat timeout dropping the route and the immediate re-bind that follows; the two
// loop guards on selection; and the advertised hop cost.
#include "../../src/router_attach.h"
#include "../../src/router_relay.h"
#include <cstdio>

static int g_fail = 0;
#define CHECK(cond, msg) do { \
  if (!(cond)) { printf("FAIL: %s (line %d)\n", msg, __LINE__); g_fail = 1; } \
} while (0)

static const uint32_t SELF = 0x1000;   // our deviceId
static const uint32_t HOLD = 5000;     // route hold-down
static const uint32_t TMO  = 3500;     // heartbeat timeout

// Build a router advertisement with the given metric.
static RouterAdvert mkadv(uint8_t hopCost, uint8_t members, uint32_t leaderId = 0x900,
                          uint16_t term = 1) {
  RouterAdvert a{};
  a.leaderId = leaderId;
  a.leaderTerm = term;
  a.hopCost = hopCost;
  a.memberCount = members;
  return a;
}

// ---- 1. Control-frame header: same wire header as a snapshot, but single-hop ----
static void test_control_header() {
  SensorSyncHeader h{};
  ra_stamp_control_header(h, SELF, SENSOR_SYNC_MSG_ROUTER_ADV, sizeof(RouterAdvert), 7, 12345);
  CHECK(h.magic[0] == 'A' && h.magic[3] == 'S', "control frame carries the AMPS magic");
  CHECK(h.version == SENSOR_SYNC_VERSION, "control frame uses the shared wire version");
  CHECK(h.msgType == SENSOR_SYNC_MSG_ROUTER_ADV, "msgType stamped as given");
  CHECK(h.dataLen == sizeof(RouterAdvert), "dataLen matches the payload");
  CHECK(h.deviceId == SELF && h.seq == 7 && h.timestamp == 12345, "origin, seq and timestamp set");
  CHECK(h.ttl == 1, "control frames are single-hop (ttl 1)");

  // The four msgTypes are distinct, so a receiver can demux them.
  CHECK(SENSOR_SYNC_MSG_SNAPSHOT != SENSOR_SYNC_MSG_BEACON &&
        SENSOR_SYNC_MSG_ROUTER_ADV != SENSOR_SYNC_MSG_ATTACH &&
        SENSOR_SYNC_MSG_ATTACH != SENSOR_SYNC_MSG_ATTACH_ACK, "msgType numbers are distinct");
}

// ---- 2. Only snapshots are relayed; every control frame stops at the node that hears it ----
static void test_control_not_relayed() {
  CHECK( ss_router_is_relayable(SENSOR_SYNC_MSG_SNAPSHOT),   "snapshots travel multi-hop");
  CHECK(!ss_router_is_relayable(SENSOR_SYNC_MSG_BEACON),     "beacons are single-hop");
  CHECK(!ss_router_is_relayable(SENSOR_SYNC_MSG_ROUTER_ADV), "advertisements are single-hop");
  CHECK(!ss_router_is_relayable(SENSOR_SYNC_MSG_ATTACH),     "attaches are single-hop");
  CHECK(!ss_router_is_relayable(SENSOR_SYNC_MSG_ATTACH_ACK), "acks are single-hop");

  // ss_router_should_relay drops a control frame even when it is otherwise fresh and has TTL left.
  const uint8_t N = 4;
  SensorRouterPeer tbl[N]{};
  SensorSyncHeader h{};
  ra_stamp_control_header(h, 0x2000, SENSOR_SYNC_MSG_ATTACH, sizeof(NodeAttach), 1, 100);
  h.ttl = SS_DEFAULT_TTL;
  uint8_t out = 0;
  CHECK(!ss_router_should_relay(h, SELF, tbl, N, &out), "a fresh attach frame is not relayed");
}

// ---- 3. Membership table: add, refresh, count, expire, full ----
static void test_membership() {
  const uint8_t N = 3;
  AttachedNode tbl[N]{};

  CHECK(ra_member_count(tbl, N) == 0, "a fresh table has no members");
  CHECK(ra_member_find(tbl, N, 0xA) == 0, "an unattached node is not found");

  CHECK(ra_member_attach(tbl, N, 0xA, 1000) != 0, "first attach admits the node");
  CHECK(ra_member_attach(tbl, N, 0xB, 1000) != 0, "second node admitted");
  CHECK(ra_member_count(tbl, N) == 2, "both nodes counted");

  // A repeat attach refreshes the existing slot rather than consuming another.
  CHECK(ra_member_attach(tbl, N, 0xA, 4000) != 0, "keepalive attach accepted");
  CHECK(ra_member_count(tbl, N) == 2, "keepalive does not add a second slot");
  CHECK(ra_member_find(tbl, N, 0xA)->lastSeenMs == 4000, "keepalive refreshed the last-seen stamp");

  // Lease expiry drops only the node that went quiet: 0xB was last heard at 1000, 0xA at 4000.
  CHECK(ra_member_expire(tbl, N, 6500, /*leaseMs*/3000) == 1, "one member expired");
  CHECK(ra_member_find(tbl, N, 0xB) == 0, "the quiet node was released");
  CHECK(ra_member_find(tbl, N, 0xA) != 0, "the node still sending keepalives was kept");
  CHECK(ra_member_count(tbl, N) == 1, "count reflects the release");

  // Exactly at the lease boundary the member is still held (expiry is strictly greater than).
  CHECK(ra_member_expire(tbl, N, 7000, /*leaseMs*/3000) == 0, "member at exactly the lease is kept");

  // A released slot is reusable, and a full table refuses the attach so the node retries later.
  CHECK(ra_member_attach(tbl, N, 0xC, 7000) != 0, "released slot is reused");
  CHECK(ra_member_attach(tbl, N, 0xD, 7000) != 0, "third slot taken");
  CHECK(ra_member_attach(tbl, N, 0xE, 7000) == 0, "full table refuses a new member");
  CHECK(ra_member_attach(tbl, N, 0xA, 7500) != 0, "an existing member still refreshes when full");
}

// ---- 4. The selection metric: hops, then load, then deviceId ----
static void test_selection_metric() {
  CHECK( ra_advert_better(1, 9, 0x10, 2, 0, 0xFF), "fewer hops wins over load and id");
  CHECK(!ra_advert_better(3, 0, 0xFF, 2, 9, 0x10), "more hops loses regardless of load and id");

  CHECK( ra_advert_better(2, 1, 0x10, 2, 5, 0xFF), "equal hops: lighter load wins");
  CHECK(!ra_advert_better(2, 5, 0xFF, 2, 1, 0x10), "equal hops: heavier load loses");

  CHECK( ra_advert_better(2, 3, 0xFF, 2, 3, 0x10), "equal hops and load: higher id wins");
  CHECK(!ra_advert_better(2, 3, 0x10, 2, 3, 0xFF), "equal hops and load: lower id loses");

  CHECK(!ra_advert_better(2, 3, 0x10, 2, 3, 0x10), "an identical metric is not better (no flapping)");
}

// ---- 5. Adoption, hold-down, and preemption by a shorter path ----
static void test_adopt_and_holddown() {
  RouteChoice r = ra_route_init();
  CHECK(!ra_route_held(r), "a fresh node holds no route");

  // With nothing chosen, the first usable advertisement is adopted outright.
  CHECK(ra_on_advert(r, 0x20, mkadv(2, 4), /*downstream*/false, 1000, HOLD),
        "first advertisement adopted");
  CHECK(r.routerId == 0x20 && r.hopCost == 2 && !r.attached, "bound to it, attach still pending");

  // Its own heartbeats refresh freshness and re-read its metric without re-binding.
  CHECK(!ra_on_advert(r, 0x20, mkadv(3, 6), /*downstream*/false, 2000, HOLD),
        "our router's heartbeat is not a change");
  CHECK(r.hopCost == 3 && r.memberCount == 6 && r.lastHeardMs == 2000, "metric and freshness updated");

  // Inside the hold-down, a merely-lighter router of the same distance must wait.
  CHECK(!ra_on_advert(r, 0x30, mkadv(3, 0), /*downstream*/false, 2500, HOLD),
        "load-only win is held down");
  CHECK(r.routerId == 0x20, "still bound to the incumbent");

  // A strictly shorter path is a real improvement and preempts immediately.
  CHECK(ra_on_advert(r, 0x40, mkadv(1, 9), /*downstream*/false, 2600, HOLD),
        "shorter path preempts during hold-down");
  CHECK(r.routerId == 0x40 && r.hopCost == 1, "switched to the closer router");
  CHECK(r.chosenMs == 2600, "hold-down restarts on the switch");

  // A worse router never wins, hold-down or not.
  CHECK(!ra_on_advert(r, 0x50, mkadv(4, 0), /*downstream*/false, 20000, HOLD),
        "a more distant router never wins");

  // Once the hold-down has passed, the full metric applies again.
  CHECK(ra_on_advert(r, 0x60, mkadv(1, 0), /*downstream*/false, 20000, HOLD),
        "lighter router wins after the hold-down");
  CHECK(r.routerId == 0x60, "switched on the load tiebreak");
}

// ---- 6. A router with no path to the leader is never selected ----
static void test_unreachable() {
  RouteChoice r = ra_route_init();
  CHECK(!ra_on_advert(r, 0x20, mkadv(SS_HOP_UNREACHABLE, 0), /*downstream*/false, 1000, HOLD),
        "an unreachable router is not adopted");
  CHECK(!ra_route_held(r), "still unattached");

  // Our own router losing its path drops the route rather than leaving us on a dead end.
  CHECK(ra_on_advert(r, 0x20, mkadv(1, 0), /*downstream*/false, 1000, HOLD),
        "adopt a reachable router");
  CHECK(ra_on_advert(r, 0x20, mkadv(SS_HOP_UNREACHABLE, 0), /*downstream*/false, 2000, HOLD),
        "our router going unreachable is a change");
  CHECK(!ra_route_held(r), "route dropped when the upstream lost its path");
}

// ---- 7. The ack handshake confirms membership, and only for us ----
static void test_ack() {
  RouteChoice r = ra_route_init();
  ra_on_advert(r, 0x20, mkadv(1, 0), /*downstream*/false, 1000, HOLD);

  AttachAck other{}; other.nodeId = 0x999; other.leaseMs = 6000;
  CHECK(!ra_on_ack(r, 0x20, other, SELF, 1500), "an ack for another node is ignored");
  CHECK(!r.attached, "still unattached");

  AttachAck mine{}; mine.nodeId = SELF; mine.leaseMs = 6000;
  CHECK(!ra_on_ack(r, 0x77, mine, SELF, 1500), "an ack from a router we are not bound to is ignored");

  CHECK(ra_on_ack(r, 0x20, mine, SELF, 1600), "our router's ack confirms the attach");
  CHECK(r.attached && r.lastHeardMs == 1600, "attached, and the ack counts as a heartbeat");
  CHECK(!ra_on_ack(r, 0x20, mine, SELF, 1700), "a repeat ack is not a new confirmation");
  CHECK(r.lastHeardMs == 1700, "but it still refreshes freshness");
}

// ---- 8. Self-healing: the upstream router goes silent, the route drops, a survivor takes over ----
static void test_failover() {
  RouteChoice r = ra_route_init();
  ra_on_advert(r, 0x20, mkadv(1, 0), /*downstream*/false, 1000, HOLD);
  AttachAck ack{}; ack.nodeId = SELF; ack.leaseMs = 6000;
  ra_on_ack(r, 0x20, ack, SELF, 1100);
  CHECK(r.attached, "attached to 0x20");

  // Within the heartbeat timeout nothing happens — a single missed advertisement is not a failure.
  CHECK(!ra_route_tick(r, 1100 + TMO, TMO), "within the timeout the route is kept");
  CHECK(r.routerId == 0x20, "still bound");

  // Past it, the route is dropped and the node is unattached again.
  CHECK(ra_route_tick(r, 1100 + TMO + 1, TMO), "silent router times out");
  CHECK(!ra_route_held(r) && !r.attached, "route dropped, membership no longer claimed");
  CHECK(!ra_route_tick(r, 100000, TMO), "an already-dropped route does not re-fire");

  // Recovery takes one advertisement: with no incumbent there is no hold-down to wait out, so a
  // router that would have been held down before is adopted at once.
  CHECK(ra_on_advert(r, 0x30, mkadv(2, 7), /*downstream*/false, 6000, HOLD),
        "the next advertisement heard is adopted");
  CHECK(r.routerId == 0x30 && !r.attached, "re-bound to the survivor, attach pending again");
}

// ---- 9. The hop cost we advertise ----
static void test_advert_cost() {
  RouteChoice r = ra_route_init();
  CHECK(ra_advert_cost(/*isLeader*/true, r) == 0, "the leader is zero hops from itself");
  CHECK(ra_advert_cost(false, r) == SS_HOP_UNREACHABLE, "a follower with no route is unreachable");

  ra_on_advert(r, 0x20, mkadv(0, 0), /*downstream*/false, 1000, HOLD);
  CHECK(ra_advert_cost(false, r) == 1, "one hop further than our upstream");
  CHECK(ra_advert_cost(true, r) == 0, "leadership overrides any route we still hold");

  // Saturating: a chain long enough to reach the sentinel reports unreachable, never wraps to 0.
  r.hopCost = SS_HOP_UNREACHABLE - 2;
  CHECK(ra_advert_cost(false, r) == SS_HOP_UNREACHABLE - 1, "cost climbs toward the sentinel");
  r.hopCost = SS_HOP_UNREACHABLE - 1;
  CHECK(ra_advert_cost(false, r) == SS_HOP_UNREACHABLE, "cost saturates instead of wrapping");
}

// ---- 10. Loop guards: never route up through our own subtree ----
static void test_loop_guards() {
  const uint8_t N = 4;
  AttachedNode tbl[N]{};
  ra_member_attach(tbl, N, 0x20, 1000);          // 0x20 attached to us, so it is below us
  CHECK(ra_member_attached(tbl, N, 0x20), "an attached node reads as downstream");
  CHECK(!ra_member_attached(tbl, N, 0x21), "an unattached node does not");

  // Even a genuinely attractive advertisement from a node below us is refused.
  RouteChoice r = ra_route_init();
  CHECK(!ra_on_advert(r, 0x20, mkadv(0, 0), /*downstream*/true, 1000, HOLD),
        "a node below us is never adopted as our upstream");
  CHECK(!ra_route_held(r), "still unattached");

  // Two nodes can still adopt each other in the window before either attach lands. Once our
  // upstream turns up in our own member table, the pair is circling — drop and re-root.
  CHECK(ra_on_advert(r, 0x20, mkadv(1, 0), /*downstream*/false, 2000, HOLD),
        "adopted before its attach arrived");
  CHECK(ra_route_break_loop(r, tbl, N), "mutual attachment detected");
  CHECK(!ra_route_held(r), "route dropped, ready to re-root");
  CHECK(!ra_route_break_loop(r, tbl, N), "nothing to break with no route held");

  // A route to a node that is not our member is left alone.
  ra_on_advert(r, 0x30, mkadv(1, 0), /*downstream*/false, 3000, HOLD);
  CHECK(!ra_route_break_loop(r, tbl, N), "a non-member upstream is not a loop");
  CHECK(r.routerId == 0x30, "route kept");
}

// ---- 11. Leadership term gates the metric ----
// A cost computed under a superseded leader describes a tree that no longer exists, so it must not
// displace an incumbent from the current term however cheap it looks; a newer term is a re-formed
// mesh and preempts at once, without waiting out the hold-down.
static void test_term_guard() {
  RouteChoice r = ra_route_init();
  AttachedNode tbl[4]{};
  (void)tbl;

  // Bind to a 3-hop router advertising term 5.
  CHECK(ra_on_advert(r, 0x10, mkadv(3, 0, 0x900, 5), false, 1000, HOLD), "adopts first router");
  CHECK(r.routerId == 0x10 && r.leaderTerm == 5, "bound at term 5");

  // A far cheaper path, but stamped with the OLD term -> refused despite winning on every metric.
  CHECK(!ra_on_advert(r, 0x20, mkadv(1, 0, 0x900, 4), false, 2000, HOLD),
        "stale-term advert refused even though its hop cost is lower");
  CHECK(r.routerId == 0x10, "incumbent kept against a stale-term challenger");

  // Same cost, but a NEWER term -> adopted immediately, inside the hold-down window.
  CHECK(ra_on_advert(r, 0x30, mkadv(3, 0, 0x900, 6), false, 2500, HOLD),
        "newer-term advert preempts inside the hold-down");
  CHECK(r.routerId == 0x30 && r.leaderTerm == 6, "re-bound to the current term");

  // Term equality falls back to the ordinary metric.
  CHECK(ra_on_advert(r, 0x40, mkadv(1, 0, 0x900, 6), false, 9000, HOLD),
        "same term: shorter path still wins");
  CHECK(r.routerId == 0x40, "metric applies within a term");
}

// ---- 12. Term comparison survives the 16-bit wrap ----
static void test_term_wraparound() {
  RouteChoice r = ra_route_init();

  CHECK(ra_on_advert(r, 0x10, mkadv(3, 0, 0x900, 0xFFFF), false, 1000, HOLD), "adopt at term 0xFFFF");
  // 0 is newer than 0xFFFF under RFC-1982 comparison, so this is a re-formed mesh, not a stale one.
  CHECK(ra_on_advert(r, 0x20, mkadv(3, 0, 0x900, 0), false, 2000, HOLD),
        "term 0 is newer than 0xFFFF (wrap), so it preempts");
  CHECK(r.routerId == 0x20, "wrapped term adopted");
  // ...and the pre-wrap term is now the stale one.
  CHECK(!ra_on_advert(r, 0x30, mkadv(1, 0, 0x900, 0xFFFE), false, 3000, HOLD),
        "pre-wrap term is stale even with a better cost");
}

// ---- 13. Time bases hold across the uint32 millisecond wrap ----
// Every interval here is unsigned difference arithmetic, so a clock that wraps mid-lease must not
// expire members early or freeze the hold-down.
static void test_time_wraparound() {
  const uint32_t NEAR_WRAP = 0xFFFFF000u;
  AttachedNode tbl[4]{};

  CHECK(ra_member_attach(tbl, 4, 0xAA, NEAR_WRAP) != 0, "attach just before the wrap");

  // 0x800 ms later the clock has wrapped past zero; the member is still inside a 6 s lease.
  const uint32_t after = NEAR_WRAP + 0x800u;   // wraps
  CHECK(ra_member_expire(tbl, 4, after, 6000) == 0, "no early expiry across the wrap");
  CHECK(ra_member_attached(tbl, 4, 0xAA), "member still held across the wrap");

  // Well past the lease, still spanning the wrap -> released.
  CHECK(ra_member_expire(tbl, 4, NEAR_WRAP + 20000u, 6000) == 1, "expires normally across the wrap");

  // Hold-down measured across the wrap.
  RouteChoice r = ra_route_init();
  CHECK(ra_on_advert(r, 0x10, mkadv(3, 0), false, NEAR_WRAP, HOLD), "adopt near the wrap");
  CHECK(!ra_on_advert(r, 0x20, mkadv(3, 0, 0x900, 1), false, NEAR_WRAP + 100u, HOLD),
        "equal-cost challenger still held off just after the wrap");
  CHECK(ra_route_tick(r, NEAR_WRAP + 20000u, TMO), "route timeout fires across the wrap");
  CHECK(!ra_route_held(r), "timed-out route is dropped");
}

// ---- 14. A 3-node cycle is bounded by cost saturation, not detected outright ----
// The membership loop guard only sees one hop, so A->B->C->A is not caught directly. Each lap adds
// a hop until the advertised cost saturates and rule 0 drops the route. This pins that documented
// bound so it cannot silently regress into an unbounded loop.
static void test_three_node_cycle_bound() {
  RouteChoice r = ra_route_init();
  AttachedNode tbl[4]{};   // the cycle peer is NOT our member, so the 2-cycle guard cannot fire

  CHECK(ra_on_advert(r, 0x10, mkadv(3, 0), false, 1000, HOLD), "adopt upstream in the cycle");
  CHECK(!ra_route_break_loop(r, tbl, 4), "2-cycle guard does not fire on a 3-node cycle");

  // Actually run the cycle: three nodes, each advertising the cost implied by the router it is
  // bound to, with A bound to B, B to C and C back to A. Nobody is anybody's member, so the 2-cycle
  // guard stays silent and only cost inflation can end it.
  RouteChoice node[3];
  const uint32_t id[3] = {0xA1, 0xB2, 0xC3};
  for (int k = 0; k < 3; k++) {
    node[k] = ra_route_init();
    // k points at its successor; seed every hop cost at 1 so the cycle starts plausible.
    CHECK(ra_on_advert(node[k], id[(k + 1) % 3], mkadv(1, 0), false, 1000, HOLD), "cycle seeded");
  }

  uint32_t t = 2000;
  int laps = 0;
  bool collapsed = false;
  while (laps < 512 && !collapsed) {
    for (int k = 0; k < 3; k++) {
      // Each node re-advertises what it would now claim, and its predecessor re-reads it.
      const uint8_t advertised = ra_advert_cost(false, node[k]);
      const int pred = (k + 2) % 3;
      ra_on_advert(node[pred], id[k], mkadv(advertised, 0), false, t, HOLD);
      if (!ra_route_held(node[pred])) collapsed = true;
    }
    t += 1000;
    laps++;
  }

  CHECK(collapsed, "the 3-node cycle collapses instead of circulating forever");
  CHECK(laps < 512, "collapse happens within a bounded number of advert rounds");
  // Once the upstream advertises unreachable, the route is dropped.
  CHECK(ra_on_advert(r, 0x10, mkadv(SS_HOP_UNREACHABLE, 0), false, 5000, HOLD),
        "unreachable upstream drops the route, ending the cycle");
  CHECK(!ra_route_held(r), "node is unattached and free to re-root");
}

int main() {
  test_control_header();
  test_control_not_relayed();
  test_membership();
  test_selection_metric();
  test_adopt_and_holddown();
  test_unreachable();
  test_ack();
  test_failover();
  test_advert_cost();
  test_loop_guards();
  test_term_guard();
  test_term_wraparound();
  test_time_wraparound();
  test_three_node_cycle_bound();
  if (g_fail) { printf("SOME TESTS FAILED\n"); return 1; }
  printf("ALL TESTS PASSED\n");
  return 0;
}
