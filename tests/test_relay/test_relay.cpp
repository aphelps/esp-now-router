// Host unit test for the multi-hop relay logic (ss_router_should_relay) in router_relay.h.
//
// Builds with a normal host compiler (no Arduino/WLED) — includes only the pure header:
//   c++ -std=c++11 -Wall -o /tmp/ss_relay_test sensor_sync_relay_test.cpp && /tmp/ss_relay_test
// Exits 0 on success, 1 on the first failed assertion.
//
// Covers: wire-compat of the reserved->ttl/flags split (still 20-byte v5 header); dedup
// (already-seen seq not relayed, newer relayed); 16-bit seq wraparound; TTL exhaustion; legacy
// ttl==0 default injection; self-origin skip; and a graph flood over loop + line topologies
// asserting termination (loop-free), relay-at-most-once, and TTL-bounded coverage.
//
#include "../../src/router_relay.h"
#include <cstdio>
#include <cstddef>
#include <vector>
#include <deque>

static int g_fail = 0;
#define CHECK(cond, msg) do { \
  if (!(cond)) { printf("FAIL: %s (line %d)\n", msg, __LINE__); g_fail = 1; } \
} while (0)

static SensorSyncHeader mkhdr(uint32_t dev, uint16_t seq, uint8_t ttl) {
  SensorSyncHeader h{};
  memcpy(h.magic, "AMPS", 4);
  h.version = SENSOR_SYNC_VERSION; h.msgType = SENSOR_SYNC_MSG_SNAPSHOT;
  h.sensorType = SS_SENSOR_TOUCH; h.dataLen = sizeof(SensorSnapshot);
  h.deviceId = dev; h.seq = seq; h.ttl = ttl; h.flags = 0; h.timestamp = 1000 + seq;
  return h;
}

static SensorSyncHeader mkhdr_type(uint32_t dev, uint16_t seq, uint8_t ttl, uint8_t msgType) {
  SensorSyncHeader h = mkhdr(dev, seq, ttl);
  h.msgType = msgType;
  return h;
}

// ---- 1b. Which msgTypes cross the backbone ----
// The predicate is a whitelist, and until M4 it whitelisted snapshots alone. These assertions pin
// BOTH directions: that control frames now travel, and that the router's own plane still does not.
// A regression either way is silent in normal operation — a leaked beacon corrupts a distant
// router's routing metric, and a dropped control frame just looks like flaky WiFi.
static void test_relayable_types() {
  CHECK(ss_router_is_relayable(SENSOR_SYNC_MSG_SNAPSHOT),  "snapshots relay");
  CHECK(ss_router_is_relayable(SENSOR_SYNC_MSG_CONTROL),   "control frames relay");
  // Reboot recovery must cross the backbone: a rebooted node whose only peers sit behind a
  // router would otherwise collect zero clock replies and stay muted until someone else
  // originates a command.
  CHECK(ss_router_is_relayable(SENSOR_SYNC_MSG_CTRL_QUERY), "clock queries relay");
  CHECK(ss_router_is_relayable(SENSOR_SYNC_MSG_CTRL_CLOCK), "clock replies relay");

  CHECK(!ss_router_is_relayable(SENSOR_SYNC_MSG_BEACON),     "election beacon stays single-hop");
  CHECK(!ss_router_is_relayable(SENSOR_SYNC_MSG_ROUTER_ADV), "router heartbeat stays single-hop");
  CHECK(!ss_router_is_relayable(SENSOR_SYNC_MSG_ATTACH),     "attach stays single-hop");
  CHECK(!ss_router_is_relayable(SENSOR_SYNC_MSG_ATTACH_ACK), "attach-ack stays single-hop");
  // Reserved but deliberately not relayed — see the comment on the predicate.
  CHECK(!ss_router_is_relayable(SENSOR_SYNC_MSG_TIMEBASE),   "timebase beacon stays single-hop");
  // An unallocated type must not start travelling just because it exists.
  CHECK(!ss_router_is_relayable(200), "unknown msgType stays single-hop");
}

// ---- 1c. Control frames get the same dedup + TTL treatment as snapshots ----
static void test_control_relay_semantics() {
  const uint32_t SELF = 0x000000AA, ORIGIN = 0x000000BB;
  SensorRouterPeer tbl[4] = {};
  uint8_t ttlOut = 0;

  SensorSyncHeader c1 = mkhdr_type(ORIGIN, 1, SS_DEFAULT_TTL, SENSOR_SYNC_MSG_CONTROL);
  CHECK(ss_router_should_relay(c1, SELF, tbl, 4, &ttlOut), "fresh control frame relays");
  CHECK(ttlOut == SS_DEFAULT_TTL - 1, "control TTL is decremented like a snapshot's");

  CHECK(!ss_router_should_relay(c1, SELF, tbl, 4, &ttlOut), "duplicate control frame dropped");

  SensorSyncHeader c2 = mkhdr_type(ORIGIN, 2, SS_DEFAULT_TTL, SENSOR_SYNC_MSG_CONTROL);
  CHECK(ss_router_should_relay(c2, SELF, tbl, 4, &ttlOut), "newer control frame relays");

  // Exhausted hop budget drops, same as a snapshot.
  SensorSyncHeader c3 = mkhdr_type(ORIGIN, 3, 1, SENSOR_SYNC_MSG_CONTROL);
  CHECK(!ss_router_should_relay(c3, SELF, tbl, 4, &ttlOut), "control frame with ttl<=1 dropped");

  // Our own control echo is never relayed.
  SensorSyncHeader own = mkhdr_type(SELF, 9, SS_DEFAULT_TTL, SENSOR_SYNC_MSG_CONTROL);
  CHECK(!ss_router_should_relay(own, SELF, tbl, 4, &ttlOut), "own control echo not relayed");

  // Snapshots and control frames from one origin share a single seq space (the edge stamps both
  // from one txSeq counter), so they must not shadow each other in the dedup table.
  //
  // The property is that a seq the origin has already spent is refused REGARDLESS of the type
  // carrying it, which is only true if the two types share one space. Under per-msgType dedup the
  // control frame below would relay, and a snapshot and a control frame could then shadow each
  // other's dedup state.
  SensorRouterPeer tbl2b[4] = {};
  SensorSyncHeader s20  = mkhdr_type(ORIGIN, 20, SS_DEFAULT_TTL, SENSOR_SYNC_MSG_SNAPSHOT);
  SensorSyncHeader c20  = mkhdr_type(ORIGIN, 20, SS_DEFAULT_TTL, SENSOR_SYNC_MSG_CONTROL);
  SensorSyncHeader c19  = mkhdr_type(ORIGIN, 19, SS_DEFAULT_TTL, SENSOR_SYNC_MSG_CONTROL);
  CHECK(ss_router_should_relay(s20, SELF, tbl2b, 4, &ttlOut), "snapshot 20 relays");
  CHECK(!ss_router_should_relay(c20, SELF, tbl2b, 4, &ttlOut),
        "control reusing seq 20 is a duplicate — the seq space is shared, not per-msgType");
  CHECK(!ss_router_should_relay(c19, SELF, tbl2b, 4, &ttlOut),
        "and a control frame BEHIND the snapshot's seq is stale in that same shared space");

  // Symmetric: the ordering must not depend on which type happened to arrive first.
  SensorRouterPeer tbl2c[4] = {};
  SensorSyncHeader c30 = mkhdr_type(ORIGIN, 30, SS_DEFAULT_TTL, SENSOR_SYNC_MSG_CONTROL);
  SensorSyncHeader s30 = mkhdr_type(ORIGIN, 30, SS_DEFAULT_TTL, SENSOR_SYNC_MSG_SNAPSHOT);
  CHECK(ss_router_should_relay(c30, SELF, tbl2c, 4, &ttlOut), "control 30 relays");
  CHECK(!ss_router_should_relay(s30, SELF, tbl2c, 4, &ttlOut),
        "snapshot reusing seq 30 is likewise a duplicate");

  // A single-hop type must be dropped by should_relay even when everything else says relay, and
  // must NOT consume dedup state that a later real frame depends on.
  SensorRouterPeer tbl3[4] = {};
  SensorSyncHeader beacon = mkhdr_type(ORIGIN, 5, SS_DEFAULT_TTL, SENSOR_SYNC_MSG_BEACON);
  CHECK(!ss_router_should_relay(beacon, SELF, tbl3, 4, &ttlOut), "beacon dropped by should_relay");
  SensorSyncHeader ctrl5 = mkhdr_type(ORIGIN, 5, SS_DEFAULT_TTL, SENSOR_SYNC_MSG_CONTROL);
  CHECK(ss_router_should_relay(ctrl5, SELF, tbl3, 4, &ttlOut),
        "dropped beacon did not poison dedup state for a later frame");
}

// ---- 1. Wire compatibility: reserved->ttl/flags is same-size, same-offset ----
static void test_wire_compat() {
  CHECK(sizeof(SensorSyncHeader) == 20, "header is still 20 bytes");
  CHECK(offsetof(SensorSyncHeader, ttl) == 14, "ttl sits where reserved[0] was (offset 14)");
  CHECK(offsetof(SensorSyncHeader, flags) == 15, "flags sits where reserved[1] was (offset 15)");
  CHECK(offsetof(SensorSyncHeader, timestamp) == 16, "timestamp offset unchanged (16)");

  // A legacy v5 frame zeroed `reserved`, i.e. ttl=0/flags=0 on the wire.
  SensorSyncHeader legacy{}; memcpy(legacy.magic, "AMPS", 4);
  legacy.version = SENSOR_SYNC_VERSION;
  CHECK(legacy.ttl == 0 && legacy.flags == 0, "legacy frame parses as ttl=0/flags=0");
}

// ---- 2. Dedup + TTL unit cases ----
static void test_unit() {
  const uint8_t N = 8;
  SensorRouterPeer tbl[N]{};
  const uint32_t self = 999, origin = 1;
  uint8_t out = 0;

  // First frame from an origin is fresh -> relay, ttl decremented.
  SensorSyncHeader a = mkhdr(origin, 10, 5);
  CHECK(ss_router_should_relay(a, self, tbl, N, &out), "first frame relays");
  CHECK(out == 4, "ttl decremented 5->4");

  // Same seq again (loop echo) -> dropped.
  CHECK(!ss_router_should_relay(a, self, tbl, N, &out), "duplicate seq not relayed");

  // Older seq -> dropped.
  SensorSyncHeader older = mkhdr(origin, 9, 5);
  CHECK(!ss_router_should_relay(older, self, tbl, N, &out), "older seq not relayed");

  // Newer seq -> relayed.
  SensorSyncHeader newer = mkhdr(origin, 11, 5);
  CHECK(ss_router_should_relay(newer, self, tbl, N, &out), "newer seq relayed");

  // TTL exhaustion: ttl==1 would reach 0 -> dropped (even though fresh seq).
  SensorSyncHeader lowttl = mkhdr(origin, 12, 1);
  CHECK(!ss_router_should_relay(lowttl, self, tbl, N, &out), "ttl==1 not relayed (budget exhausted)");

  // ...but the fresh seq 12 was still recorded, so a later echo of 12 is a dup, and 13 relays.
  SensorSyncHeader after = mkhdr(origin, 13, 4);
  CHECK(ss_router_should_relay(after, self, tbl, N, &out), "seq after exhausted-ttl frame still relays");
  SensorSyncHeader echo12 = mkhdr(origin, 12, 8);
  CHECK(!ss_router_should_relay(echo12, self, tbl, N, &out), "echo of seq recorded at low ttl is deduped");

  // Legacy ttl==0 -> inject SS_DEFAULT_TTL and relay with default-1.
  SensorRouterPeer tbl2[N]{};
  SensorSyncHeader legacy = mkhdr(2, 1, 0);
  CHECK(ss_router_should_relay(legacy, self, tbl2, N, &out), "legacy ttl==0 relays (default injected)");
  CHECK(out == SS_DEFAULT_TTL - 1, "legacy relay uses SS_DEFAULT_TTL-1");

  // Self-origin frame is never relayed.
  SensorRouterPeer tbl3[N]{};
  SensorSyncHeader mine = mkhdr(self, 5, 8);
  CHECK(!ss_router_should_relay(mine, self, tbl3, N, &out), "self-origin frame not relayed");
}

// ---- 3. 16-bit seq wraparound ----
static void test_wraparound() {
  const uint8_t N = 4;
  SensorRouterPeer tbl[N]{};
  const uint32_t self = 7, origin = 3;
  uint8_t out = 0;
  CHECK(ss_router_should_relay(mkhdr(origin, 0xFFFE, 6), self, tbl, N, &out), "0xFFFE fresh");
  CHECK(ss_router_should_relay(mkhdr(origin, 0xFFFF, 6), self, tbl, N, &out), "0xFFFF newer than 0xFFFE");
  CHECK(ss_router_should_relay(mkhdr(origin, 0x0000, 6), self, tbl, N, &out), "0x0000 newer than 0xFFFF (wrap)");
  CHECK(ss_router_should_relay(mkhdr(origin, 0x0001, 6), self, tbl, N, &out), "0x0001 newer than 0x0000");
  CHECK(!ss_router_should_relay(mkhdr(origin, 0xFFFF, 6), self, tbl, N, &out), "0xFFFF is older than 0x0001 (wrap-safe)");
}

// ---- 4. Graph flood simulation ----
struct SimNode {
  uint32_t id;
  SensorRouterPeer tbl[16];
  int  relayCount = 0;
  bool received   = false;
};
struct Tx { int from; SensorSyncHeader h; };

// Flood one origin frame across `adj` (N x N adjacency), return total transmissions processed.
// Caps iterations to prove termination rather than hang on a bug.
static int flood(std::vector<SimNode> &nodes, const std::vector<std::vector<bool>> &adj,
                 int origin, uint16_t seq, uint8_t ttl) {
  int N = (int)nodes.size();
  std::deque<Tx> q;
  nodes[origin].received = true;
  q.push_back(Tx{origin, mkhdr(nodes[origin].id, seq, ttl)});
  int processed = 0;
  const int CAP = 100000;   // termination backstop
  while (!q.empty() && processed < CAP) {
    Tx t = q.front(); q.pop_front(); processed++;
    for (int n = 0; n < N; n++) {
      if (n == t.from || !adj[t.from][n]) continue;
      nodes[n].received = true;                       // n heard the frame
      uint8_t out = 0;
      if (ss_router_should_relay(t.h, nodes[n].id, nodes[n].tbl, 16, &out)) {
        nodes[n].relayCount++;
        SensorSyncHeader h2 = t.h; h2.ttl = out;
        q.push_back(Tx{n, h2});
      }
    }
  }
  return processed;
}

static std::vector<SimNode> mknodes(int n) {
  std::vector<SimNode> v(n);
  for (int i = 0; i < n; i++) v[i].id = (uint32_t)(100 + i);  // distinct ids
  return v;
}

// Triangle A-B-C fully connected: origin A. Loop must terminate; B,C each relay exactly once;
// A never re-relays its own echo; all three received.
static void test_triangle_loop() {
  auto nodes = mknodes(3);
  std::vector<std::vector<bool>> adj(3, std::vector<bool>(3, true));
  for (int i = 0; i < 3; i++) adj[i][i] = false;
  int processed = flood(nodes, adj, /*origin=*/0, /*seq=*/42, SS_DEFAULT_TTL);
  CHECK(processed < 100000, "triangle flood terminates (loop-free)");
  CHECK(nodes[0].received && nodes[1].received && nodes[2].received, "all three nodes received");
  CHECK(nodes[0].relayCount == 0, "origin never relays its own echo");
  CHECK(nodes[1].relayCount == 1, "B relays exactly once");
  CHECK(nodes[2].relayCount == 1, "C relays exactly once");
}

// Line A-B-C-D-E: inject a small ttl so the flood dies before the far end -> TTL-bounded coverage.
static void test_line_ttl_cutoff() {
  auto nodes = mknodes(5);
  std::vector<std::vector<bool>> adj(5, std::vector<bool>(5, false));
  for (int i = 0; i + 1 < 5; i++) { adj[i][i+1] = adj[i+1][i] = true; }
  int processed = flood(nodes, adj, /*origin=*/0, /*seq=*/7, /*ttl=*/3);
  CHECK(processed < 100000, "line flood terminates");
  CHECK(nodes[0].received && nodes[1].received && nodes[2].received && nodes[3].received,
        "nodes within ttl budget received");
  CHECK(!nodes[4].received, "node beyond ttl budget did NOT receive (TTL bounds the flood)");

  // Each intermediate relays at most once.
  for (int i = 0; i < 5; i++) CHECK(nodes[i].relayCount <= 1, "no node relays a frame twice");
}

// Two edges (0,4) out of direct range, bridged by a router chain 1-2-3:
// an event from edge 0 reaches edge 4; no node relays twice.
static void test_bridged_edges() {
  auto nodes = mknodes(5);
  std::vector<std::vector<bool>> adj(5, std::vector<bool>(5, false));

  // 0-1-2-3-4 line: 0 and 4 cannot hear each other, only via the 1-2-3 relays.
  for (int i = 0; i + 1 < 5; i++) { adj[i][i+1] = adj[i+1][i] = true; }
  int processed = flood(nodes, adj, /*origin=*/0, /*seq=*/500, SS_DEFAULT_TTL);
  CHECK(processed < 100000, "bridged flood terminates");
  CHECK(nodes[4].received, "far edge received via multi-hop relay");
  for (int i = 0; i < 5; i++) CHECK(nodes[i].relayCount <= 1, "loop-free: each relays at most once");
}

// ---- 5. Leader-election metric (ss_router_beacon_better) ----
static void test_beacon_metric() {
  // Higher uptime wins regardless of deviceId.
  CHECK( ss_router_beacon_better(100, 1, 50, 999), "higher uptime wins (even with lower id)");
  CHECK(!ss_router_beacon_better(50, 999, 100, 1), "lower uptime loses (even with higher id)");

  // Equal uptime -> higher deviceId is the deterministic tiebreak.
  CHECK( ss_router_beacon_better(100, 7, 100, 3), "equal uptime: higher id wins");
  CHECK(!ss_router_beacon_better(100, 3, 100, 7), "equal uptime: lower id loses");

  // Not strictly better than self (identical metric) -> false (no needless flapping).
  CHECK(!ss_router_beacon_better(100, 5, 100, 5), "identical metric is not 'better'");

  // A just-rebooted router (uptime 0) never unseats an established leader.
  CHECK(!ss_router_beacon_better(0, 0xFFFFFFFF, 10, 1), "fresh reboot (uptime 0) does not win on id");
}

// ---- 6. Relay dedup-table full: eviction re-floods the evicted origin exactly once (bounded) ----
static void test_relay_eviction() {
  const uint8_t N = 2;                 // tiny table: only 2 origins tracked at once
  SensorRouterPeer tbl[N]{};
  const uint32_t self = 500;
  uint8_t out = 0;
  CHECK( ss_router_should_relay(mkhdr(1, 7, 6), self, tbl, N, &out), "origin 1 relays (slot 0)");
  CHECK( ss_router_should_relay(mkhdr(2, 7, 6), self, tbl, N, &out), "origin 2 relays (slot 1)");
  CHECK(!ss_router_should_relay(mkhdr(1, 7, 6), self, tbl, N, &out), "origin 1 same seq deduped (still tracked)");

  // Table full -> origin 3 evicts slot 0 (origin 1).
  CHECK( ss_router_should_relay(mkhdr(3, 7, 6), self, tbl, N, &out), "origin 3 relays, evicting slot 0");

  // Origin 1 was evicted -> its next frame is treated as fresh and re-floods ONCE (bounded).
  CHECK( ss_router_should_relay(mkhdr(1, 7, 6), self, tbl, N, &out), "evicted origin 1 re-floods once");

  // ...but immediately deduped again now that it re-occupies a slot -> no runaway loop.
  CHECK(!ss_router_should_relay(mkhdr(1, 7, 6), self, tbl, N, &out), "re-admitted origin 1 deduped again (bounded)");
}

int main() {
  test_wire_compat();
  test_unit();
  test_wraparound();
  test_triangle_loop();
  test_line_ttl_cutoff();
  test_bridged_edges();
  test_beacon_metric();
  test_relay_eviction();
  test_relayable_types();
  test_control_relay_semantics();
  if (g_fail) { printf("SOME TESTS FAILED\n"); return 1; }
  printf("ALL TESTS PASSED\n");
  return 0;
}
