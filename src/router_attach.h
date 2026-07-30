#pragma once
//
// router_attach.h — the attach protocol's PURE logic (host-testable, no Arduino), in two halves:
//
//   * Membership. A router keeps a table of the nodes (WLED edges and downstream routers) that
//     have attached to it, each with the time it was last heard. A node re-sends its attach as a
//     keepalive; a member that goes quiet past its lease is expired, which is what makes the mesh
//     shrink back when a node disappears.
//   * Route selection. Every node ranks the router heartbeats it hears and binds to the best one:
//     fewest hops to the timebase leader, then least loaded, with the deviceId as a deterministic
//     tiebreak. A hold-down keeps a marginally-better neighbour from causing the choice to flap,
//     and a heartbeat timeout drops the route so the next heartbeat can be adopted immediately.
//     The two tables meet here: a node that is attached to us is below us in the tree, so it is
//     never adopted as our upstream, and a pair that adopted each other before their attaches
//     landed is broken apart rather than left circling.
//
// The wire structs these operate on (RouterAdvert / NodeAttach / AttachAck) and their msgType
// numbers live in the shared sensor_sync_protocol.h so both sides agree on the format. The WLED
// edge currently only reserves that format — ss_parse_header rejects these msgTypes, so the
// handshake runs router-to-router and an edge does not yet bind to a router. Nothing here reads a
// clock — `now` is always passed in — so it unit-tests on a host.
//
#include <stdint.h>
#include "sensor_sync_protocol.h"   // RouterAdvert, NodeAttach, AttachAck, SS_HOP_UNREACHABLE

// --- Control-frame header -----------------------------------------------------------------------
// Fill the fixed part of an outgoing control-frame header. Shared by every control-frame sender —
// the election beacon, the router advertisement, and both halves of the attach handshake — so the
// magic, version and single-hop TTL cannot drift between them. `sensorType` is meaningless outside
// a snapshot and is zeroed.
static inline void ra_stamp_control_header(SensorSyncHeader &h, uint32_t selfId, uint8_t msgType,
                                           uint8_t dataLen, uint16_t seq, uint32_t now) {
  h.magic[0] = 'A'; h.magic[1] = 'M'; h.magic[2] = 'P'; h.magic[3] = 'S';
  h.version    = SENSOR_SYNC_VERSION;
  h.msgType    = msgType;
  h.sensorType = 0;
  h.dataLen    = dataLen;
  h.deviceId   = selfId;
  h.seq        = seq;
  h.ttl        = 1;          // control frames are single-hop; no router relays them
  h.flags      = 0;
  h.timestamp  = now;
}

// --- Membership table ---------------------------------------------------------------------------
// One entry per node currently attached to this router. Liveness only: attach frames are single-hop
// keepalives, so a duplicate can be at most one radio hop old and refreshing from it is harmless.
// Deliberately no sequence-number dedup — it would strand a node that rebooted and restarted its
// sequence at 0 until the counter climbed back past the stored value.
struct AttachedNode {
  uint32_t deviceId;
  uint32_t lastSeenMs;   // when this node's most recent attach arrived
  bool     used;
};

// Return the table slot held by `dev`, or null when that node is not attached.
static inline AttachedNode *ra_member_find(AttachedNode *tbl, uint8_t maxEntries, uint32_t dev) {
  for (uint8_t i = 0; i < maxEntries; i++)
    if (tbl[i].used && tbl[i].deviceId == dev) return &tbl[i];
  return 0;
}

// Admit `dev` as a member, or refresh it if already attached. Returns the slot, or null when the
// table is full — a full table means the attach is not acked, so the node re-tries and lands
// wherever a lease has since expired, instead of silently believing it is attached.
static inline AttachedNode *ra_member_attach(AttachedNode *tbl, uint8_t maxEntries, uint32_t dev,
                                             uint32_t now) {
  AttachedNode *p = ra_member_find(tbl, maxEntries, dev);
  if (p) { p->lastSeenMs = now; return p; }

  for (uint8_t i = 0; i < maxEntries; i++)
    if (!tbl[i].used) { tbl[i] = AttachedNode{dev, now, true}; return &tbl[i]; }

  return 0;
}

// Release members whose lease has run out (no attach within `leaseMs`). Returns how many were
// dropped, which the caller logs as the visible signal that part of the mesh went away.
static inline uint8_t ra_member_expire(AttachedNode *tbl, uint8_t maxEntries, uint32_t now,
                                       uint32_t leaseMs) {
  uint8_t dropped = 0;
  for (uint8_t i = 0; i < maxEntries; i++) {
    if (!tbl[i].used) continue;
    if ((uint32_t)(now - tbl[i].lastSeenMs) > leaseMs) { tbl[i] = AttachedNode{}; dropped++; }
  }
  return dropped;
}

// Is `dev` currently attached to us? A member sits below us in the tree, which is what makes this
// the loop test used by route selection.
static inline bool ra_member_attached(const AttachedNode *tbl, uint8_t maxEntries, uint32_t dev) {
  for (uint8_t i = 0; i < maxEntries; i++)
    if (tbl[i].used && tbl[i].deviceId == dev) return true;
  return false;
}

// Count the live members. This is the load term a router advertises so nodes spread themselves
// across routers rather than all piling onto whichever one they hear first.
static inline uint8_t ra_member_count(const AttachedNode *tbl, uint8_t maxEntries) {
  uint8_t n = 0;
  for (uint8_t i = 0; i < maxEntries; i++) if (tbl[i].used) n++;
  return n;
}

// --- Route selection ----------------------------------------------------------------------------
// The router this node is currently bound to, plus the metric it last advertised. routerId == 0
// means no route is held: the node is unattached and will take the first usable heartbeat it hears.
struct RouteChoice {
  uint32_t routerId;     // deviceId of the chosen upstream router (0 = none)
  uint32_t leaderId;     // timebase leader that router routes toward
  uint16_t leaderTerm;   // leadership term its metric was computed in
  uint8_t  hopCost;      // its advertised distance to the leader
  uint8_t  memberCount;  // its advertised load
  uint32_t lastHeardMs;  // when its most recent heartbeat arrived (freshness)
  uint32_t chosenMs;     // when this choice was adopted (hold-down base)
  bool     attached;     // true once that router has acked our attach
};

// Fresh state: unattached, with no router in mind.
static inline RouteChoice ra_route_init() {
  return RouteChoice{0, 0, 0, 0, 0, 0, 0, false};
}

// Is a route currently held? Callers use this to decide whether to send an attach keepalive.
static inline bool ra_route_held(const RouteChoice &r) {
  return r.routerId != 0;
}

// Forget the current route, leaving the node unattached and free to adopt the next heartbeat.
static inline void ra_route_clear(RouteChoice &r) {
  r = ra_route_init();
}

// Selection metric: is the candidate router strictly preferable to the incumbent? Fewest hops to
// the timebase leader wins, because that is what bounds delivery latency; ties go to the router
// carrying fewer members, so load spreads; the deviceId settles the rest (globally unique 32-bit
// MAC hash, so two routers never compare exactly equal and every node picks the same winner).
static inline bool ra_advert_better(uint8_t candCost, uint8_t candLoad, uint32_t candId,
                                    uint8_t curCost, uint8_t curLoad, uint32_t curId) {
  if (candCost != curCost) return candCost < curCost;
  if (candLoad != curLoad) return candLoad < curLoad;
  return candId > curId;
}

// Adopt `senderId` as the upstream router, resetting the hold-down and the attach handshake.
static inline void ra_route_adopt(RouteChoice &r, uint32_t senderId, const RouterAdvert &a,
                                  uint32_t now) {
  r.routerId    = senderId;
  r.leaderId    = a.leaderId;
  r.leaderTerm  = a.leaderTerm;
  r.hopCost     = a.hopCost;
  r.memberCount = a.memberCount;
  r.lastHeardMs = now;
  r.chosenMs    = now;
  r.attached    = false;
}

// Feed in a router heartbeat. `senderIsDownstream` says whether the sender is attached to us, i.e.
// sits below us in the tree — pass ra_member_attached() for it. Returns true iff the bound router
// changed (adopted, switched, or dropped), which is the caller's cue to send a fresh attach. Rules,
// in order:
//   0. A router advertising SS_HOP_UNREACHABLE has no path to the leader, so it is never selected;
//      if it is the router we are bound to, the route is dropped rather than kept as a dead end.
//   1. Our own router's heartbeat refreshes freshness and re-reads its metric — the metric can
//      worsen without unbinding us, so a whole-mesh cost change does not deselect everything at once.
//   2. A node below us is never adopted: routing up through our own subtree is a loop.
//   3. With no route held, the first usable heartbeat is adopted outright.
//   4. Otherwise the candidate must beat the incumbent on the selection metric, and within
//      `holdDownMs` of the last change it must beat it on hop count specifically — a genuinely
//      shorter path preempts immediately, a mere load or tiebreak edge waits out the hold-down.
static inline bool ra_on_advert(RouteChoice &r, uint32_t senderId, const RouterAdvert &a,
                                bool senderIsDownstream, uint32_t now, uint32_t holdDownMs) {
  if (senderId == 0) return false;

  if (a.hopCost >= SS_HOP_UNREACHABLE) {
    if (r.routerId != senderId) return false;
    ra_route_clear(r);
    return true;
  }

  if (r.routerId == senderId) {
    r.leaderId    = a.leaderId;
    r.leaderTerm  = a.leaderTerm;
    r.hopCost     = a.hopCost;
    r.memberCount = a.memberCount;
    r.lastHeardMs = now;
    return false;
  }

  if (senderIsDownstream) return false;

  if (r.routerId == 0) { ra_route_adopt(r, senderId, a, now); return true; }

  // A cost is only meaningful relative to the leadership term it was computed in. After a failover
  // the old leader's tree is gone, so an advert still carrying the superseded term describes a path
  // that may no longer exist: never let it displace an incumbent from the current term. The reverse
  // is a re-formed mesh, so the fresher view preempts immediately instead of waiting out the
  // hold-down, which is what keeps a stale-but-cheap-looking route from pinning the node.
  if (a.leaderTerm != r.leaderTerm) {
    if (ss_seq_newer(r.leaderTerm, a.leaderTerm)) return false;
    ra_route_adopt(r, senderId, a, now);
    return true;
  }

  if (!ra_advert_better(a.hopCost, a.memberCount, senderId, r.hopCost, r.memberCount, r.routerId))
    return false;

  const bool holdDownOver = (uint32_t)(now - r.chosenMs) >= holdDownMs;
  if (!holdDownOver && a.hopCost >= r.hopCost) return false;

  ra_route_adopt(r, senderId, a, now);
  return true;
}

// Feed in an attach ack. Confirms the handshake only when it comes from the router we are bound to
// and names us, so a broadcast ack meant for another node is ignored. Returns true iff it confirmed
// an attach that was still pending.
static inline bool ra_on_ack(RouteChoice &r, uint32_t senderId, const AttachAck &ack,
                             uint32_t selfId, uint32_t now) {
  if (r.routerId == 0 || senderId != r.routerId) return false;
  if (ack.nodeId != selfId) return false;

  r.lastHeardMs = now;
  if (r.attached) return false;

  r.attached = true;
  return true;
}

// Periodic liveness check: drop the route when the bound router has been silent for longer than
// `timeoutMs`. Returns true iff a route was dropped, which is the self-healing trigger — the node
// is unattached again and rule 2 of ra_on_advert re-binds it to whichever router it still hears.
static inline bool ra_route_tick(RouteChoice &r, uint32_t now, uint32_t timeoutMs) {
  if (r.routerId == 0) return false;
  if ((uint32_t)(now - r.lastHeardMs) <= timeoutMs) return false;

  ra_route_clear(r);
  return true;
}

// Break a two-node routing loop. Two nodes can adopt each other in the window before either one's
// attach arrives, after which each is both the other's upstream and its member. Neither can then
// reach the leader, so dropping the route re-roots this node on the next advertisement it hears.
// Returns true iff a route was dropped.
//
// Scope: this detects 2-cycles only, because membership is the single hop of topology a node can
// see. A longer cycle (A -> B -> C -> A, formed when an orphaned router adopts a grandchild's
// pre-failover advert) is not detected here and instead unwinds through ra_advert_cost saturation:
// each lap adds a hop until the cost reaches SS_HOP_UNREACHABLE, at which point rule 0 of
// ra_on_advert drops the route. That bounds the cycle rather than preventing it, so recovery is
// SS_HOP_UNREACHABLE laps of the advert interval, not one timeout. Cheaper than carrying a path
// vector; if that recovery window proves too slow on real hardware, poison-reverse is the next
// step.
static inline bool ra_route_break_loop(RouteChoice &r, const AttachedNode *tbl, uint8_t maxEntries) {
  if (r.routerId == 0 || !ra_member_attached(tbl, maxEntries, r.routerId)) return false;

  ra_route_clear(r);
  return true;
}

// The hop cost this node advertises to others: 0 if it owns the timebase, one more than its
// upstream router if it has a route, and SS_HOP_UNREACHABLE while it has neither. Saturating, so a
// long chain reports "unreachable" rather than wrapping to a short, and therefore attractive, cost.
static inline uint8_t ra_advert_cost(bool isLeader, const RouteChoice &r) {
  if (isLeader) return 0;
  if (r.routerId == 0) return SS_HOP_UNREACHABLE;
  if (r.hopCost >= SS_HOP_UNREACHABLE - 1) return SS_HOP_UNREACHABLE;
  return (uint8_t)(r.hopCost + 1);
}
