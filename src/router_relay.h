#pragma once
//
// router_relay.h — the backbone router's pure relay + election-metric logic (host-testable, no
// Arduino). This lives in the ROUTER repo, not the shared WLED wire header: the WLED edge never
// calls any of it. It only *uses* the shared wire types (SensorSyncHeader, ss_seq_newer,
// SS_DEFAULT_TTL) and the reserved msgType/flag numbers, which stay in sensor_sync_protocol.h so
// both sides agree on the on-the-wire format.
//
#include <stdint.h>
#include "sensor_sync_protocol.h"   // SensorSyncHeader, ss_seq_newer, SS_DEFAULT_TTL, SENSOR_SYNC_MSG_BEACON

// --- Multi-hop relay (loop-free flood) ------------------------------------------------------
// Per-origin dedup state a router keeps to decide whether a frame is fresh. Distinct from the
// edge's SensorPeer (which also derives events); a relay only needs (deviceId -> lastSeq).
struct SensorRouterPeer {
  uint32_t deviceId;
  uint16_t lastSeq;
  bool     used;
  bool     haveSeq;   // false until the first frame from this origin sets lastSeq
};

// Find/allocate the relay-dedup slot for `dev`. Reuses a free slot; if the table is full, evicts
// slot 0 (that origin then re-floods once on its next frame — bounded, not a loop).
static inline SensorRouterPeer *ss_router_relay_slot(SensorRouterPeer *tbl, uint8_t maxEntries, uint32_t dev) {
  for (uint8_t i = 0; i < maxEntries; i++)
    if (tbl[i].used && tbl[i].deviceId == dev) return &tbl[i];
  for (uint8_t i = 0; i < maxEntries; i++)
    if (!tbl[i].used) { tbl[i] = SensorRouterPeer{dev, 0, true, false}; return &tbl[i]; }
  tbl[0] = SensorRouterPeer{dev, 0, true, false};
  return &tbl[0];
}

// Does this msgType travel multi-hop? Sensor snapshots and UI/preset commands do: both are
// installation-wide state that every node needs. The router's own plane — the election beacon, the
// heartbeat and the attach handshake — is single-hop and consumed by whichever node hears it, so
// relaying one would leak a local routing decision across the whole mesh.
//
// This is a whitelist, not a blacklist, so a msgType added later stays single-hop until someone
// deliberately lists it here. TIMEBASE is absent on purpose: it is reserved but unimplemented, and
// a beacon whose whole value is *when* it arrived should not be forwarded by a hop that adds
// unmeasured delay. Whoever implements it owns that decision.
static inline bool ss_router_is_relayable(uint8_t msgType) {
  return msgType == SENSOR_SYNC_MSG_SNAPSHOT ||
         msgType == SENSOR_SYNC_MSG_CONTROL;
}

// Relay decision for a router. Returns true iff `h` should be re-broadcast, and writes the TTL to
// stamp on the outgoing copy to *outTtl. `selfId` is this node's deviceId. Rules:
//   0a. Non-relayable types (the router's own beacon/heartbeat/attach plane) are single-hop -> drop.
//   0b. Self: never relay a frame we originated (its echo looping back) -> drop. Lets a node that is
//      both an edge (originator) and a relay coexist; for a pure router selfId never matches.
//   1. Dedup: a frame whose seq is not newer than the last seen from this origin is a duplicate
//      (a loop echo or a reorder) -> drop. Primary loop terminator. First frame from an origin is
//      always fresh. Dedup state advances only for fresh frames.
//   2. TTL: a frame arriving with ttl==0 (sender predates the field) is treated as SS_DEFAULT_TTL. A
//      frame with ttl<=1 has exhausted its hop budget -> drop. Otherwise the outgoing TTL is eff-1.
// The seen table advances (seq) only for fresh frames, regardless of TTL, so a short-TTL frame
// still suppresses its own later loop echoes.
static inline bool ss_router_should_relay(const SensorSyncHeader &h, uint32_t selfId,
                                          SensorRouterPeer *tbl, uint8_t maxEntries, uint8_t *outTtl) {
  if (!ss_router_is_relayable(h.msgType)) return false;             // single-hop control frame
  if (h.deviceId == selfId) return false;                           // never relay our own echo
  SensorRouterPeer *p = ss_router_relay_slot(tbl, maxEntries, h.deviceId);
  if (p->haveSeq && !ss_seq_newer(h.seq, p->lastSeq)) return false;  // duplicate / reorder
  p->lastSeq = h.seq;
  p->haveSeq = true;
  uint8_t eff = h.ttl ? h.ttl : SS_DEFAULT_TTL;   // legacy/unset -> inject default
  if (eff <= 1) return false;                     // hop budget exhausted
  if (outTtl) *outTtl = (uint8_t)(eff - 1);
  return true;
}

// --- Leader election metric ------------------------------------------------------------------
// Beacon payload (follows the header when msgType == SENSOR_SYNC_MSG_BEACON). The origin deviceId
// is the header's deviceId; the payload adds the election metric. Beacons are single-hop (routers
// do NOT relay them; edges reject them via ss_parse_header's msgType check).
struct __attribute__((packed)) RouterBeacon {
  uint32_t uptimeTicks;  // monotonic liveness (higher = longer-lived -> preferred leader)
  uint16_t term;         // increments on each new leadership; breaks stale-beacon races
};

// Election metric: is (challengerUptime, challengerId) strictly better than
// (incumbentUptime, incumbentId)? Longest-lived wins; deviceId is the deterministic tiebreak
// (globally unique, so never an exact tie). Uptime-first avoids flapping to a just-rebooted node.
static inline bool ss_router_beacon_better(uint32_t challengerUptime, uint32_t challengerId,
                                           uint32_t incumbentUptime, uint32_t incumbentId) {
  if (challengerUptime != incumbentUptime) return challengerUptime > incumbentUptime;
  return challengerId > incumbentId;
}
