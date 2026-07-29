#pragma once
//
// router_election.h — the backbone router's leader-election state machine, as PURE logic so it can
// be unit-tested on a host build (no Arduino / WiFi / ESP-NOW). main.cpp holds one RouterElection
// and drives it from received beacons (re_on_beacon) and the periodic tick (re_tick); time and the
// uptime metric are passed in so nothing here touches millis()/esp_timer.
//
// Election rule (see BACKBONE_ROUTER.md): highest (uptimeTicks, deviceId) wins; a router starts as
// a FOLLOWER and only asserts leadership after a hold-down with no better beacon; it tracks the
// BEST leader heard (not the latest) and uses beacon `term` to reject stale beacons from that
// leader. Degenerates cleanly to N=1.
//
#include <stdint.h>
#include "router_relay.h"           // ss_router_beacon_better (+ shared wire types via it)

struct RouterElection {
  bool     isLeader;       // are we the timebase leader?
  uint16_t term;           // our leadership term (bumped each time we take over)
  uint32_t leaderId;       // deviceId of the best leader currently tracked (0 = none/self)
  uint32_t leaderUptime;   // that leader's beaconed uptime metric
  uint16_t leaderTerm;     // that leader's beaconed term (stale-beacon guard)
  uint32_t lastBetterMs;   // when the tracked leader was last heard (liveness / boot hold-down)
};

// Fresh state: a follower with no known leader. lastBetterMs = 0 doubles as the boot hold-down
// start (re_tick promotes only once `now` exceeds the timeout).
static inline RouterElection re_init() {
  return RouterElection{ false, 0, 0, 0, 0, 0 };
}

// Process a beacon from `senderId` carrying (senderUptime, senderTerm). `selfUptime` is our own
// current uptime metric, `now` a monotonic ms clock. Adopts the sender as our leader when it
// outranks us AND is at least as good as the leader we already track (best-heard, not latest),
// refreshing liveness on the tracked leader's own beacons while ignoring stale (older-term) ones.
static inline void re_on_beacon(RouterElection &e, uint32_t senderId, uint32_t senderUptime,
                                uint16_t senderTerm, uint32_t selfId, uint32_t selfUptime,
                                uint32_t now) {
  if (!ss_router_beacon_better(senderUptime, senderId, selfUptime, selfId)) return;  // doesn't outrank us

  const bool sameLeader = (!e.isLeader && senderId == e.leaderId);
  if (sameLeader && senderTerm < e.leaderTerm) return;   // stale beacon from our leader — don't refresh
  if (e.isLeader || sameLeader ||
      ss_router_beacon_better(senderUptime, senderId, e.leaderUptime, e.leaderId)) {
    e.isLeader     = false;                              // step down / stay follower
    e.leaderId     = senderId;
    e.leaderUptime = senderUptime;
    e.leaderTerm   = senderTerm;
    e.lastBetterMs = now;
  }
}

// Periodic tick: if no better beacon arrived within `timeoutMs` (also the boot hold-down, since
// lastBetterMs starts at 0) we take leadership with a fresh term. Returns true iff we just did.
static inline bool re_tick(RouterElection &e, uint32_t selfId, uint32_t selfUptime,
                           uint32_t now, uint32_t timeoutMs) {
  if (!e.isLeader && (now - e.lastBetterMs) > timeoutMs) {
    e.isLeader     = true;
    e.leaderId     = selfId;
    e.leaderUptime = selfUptime;
    e.term++;
    return true;
  }
  return false;
}
