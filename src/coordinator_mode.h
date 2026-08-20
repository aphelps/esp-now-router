#pragma once
//
// coordinator_mode.h — the WLED-coordinator's mode and hop-permission logic. Pure, host-testable.
//
// Every safety property of this role is a rule about WHEN a hop is allowed, so they all live here
// rather than being scattered through the radio glue where they cannot be tested:
//
//   1. Never hop while operating.  Hopping drags the radio to the target's channel, which breaks
//      the ESP-NOW backbone and, in own-AP mode, drops every device already onboarded (stock WLED
//      does no channel-switch-announce, so clients do not follow — they are dropped). During a show
//      that is unacceptable, and core 1 is unaffected by an idle API, so waiting is always better.
//   2. Every hop is bounded.  A hop with no deadline is how a coordinator strands itself on a dead
//      AP with no operator and no network.
//   3. A target that wedged us is not retried.  A watchdog reboot discards RAM, so without a
//      persisted brake the coordinator re-scans, re-picks the same target and wedges identically,
//      forever.
//
#include <stdint.h>
#include <stddef.h>

#ifndef COORD_MAX_BLACKLIST
#define COORD_MAX_BLACKLIST 8
#endif

enum coord_mode_t {
  COORD_MODE_ONBOARD = 0,   // ESP-NOW down, hopping permitted, backbone deliberately down
  COORD_MODE_OPERATE = 1,   // ESP-NOW up, hopping FORBIDDEN, JSON over IP only
};

// Why a hop was refused. Reported rather than folded into a bool so a refusal is diagnosable from
// the outside — "it just did not onboard" is the failure mode this avoids.
enum coord_hop_verdict_t {
  COORD_HOP_PERMIT = 0,
  COORD_HOP_REFUSE_OPERATING,     // rule 1
  COORD_HOP_REFUSE_BLACKLISTED,   // rule 3
  COORD_HOP_REFUSE_IN_FLIGHT,     // a hop is already running
};

struct coord_state_t {
  coord_mode_t mode;

  bool     hop_in_flight;
  uint32_t hop_started_ms;
  uint32_t hop_budget_ms;

  uint64_t blacklist[COORD_MAX_BLACKLIST];   // target MACs that wedged us
  uint8_t  blacklist_len;
};

static inline void coord_init(coord_state_t *s, uint32_t hop_budget_ms) {
  if (!s) return;
  s->mode = COORD_MODE_ONBOARD;
  s->hop_in_flight = false;
  s->hop_started_ms = 0;
  s->hop_budget_ms = hop_budget_ms;
  s->blacklist_len = 0;
  for (size_t i = 0; i < COORD_MAX_BLACKLIST; i++) s->blacklist[i] = 0;
}

static inline bool coord_is_blacklisted(const coord_state_t *s, uint64_t mac) {
  if (!s) return false;
  for (uint8_t i = 0; i < s->blacklist_len; i++) if (s->blacklist[i] == mac) return true;
  return false;
}

// Returns false when the list is full. Full is NOT silently ignored: a coordinator that has wedged
// on eight distinct targets is in a situation nobody designed for, and the caller should say so
// rather than quietly resume retrying the ninth forever.
static inline bool coord_blacklist_add(coord_state_t *s, uint64_t mac) {
  if (!s || coord_is_blacklisted(s, mac)) return true;
  if (s->blacklist_len >= COORD_MAX_BLACKLIST) return false;
  s->blacklist[s->blacklist_len++] = mac;
  return true;
}

static inline coord_hop_verdict_t coord_may_hop(const coord_state_t *s, uint64_t target_mac) {
  if (!s) return COORD_HOP_REFUSE_OPERATING;      // fail closed
  if (s->mode == COORD_MODE_OPERATE) return COORD_HOP_REFUSE_OPERATING;
  if (s->hop_in_flight) return COORD_HOP_REFUSE_IN_FLIGHT;
  if (coord_is_blacklisted(s, target_mac)) return COORD_HOP_REFUSE_BLACKLISTED;
  return COORD_HOP_PERMIT;
}

static inline bool coord_hop_begin(coord_state_t *s, uint64_t target_mac, uint32_t now_ms) {
  if (coord_may_hop(s, target_mac) != COORD_HOP_PERMIT) return false;
  s->hop_in_flight = true;
  s->hop_started_ms = now_ms;
  return true;
}

static inline void coord_hop_end(coord_state_t *s) {
  if (!s) return;
  s->hop_in_flight = false;
}

// Subtraction is unsigned and wrap-safe, so this stays correct across the ~49-day millis() rollover
// rather than permitting an unbounded hop once every seven weeks.
static inline bool coord_hop_expired(const coord_state_t *s, uint32_t now_ms) {
  if (!s || !s->hop_in_flight) return false;
  return (uint32_t)(now_ms - s->hop_started_ms) >= s->hop_budget_ms;
}

// Entering operate mode always ends any hop in flight: the two states are mutually exclusive, and
// leaving hop_in_flight set would make coord_may_hop refuse forever after a mode change.
static inline void coord_enter_operate(coord_state_t *s) {
  if (!s) return;
  s->hop_in_flight = false;
  s->mode = COORD_MODE_OPERATE;
}

static inline void coord_enter_onboard(coord_state_t *s) {
  if (!s) return;
  s->mode = COORD_MODE_ONBOARD;
}
