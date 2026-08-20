#pragma once
//
// coordinator_phase.h — own-AP onboarding phase sequencer. Pure, host-testable.
//
// The problem this solves: in own-AP mode the coordinator's SoftAP IS the network it is onboarding
// devices onto, and it must be DOWN while hopping (a hop drags the radio to the target's channel and
// drops every already-onboarded client, since stock WLED does no channel-switch-announce). So a
// freshly-onboarded target reboots into credentials for an AP that does not exist yet.
//
// That window self-heals, but only if the phases are ordered and the settle phase is long enough.
// All three timings below are READ FROM WLED, not guessed:
//
//   WLED_STA_RETRY_MS   18000   wled.cpp:947  `now - lastReconnectAttempt > ((stac) ? 300000 : 18000)`
//   WLED_STA_RETRY_STAC 300000  same line — the backoff while a station is associated to the target
//   WLED_AP_FALLBACK_MS 12000   wled.cpp:953  `!apActive && now - lastReconnectAttempt > 12000 && ...`
//
// The retry at :947 is NOT gated on !apActive, so an orphan sitting on its own fallback AP keeps
// knocking — which is what makes the settle phase work at all.
//
#include <stdint.h>
#include <stddef.h>

#define WLED_STA_RETRY_MS    18000u
#define WLED_STA_RETRY_STAC 300000u
#define WLED_AP_FALLBACK_MS  12000u

// How long to hold the AP up before trusting discovery. One full retry interval is the MINIMUM for
// an orphan to get a chance at all; the margin covers a target whose retry timer just fired as our
// AP came up, plus DHCP and mDNS. Under-running this is silent: discovery simply finds fewer
// devices and the sync looks like it "missed some".
#ifndef COORD_SETTLE_MS
#define COORD_SETTLE_MS (WLED_STA_RETRY_MS + 12000u)   /* 30 s */
#endif

enum coord_phase_t {
  COORD_PHASE_IDLE = 0,
  COORD_PHASE_HOP,      // AP down, hopping to each candidate
  COORD_PHASE_SETTLE,   // home, AP up, waiting for orphans to converge
  COORD_PHASE_SYNC,     // discover + fan out
};

struct coord_phase_state_t {
  coord_phase_t phase;
  uint32_t      phase_started_ms;
  uint32_t      settle_ms;
};

static inline void coord_phase_init(coord_phase_state_t *s, uint32_t settle_ms) {
  if (!s) return;
  s->phase = COORD_PHASE_IDLE;
  s->phase_started_ms = 0;
  /* A settle shorter than one retry interval cannot work — an orphan would not get a single
   * chance to knock. Clamp rather than trust the caller: the failure is silent (fewer devices
   * found), so it would not announce itself. */
  s->settle_ms = (settle_ms < WLED_STA_RETRY_MS) ? WLED_STA_RETRY_MS : settle_ms;
}

static inline void coord_phase_enter(coord_phase_state_t *s, coord_phase_t p, uint32_t now_ms) {
  if (!s) return;
  s->phase = p;
  s->phase_started_ms = now_ms;
}

// Wrap-safe, like the hop budget: unsigned subtraction stays correct across the millis() rollover.
static inline uint32_t coord_phase_elapsed(const coord_phase_state_t *s, uint32_t now_ms) {
  if (!s) return 0;
  return (uint32_t)(now_ms - s->phase_started_ms);
}

// True once the AP has been up long enough that an orphan has had a real chance to rejoin.
static inline bool coord_settle_complete(const coord_phase_state_t *s, uint32_t now_ms) {
  if (!s || s->phase != COORD_PHASE_SETTLE) return false;
  return coord_phase_elapsed(s, now_ms) >= s->settle_ms;
}

// Whether the coordinator's own SoftAP should be up in this phase. Hopping with the AP up is the
// mistake this exists to make un-writable: it is the one combination that drops onboarded clients.
static inline bool coord_ap_should_be_up(const coord_phase_state_t *s) {
  if (!s) return false;
  return s->phase != COORD_PHASE_HOP;
}

// Discovery is only meaningful after the settle window; running it in HOP or mid-SETTLE finds a
// subset and makes the sync look flaky rather than early.
static inline bool coord_may_discover(const coord_phase_state_t *s, uint32_t now_ms) {
  if (!s) return false;
  if (s->phase == COORD_PHASE_SYNC) return true;
  return coord_settle_complete(s, now_ms);
}
