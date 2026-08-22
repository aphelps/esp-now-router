#pragma once
//
// coordinator_boot.h — what to do on the boot AFTER a hop went wrong. Pure, host-testable.
//
// The return-home design reboots out of a wedged hop (watchdog, or an explicit reset). That
// recovers the device but NOT the situation: a reboot discards RAM, so on the way back up the
// coordinator re-scans, re-picks the same unreachable target, wedges identically, and reboots
// again. Forever. The reboot is only a recovery if something survives it that stops the loop.
//
// So two things must be persisted BEFORE each hop, and consulted on every boot:
//   * which target was in flight, and
//   * how many times it has already been attempted.
//
// The decision itself is pure and lives here; the NVS read/write is glue.
//
#include <stdint.h>
#include <stddef.h>

// Mirrors esp_reset_reason() without dragging in the IDF header, so this stays host-testable.
// Only the distinction that matters is modelled: "did we come up because something went wrong".
enum coord_reset_kind_t {
  COORD_RESET_NORMAL = 0,   // power-on, or a deliberate restart
  COORD_RESET_FAULT,        // watchdog, panic, brownout — a hop that did not come home
};

struct coord_boot_input_t {
  coord_reset_kind_t reset;
  bool     had_hop_in_flight;   // persisted before the hop, cleared on a clean return
  uint64_t in_flight_mac;
  uint8_t  attempts;            // how many times this target has already been tried
};

#ifndef COORD_MAX_ATTEMPTS
#define COORD_MAX_ATTEMPTS 2
#endif

enum coord_boot_action_t {
  COORD_BOOT_RESUME = 0,        // nothing was in flight; carry on normally
  COORD_BOOT_RETRY,             // faulted, but this target has attempts left
  COORD_BOOT_BLACKLIST,         // faulted too often — never hop to this one again
};

// Decide what a boot means.
//
// A fault with a hop in flight is the ONLY case that can loop, so it is the only case that
// consumes an attempt. A normal boot clears the record rather than counting it: a deliberate
// restart mid-hop (an operator power-cycling the box) is not evidence the target is bad, and
// counting it would blacklist innocent devices.
static inline coord_boot_action_t coord_boot_decide(const coord_boot_input_t &in) {
  if (!in.had_hop_in_flight) return COORD_BOOT_RESUME;
  if (in.reset != COORD_RESET_FAULT) return COORD_BOOT_RESUME;
  if (in.attempts + 1 >= COORD_MAX_ATTEMPTS) return COORD_BOOT_BLACKLIST;
  return COORD_BOOT_RETRY;
}

// The attempt count to persist after acting on the decision above.
static inline uint8_t coord_boot_next_attempts(const coord_boot_input_t &in,
                                               coord_boot_action_t action) {
  if (action == COORD_BOOT_RESUME) return 0;            // record cleared
  if (in.attempts >= 0xFF) return 0xFF;                  // saturate rather than wrap to 0
  return (uint8_t)(in.attempts + 1);
}
