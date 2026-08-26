#pragma once
// coordinator.h — the radio glue that turns the pure decision layer into a device that finds,
// onboards and drives stock WLED devices with no laptop involved.
//
// Everything policy-shaped already lives in headers next to this one and is host-tested:
//   wled_candidate.h    which scan results are worth hopping to, and the push log that stops the
//                       own-AP re-candidacy loop
//   wled_onboard_cfg.h  the /json/cfg body that joins a target to our network
//   wled_translate.h    the /json/state body that sets a colour
//   coordinator_mode.h  when hopping is permitted at all
//   coordinator_phase.h hop -> settle -> sync sequencing
//   coordinator_boot.h  the reboot-loop brake
//
// This file owns only the parts that need a radio: scanning, joining, HTTP, discovery, and the
// NVS write that makes "return home" survive a watchdog reset. Compiled only when
// WLED_COORDINATOR_ROLE is defined, so the plain router build is byte-identical without it.
#include <stdint.h>

void coordinatorSetup();
void coordinatorLoop(uint32_t now);

// Status for the HTTP surface (main.cpp owns the server).
struct CoordinatorStatus {
  const char *phase;
  uint8_t     targetR, targetG, targetB;
  uint16_t    onboarded;     // devices pushed this run
  uint16_t    synced;        // devices whose colour we set on the last sync sweep
  uint16_t    discovered;    // devices seen on the home LAN on the last sweep
  uint32_t    hops;          // AP hops attempted
  uint32_t    hopFailures;
  bool        homeUp;
  uint8_t     selfHost;      // our own last octet — the ONE address the sweep skips
  uint8_t     sweepAt;       // sweep cursor, 0 when not sweeping
  uint16_t    retryHits;     // devices that were found ONLY because the probe was retried
  uint16_t    sweepFoundNow; // devices found so far in the CURRENT pass (discovered only publishes
                             // at the end, which hides where a sweep is losing devices)

  // Policy visibility. Without these a refused or failed hop is a bare number with no attribution:
  // one bench session logged hops=7 hopFailures=7 with no way to tell which device or why, which is
  // exactly what made it undiagnosable from outside the device.
  uint8_t     mode;          // coord_mode_t: 0 = onboard (hopping allowed), 1 = operate (forbidden)
  uint8_t     lastRefusal;   // coord_hop_verdict_t of the most recent refusal, 0 if none
  uint8_t     blacklisted;   // targets retired after a failed hop
  bool        blacklistFull; // COORD_MAX_BLACKLIST reached — a further failure cannot be retired
};
CoordinatorStatus coordinatorStatus();

// --- device registry ---------------------------------------------------------------------------
// What we know about each WLED device the sweep has seen. Populated from the sweep that already
// visits every address, so it costs one extra GET per device rather than any new traffic.
#ifndef COORD_MAX_DEVICES
#define COORD_MAX_DEVICES 32
#endif
struct CoordDevice {
  bool     used;
  uint8_t  host;            // last octet of the address on the home subnet
  char     mac[13];
  char     name[17];
  char     ver[12];
  uint16_t leds;            // leds.count
  bool     matrix;          // leds.matrix present => a 2D device (the 2.5D work needs this)
  // RSSI AS REPORTED BY THE DEVICE, which is the device's signal TO THE ACCESS POINT — not to this
  // coordinator. It estimates distance from the AP, so on its own it places devices on a sphere
  // around the router, not relative to each other. Recorded because it is free and genuinely useful
  // for that, and labelled so nobody later reads it as a device-to-device distance.
  int8_t   apRssi;
  uint8_t  apSignalPct;
  // Current look, so the registry answers "what is everything doing" without a second sweep.
  uint8_t  fx, pal, bri, r, g, b;
  bool     on;
  uint32_t lastSeenMs;
};
const CoordDevice *coordinatorDevices(uint8_t *countOut);
void coordinatorSetTarget(uint8_t r, uint8_t g, uint8_t b);
