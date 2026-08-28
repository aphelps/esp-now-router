#pragma once
// When may ESP-NOW be started? Split out of main.cpp so the decision is host-testable — the radio
// call is not testable off-device, but the decision is not radio code, and it is the part that has
// now been got wrong twice. Same split router_election.h / router_attach.h already use.
#include <stdbool.h>

// `begin()` resolves CURRENT_WIFI_CHANNEL ONCE and pins it, so with infra credentials it must not
// run until the STA has associated — otherwise it pins the boot channel and every send fails.
//
// There is deliberately NO "give up and pin a channel anyway" fallback, for the reason recorded in
// main.cpp's setup(): pinning a channel while infra credentials are set "does not merely degrade the
// link, it stops the STA associating at all" (bench 2026-08-22: WL_DISCONNECTED with ip 0.0.0.0
// forever, while WiFi.channel() correctly reported the AP's channel). A router with credentials and
// no AP in range therefore stays MUTE and keeps retrying, rather than pinning itself into a state it
// cannot leave. Mute is recoverable and keeps HTTP/OTA alive; pinned is not.
//
// An attempt to re-confirm that on 2026-08-28 was INCONCLUSIVE and should not be cited as evidence:
// the AP was down that day, so the STA reported WL_NO_SSID_AVAIL (SSID not found) rather than the
// WL_DISCONNECTED (found, could not associate) the 2026-08-22 run saw. Different failure, different
// meaning. The 2026-08-22 finding stands on its own; this note exists so the two are not conflated.
static inline bool espnow_should_start(bool haveInfraCreds, bool connected, bool alreadyStarted) {
  if (alreadyStarted) return false;   // begin() is not re-entrant: initComms() would leak its queues
  if (!haveInfraCreds) return true;   // standalone: nothing else owns the channel
  return connected;                   // infra: only once the AP's channel is knowable
}
