// Compile-only coverage for the WLED-coordinator headers on the TARGET toolchain.
//
// They are host-tested, which proves the logic but not that they build for xtensa: int widths,
// the absence of a host stdlib, and stricter warnings all differ. This TU includes each of them
// and pins the assumptions that would break silently across that gap.
//
// It emits no code — everything here is static_assert or a static inline never called — so it
// costs the router firmware nothing while keeping the headers honest at every build.
//
#include "wled_translate.h"
#include "coordinator_mode.h"
#include "coordinator_phase.h"
#include "hmtl_to_wled.h"
#include "wled_onboard_cfg.h"
#include "wled_candidate.h"
#include "coordinator_boot.h"

// Wire-facing sizes must not drift with the toolchain.
static_assert(sizeof(WledColour) == 3, "WledColour is three bytes");
static_assert(sizeof(WledMac) == 6, "a MAC is six bytes");

// The HMTL layout constants must agree with the offsets the parser reads.
static_assert(HMTL_HDR_LEN == 8, "HMTL header is 8 bytes");
static_assert(HMTL_OUTPUT_HDR_LEN == 2, "output header is 2 bytes");
static_assert(HMTL_RGB_MSG_LEN == 13, "an RGB OUTPUT frame is 13 bytes");

// The sentinels are the values HMTLWireFormat.h defines, not merely "some byte".
static_assert(HMTL_WIRE_ALL_OUTPUTS == 0xFE, "ALL_OUTPUTS sentinel");
static_assert(HMTL_WIRE_NO_OUTPUT == 0xFF, "NO_OUTPUT sentinel");
static_assert(HMTL_WIRE_START == 0xFC, "startcode");

// The settle window must still exceed one WLED retry interval after any edit to either constant —
// this is the invariant coord_phase_init() clamps, asserted here so a bad default is a BUILD
// failure rather than a silently short settle nobody notices.
static_assert(COORD_SETTLE_MS >= WLED_STA_RETRY_MS,
              "settle window must span at least one WLED STA retry interval");

// The declared body maximum must actually hold the worst case: the fixed skeleton plus three
// 3-digit channels plus a 3-digit brightness plus the NUL.
static_assert(WLED_TRANSLATE_MAX_BODY >= 44 + 3 + 3 + 3 + 3 + 1,
              "WLED_TRANSLATE_MAX_BODY covers the worst-case body");

// The setup-AP facts the matcher depends on. If WLED ever changes them, this fails at BUILD time
// rather than as a scan that silently matches nothing.
static_assert(WLED_DEFAULT_AP_CHANNEL == 6, "apChannel default is 6 (wled.h:368), not 1");
static_assert(sizeof(WLED_SETUP_AP_PASS) == 9, "\"wled1234\" plus NUL — the AP is NOT open");

// The reboot-loop brake must actually terminate. A threshold of 0 or 1 would blacklist on the very
// first fault (or immediately), and anything that cannot reach the ceiling never breaks the loop.
static_assert(COORD_MAX_ATTEMPTS >= 1, "at least one attempt before blacklisting");
static_assert(COORD_MAX_ATTEMPTS < 0xFF, "the threshold must be reachable by a uint8_t counter");
