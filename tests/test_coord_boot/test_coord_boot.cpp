// Host unit test for coordinator_boot.h — the reboot-loop brake.
#include "../../src/coordinator_boot.h"
#include <cstdio>

static int g_fail = 0;
#define CHECK(c,m) do { if(!(c)){ printf("FAIL: %s\n", m); g_fail=1; } } while(0)

int main() {
  const uint64_t T = 0xAABBCCDDEE01ULL;

  // --- Nothing in flight: a boot is just a boot ---------------------------------------------------
  { coord_boot_input_t in = { COORD_RESET_FAULT, false, T, 0 };
    CHECK(coord_boot_decide(in) == COORD_BOOT_RESUME, "fault with no hop in flight -> resume");
    CHECK(coord_boot_next_attempts(in, COORD_BOOT_RESUME) == 0, "record cleared"); }

  // --- A DELIBERATE restart mid-hop must not count against the target -----------------------------
  // Otherwise power-cycling the box while it is onboarding blacklists an innocent device.
  { coord_boot_input_t in = { COORD_RESET_NORMAL, true, T, 0 };
    CHECK(coord_boot_decide(in) == COORD_BOOT_RESUME, "clean restart mid-hop does not accuse the target");
    CHECK(coord_boot_next_attempts(in, COORD_BOOT_RESUME) == 0, "and does not consume an attempt"); }

  // --- A fault mid-hop consumes an attempt, then blacklists ----------------------------------------
  { coord_boot_input_t in = { COORD_RESET_FAULT, true, T, 0 };
    coord_boot_action_t a = coord_boot_decide(in);
    CHECK(a == COORD_BOOT_RETRY, "first fault: retry");
    CHECK(coord_boot_next_attempts(in, a) == 1, "attempt recorded");

    coord_boot_input_t in2 = { COORD_RESET_FAULT, true, T, 1 };
    CHECK(coord_boot_decide(in2) == COORD_BOOT_BLACKLIST, "second fault: blacklist, loop broken"); }

  // --- The loop terminates for ANY threshold, not just the default ---------------------------------
  { for (uint8_t start = 0; start < COORD_MAX_ATTEMPTS + 3; start++) {
      coord_boot_input_t in = { COORD_RESET_FAULT, true, T, start };
      coord_boot_action_t a = coord_boot_decide(in);
      if (start + 1 >= COORD_MAX_ATTEMPTS)
        CHECK(a == COORD_BOOT_BLACKLIST, "past the threshold always blacklists");
    } }

  // --- The counter saturates rather than wrapping to 0 and re-arming the loop ----------------------
  { coord_boot_input_t in = { COORD_RESET_FAULT, true, T, 0xFF };
    CHECK(coord_boot_decide(in) == COORD_BOOT_BLACKLIST, "saturated count still blacklists");
    CHECK(coord_boot_next_attempts(in, COORD_BOOT_BLACKLIST) == 0xFF, "saturates, does not wrap"); }

  printf(g_fail ? "TESTS FAILED\n" : "ALL TESTS PASSED\n");
  return g_fail;
}
