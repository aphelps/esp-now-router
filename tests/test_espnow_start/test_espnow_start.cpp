// Host tests for the ESP-NOW start decision (espnow_start.h).
#include "espnow_start.h"
#include <stdio.h>

static int failures = 0;
static void check(bool cond, const char *what) {
  if (!cond) { printf("FAIL: %s\n", what); failures++; }
}

int main() {
  // Standalone owns its own channel; nothing to wait for.
  check(espnow_should_start(/*creds=*/false, /*connected=*/false, /*started=*/false),
        "standalone starts immediately");

  // THE BUG: with creds, starting before association pins the boot channel and loses 100% of sends.
  check(!espnow_should_start(true, false, false),
        "infra does NOT start before the association lands");
  check(espnow_should_start(true, true, false),
        "infra starts once associated");

  // begin() is not re-entrant: initComms() creates fresh queues and tasks without freeing the old.
  check(!espnow_should_start(true, true, /*started=*/true), "never starts twice (infra)");
  check(!espnow_should_start(false, false, /*started=*/true), "never starts twice (standalone)");

  // No grace fallback: an infra router with no AP stays mute forever rather than pinning a channel,
  // because pinning blocks the association permanently (measured twice).
  check(!espnow_should_start(true, false, false),
        "no fallback pin when the AP never appears");

  if (failures) { printf("%d test(s) failed\n", failures); return 1; }
  printf("ALL TESTS PASSED\n");
  return 0;
}
