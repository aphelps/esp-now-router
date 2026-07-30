# esp-now-router

Headless **ESP-NOW backbone router** for the AMPWorks sensor-sync mesh.

Dedicated non-WLED ESP32 nodes that **relay** SensorSync (`AMPS`) frames multi-hop, so edge
nodes that can't hear each other directly still receive every event. Loop-free flood via the
shared, host-tested `ss_router_should_relay()` (per-origin `seq` dedup + TTL). No LEDs, no UI.

Routers also run an **attach protocol**: each advertises its distance to the elected timebase
leader, nodes bind to the router they rank best, and heartbeat timeouts drop dead routes so the
mesh re-forms on its own when a router goes away.

## Layout in the mesh

```
edge ──ESP-NOW─┐                             ┌─ESP-NOW── edge
edge ──ESP-NOW─┤  router ──ESP-NOW── router  ├─ESP-NOW── edge   (this repo = the router tier)
edge ──ESP-NOW─┘  (relay + leader + attach)  └─ESP-NOW── edge
```

## Shared wire format

The router uses the **same** `sensor_sync_protocol.h` / `sensor_sync_ring.h` as the WLED edge
firmware, included directly from the sibling `WLED` submodule (`-I ../WLED/usermods/ampworks` in
`platformio.ini`) — single source of truth so the format can't drift. This repo is a **submodule
of [WLED_dev](https://github.com/aphelps/WLED_dev)**; build it from inside that super-repo so the
sibling `WLED` checkout is present.

## Build

```bash
# from inside WLED_dev (WLED submodule must be checked out):
cd esp-now-router
pio run -e wt32-eth01     # target board (LAN8720 ethernet; ESP-NOW rides WiFi)
pio run -e esp32dev       # any ESP32 devkit, for bench-testing the relay logic
```

First flash is over a USB-TTL serial adapter (WT32-ETH01 has no USB; hold IO0 low for the
bootloader); every update after that goes over HTTP-OTA, so the cable is needed only once.

## OTA

Set infra-WiFi creds in a gitignored `platformio_override.ini` (see
`platformio_override.sample.ini`), flash once over serial, then update wirelessly:

```bash
curl -F "file=@.pio/build/wt32-eth01/firmware.bin" http://<router-ip>/update
```

HTTP `POST /update` streams into the inactive OTA slot (a failed flash never touches the running
image), then reboots into the new slot. HTTP, not UDP/espota, so it works from macOS.

With **no** infra-WiFi creds set, the router stands up a fallback SoftAP — join **`esp-now-router`**
(password `meshrouter`) and browse `http://192.168.4.1/update` — so OTA is always reachable.

## Tests

Host unit tests for the router-used logic (`ss_router_*` relay, leader election, attach protocol)
live in `tests/` (pure logic, run on your machine — no device). Run from **inside `WLED_dev`** so
the sibling `WLED` wire header resolves:

```bash
pio test -e native        # idiomatic — runs every suite on the host
make -C tests coverage    # gcov line coverage (pio's custom runner doesn't do coverage)
```

(`make -C tests test` also works as a plain-`c++` alternative.)

## Design

`BACKBONE_ROUTER.md` — relay, leader-election, attach and failover design, plus the
fixed-channel go/no-go.
