# AMPWorks Mesh — Backbone Router design

How the dedicated **non-WLED ESP32 "router" tier** relays SensorSync frames multi-hop, so an event
injected at one edge reaches every edge even when the edges cannot hear each other directly.
Companion to `SENSOR_SYNC.md` (the edge protocol).

## Where the router sits

```
edge ──ESP-NOW─┐                             ┌─ESP-NOW── edge
edge ──ESP-NOW─┤  router ──ESP-NOW── router  ├─ESP-NOW── edge
edge ──ESP-NOW─┘   (relay + leader)          └─ESP-NOW── edge
```

- Edges are WLED nodes running the SensorSync usermod. They broadcast `AMPS` frames on the shared
  ESP-NOW channel, stamping `ttl = SS_DEFAULT_TTL`.
- Routers are headless (no LED/UI). They **relay** frames and **elect a leader** that owns the
  shared timebase. A router is `ss_router_should_relay` + a neighbor table + a beacon + HTTP-OTA.

## Relay

Loop-free flood with two independent stoppers:

1. **Per-origin seq dedup** (primary loop terminator): each router keeps a `(deviceId → lastSeq)`
   table (`SensorRouterPeer`). A frame whose `seq` is not newer than the last seen from that origin
   (RFC-1982 `ss_seq_newer`) is a loop echo / reorder → dropped, not re-broadcast.
2. **TTL** (diameter backstop, for when a dedup slot was evicted): the origin stamps
   `SS_DEFAULT_TTL`; each relay re-broadcasts with `ttl-1` and drops at `ttl ≤ 1`.

A frame arriving with `ttl == 0` (a sender predating the field) has `SS_DEFAULT_TTL` injected so it
still propagates, and a node never relays a frame bearing its **own** `deviceId` (self-echo skip,
which lets an edge double as a relay). Correctness is covered by `tests/test_relay/test_relay.cpp`
(triangle loop terminates, line TTL cutoff, bridged out-of-range edges delivered).

## Leader election

One router is the **timebase leader**, so animations stay phase-aligned across the installation.
The election is deterministic, degenerates cleanly to **N = 1**, and self-heals when the leader
drops. Implemented as a pure state machine in `src/router_election.h`, host-tested in
`tests/test_election/test_election.cpp`.

Bully-style and beacon-driven, so it costs no extra round-trips:

- Every router periodically broadcasts a **beacon** — an `AMPS` frame with
  `msgType = SENSOR_SYNC_MSG_BEACON` carrying `{uptimeTicks, term}` (the origin `deviceId` is in the
  header). Beacons are single-hop: they carry `ttl = 1` and routers never relay them.
- **Election metric**, highest wins: `(uptimeTicks, deviceId)` — the longest-lived router leads, with
  `deviceId` as the tiebreak (a globally unique 32-bit MAC hash, so never an exact tie).
  Uptime-first keeps a just-rebooted node from taking over.
- A router asserts leadership only when it has heard **no better** beacon within
  `ROUTER_LEADER_TIMEOUT_MS` (a small multiple of the beacon interval). Hearing a better beacon
  makes it **step down**. Because that timeout also gates the first assertion after boot, a router
  starts as a follower rather than briefly claiming leadership.
- It tracks the **best** leader heard rather than the most recent, and rejects stale (older-`term`)
  beacons from that leader, so a delayed duplicate cannot keep a dead leader alive.
- **N = 1**: a lone router hears no beacons, so it becomes leader through the same timeout path —
  no special case.
- **Failover**: when the leader goes silent, every remaining router re-asserts on its own metric
  after the timeout and the best survivor wins, incrementing `term`. Split-brain during the gap is
  benign for a timebase — frames still relay, and the edge `keyframeMs` resync covers the phase gap.

The scheme is deliberately lighter than a beat-master handshake: a timebase owner only needs a
monotonic metric plus step-down, not negotiation round-trips.

## Channel coexistence

ESP-NOW rides the WiFi radio's current channel, which constrains deployment:

- **Channel lock with STA up:** a router associated to an AP sits on that AP's channel, so every
  node must share it (`ROUTER_ESPNOW_CHANNEL` must match the edges'). If the AP roams channels,
  ESP-NOW breaks — pin the routers and edges to a fixed channel independent of the association.
- **Latency budget:** origin→delivery is measured via the header `timestamp`; the target is
  p95 < 50 ms per hop.
- **Association under load:** the router keeps STA associated (for OTA and the channel) while
  relaying.

## Firmware shape

- **Own repo `esp-now-router`, a submodule of `WLED_dev`** (sibling to `WLED`/`ArduinoLibs`;
  `setup.sh` inits it). Non-WLED, no LED/UI.
- Shares `sensor_sync_protocol.h` with the edge as a **single-source include** (not a copy): the
  header is dependency-free and lives in the `WLED` submodule, and the router's `platformio.ini`
  references it from the sibling submodule path, so the wire format cannot drift. Router-only logic
  (`router_relay.h`) lives here, since the edge never calls it.
- Loop: `quickEspNow` RX → `ss_is_our_frame` → SPSC ring → `ss_router_should_relay` → re-broadcast
  `ttl-1`, honoring `readyToSendData` backpressure. Beacon on a timer; neighbor table with expiry.
- **HTTP OTA**: `Update.h` + a web server (`POST /update`) streaming into the inactive OTA slot,
  with `esp_ota` mark-valid, so the serial cable is only needed for the first flash.
- Target board WT32-ETH01 (WiFi carries ESP-NOW; the ethernet PHY is unused). Its RMII pins limit
  free GPIO and it has no USB, so the first flash uses a USB-TTL serial adapter.
