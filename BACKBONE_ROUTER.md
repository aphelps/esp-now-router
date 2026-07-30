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
- Routers are headless (no LED/UI). They **relay** frames, **elect a leader** that owns the shared
  timebase, and **accept attaches** from the nodes that depend on them. A router is
  `ss_router_should_relay` + a neighbor table + a beacon + the attach protocol + HTTP-OTA.

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

## Attach protocol

Relaying works without any node knowing who else exists, but self-healing does not: something has
to notice that a router went away. The attach protocol is that bookkeeping — a router advertises how
far it is from the leader, nodes bind to the router they rank best, and both sides track liveness.
Pure logic in `src/router_attach.h`, radio glue in `src/attach.{h,cpp}`, host-tested in
`tests/test_attach/test_attach.cpp`.

Three single-hop frames, all riding the same `AMPS` header (`ttl = 1`, never relayed — the relay
rule is now "only `SENSOR_SYNC_MSG_SNAPSHOT` travels multi-hop", so control frames cannot leak
across the mesh):

| msgType | Frame | Direction | Payload |
|---|---|---|---|
| `SENSOR_SYNC_MSG_ROUTER_ADV` | advertisement | router → all | `RouterAdvert{leaderId, leaderTerm, hopCost, memberCount}` |
| `SENSOR_SYNC_MSG_ATTACH` | attach / keepalive | node → router | `NodeAttach{routerId}` |
| `SENSOR_SYNC_MSG_ATTACH_ACK` | ack | router → node | `AttachAck{nodeId, leaseMs}` |

The payload structs live in the shared `sensor_sync_protocol.h` because a WLED edge parses them too;
the selection and membership logic lives here, since the edge never calls it.

### Choosing a router

**No RSSI.** WLED's `onEspNowMessage(sender, payload, len)` hook does not forward the per-frame RSSI
(`espNowReceiveCB` drops it before the hook), so an edge cannot rank routers by signal strength
without patching WLED core — divergence this fork re-applies on every upstream bump. Instead the
router advertises a **hop/cost metric** and every node selects on that plus freshness, which needs no
core change and is fully host-testable.

The metric, best first:

1. **`hopCost`** — hops to the timebase leader; the leader advertises 0, everyone else one more than
   the router it is bound to. This is what bounds delivery latency. Saturates at
   `SS_HOP_UNREACHABLE` (255), which also means "no path": such a router is never selected.
2. **`memberCount`** — how many nodes are already attached, so load spreads instead of piling onto
   whichever router is heard first.
3. **`deviceId`** — highest wins. Globally unique, so the order is total and every node picks the
   same winner.

A **hold-down** (`ROUTER_ROUTE_HOLDDOWN_MS`) follows each change: within it only a strictly *shorter*
path may preempt, so a genuine topology improvement lands immediately while a mere load or tiebreak
edge waits, and the choice cannot flap between two near-equal routers.

Selection is loop-free by construction. A node attached to us sits below us in the tree and is never
adopted as our upstream; a pair that adopted each other in the window before either attach landed is
detected (our upstream appears in our own member table) and broken apart.

### Membership

A router admits each attaching node into a table of `ROUTER_MAX_MEMBERS`, acks it, and holds the
membership for `ROUTER_MEMBER_LEASE_MS`. The node re-attaches every `ROUTER_ATTACH_MS` as a
keepalive; a member that goes quiet past its lease is released, which is what shrinks the mesh back
when a node disappears. Membership is liveness only — deliberately no sequence-number dedup, which
would strand a node that rebooted and restarted its sequence at 0. Both tables are visible at
`GET /routes` (`upstream` + `members`).

## Failover

Losing a router is handled by three independent timers, so no single missed frame disturbs anything:

1. **Route drop.** A node whose router has been silent for `ROUTER_ROUTE_TIMEOUT_MS` drops the route
   and is unattached again. There is no incumbent to hold down at that point, so the very next
   advertisement it hears is adopted — recovery costs one advertisement interval.
2. **Re-election.** The dead router's followers stop hearing its beacon and, after
   `ROUTER_LEADER_TIMEOUT_MS`, the best survivor asserts leadership with a bumped `term`. Equally
   long-lived survivors are separated by `deviceId`, so they converge on the same winner without
   negotiating.
3. **Membership expiry.** Routers below release the departed node's lease after
   `ROUTER_MEMBER_LEASE_MS`, dropping its load contribution back out of the metric.

While the tree re-roots, a router with no path advertises `SS_HOP_UNREACHABLE`, which pushes its
members to look elsewhere rather than sitting behind a dead end. **Relaying never stops during any of
this** — the flood is stateless apart from the dedup table, so sensor frames keep crossing the mesh
while the attach state converges. What the gap can cost is *state*, not delivery: an edge that missed
a touch transition during the churn is re-synced by the next `keyframeMs` full-snapshot broadcast
(default 3 s). Note the route timeout (`ROUTER_ROUTE_TIMEOUT_MS`, 3.5 s) is slightly *longer* than
that keyframe, so a node can miss one keyframe while its route is still timing out; the following
one re-syncs it. Tightening the timeout below `keyframeMs` would close that window, at the cost of
dropping routes on a single missed advert — worth measuring on hardware before choosing.

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
  (`router_relay.h`, `router_election.h`, `router_attach.h`) lives here, since the edge never calls
  it; the attach payload structs are the exception, because the edge parses those off the wire.
- Loop: `quickEspNow` RX → `ss_is_our_frame` → SPSC ring → beacon / attach-protocol demux →
  `ss_router_should_relay` → re-broadcast `ttl-1`, honoring `readyToSendData` backpressure. Beacon,
  advertisement and attach keepalive each on their own timer; neighbor and member tables with expiry.
- **HTTP OTA**: `Update.h` + a web server (`POST /update`) streaming into the inactive OTA slot,
  with `esp_ota` mark-valid, so the serial cable is only needed for the first flash.
- Target board WT32-ETH01 (WiFi carries ESP-NOW; the ethernet PHY is unused). Its RMII pins limit
  free GPIO and it has no USB, so the first flash uses a USB-TTL serial adapter.
