// attach.cpp — attach-protocol transport glue. See attach.h for the module's role; the decisions
// it makes are all in the pure, host-tested router_attach.h.
//
// Three single-hop control frames ride the same AMPS header as sensor snapshots:
//   ROUTER_ADV  — we broadcast our distance to the timebase leader and our current load.
//   ATTACH      — we announce ourselves to the router we picked, and repeat it as a keepalive.
//   ATTACH_ACK  — we answer an attach we accepted, granting the sender a membership lease.
//
#include <QuickEspNow.h>
#include <Debug.h>

#include "attach.h"

static const uint8_t ATTACH_BCAST_ADDR[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static uint32_t     selfId = 0;
static AttachedNode members[ROUTER_MAX_MEMBERS];
static RouteChoice  route = ra_route_init();

// Send-failure telemetry. See sendControl() for why these are not behind ROUTER_CHANNEL_DIAG.
static uint32_t espnowSendFails  = 0;
static int      espnowLastSendRc = 0;

// Per-frame-type sequence counters. Each control stream numbers itself independently so a receiver
// can spot gaps in one without the others perturbing it.
static uint16_t advertSeq = 0;
static uint16_t attachSeq = 0;
static uint16_t ackSeq    = 0;

static uint32_t lastAdvertMs = 0;
static uint32_t lastAttachMs = 0;

// The lease our router quoted in its ack (0 until one arrives). Kept so the keepalive cadence can
// follow the router rather than assume it was built with the same constants we were.
static uint32_t routerLeaseMs = 0;

// How often to re-send the attach keepalive. Normally ROUTER_ATTACH_MS, but never slower than a
// third of the lease our router actually quoted — otherwise a router built with a shorter lease
// would expire us between keepalives and membership would flap with nothing in the log to explain
// it. Gated on `attached`, so a lease is only ever applied to the router that quoted it: adopting
// or dropping a route clears that flag, which stops a previous router's lease from setting the
// cadence for the next one. Implausibly short leases are rejected on arrival (see the ack handler),
// so the division here always leaves at least ROUTER_ATTACH_MIN_MS.
static uint32_t attachIntervalMs() {
  if (!route.attached || routerLeaseMs == 0) return ROUTER_ATTACH_MS;

  const uint32_t fromLease = routerLeaseMs / 3;
  return fromLease < ROUTER_ATTACH_MS ? fromLease : ROUTER_ATTACH_MS;
}

// Broadcast one control frame carrying `payload`. Returns whether the radio accepted it; a refusal
// is not retried here, because every control frame is periodic and the next tick re-sends it.
static bool sendControl(uint8_t msgType, uint16_t seq, const void *payload, uint8_t payloadLen,
                        uint32_t now) {
  uint8_t buf[sizeof(SensorSyncHeader) + 16];
  if (payloadLen > sizeof(buf) - sizeof(SensorSyncHeader)) return false;

  SensorSyncHeader h{};
  ra_stamp_control_header(h, selfId, msgType, payloadLen, seq, now);
  memcpy(buf, &h, sizeof(h));
  memcpy(buf + sizeof(h), payload, payloadLen);

  // Count failures UNCONDITIONALLY. This bug was invisible for exactly one reason: nothing looked
  // at the return value. A diag-only log leaves it invisible on every deployed build — i.e. every
  // build where it matters. Surfaced in /debug so "everything is failing" is one HTTP GET away.
  const int rc = quickEspNow.send(ATTACH_BCAST_ADDR, buf, sizeof(h) + payloadLen);
  if (rc != 0) { espnowSendFails++; espnowLastSendRc = rc; }
  return rc == 0;
}

// Advertise our distance to the timebase leader plus our current load, so the nodes around us can
// rank us against the other routers they hear.
static void sendAdvert(const RouterElection &election, uint32_t now) {
  RouterAdvert a{};
  a.leaderId    = election.leaderId;
  a.leaderTerm  = election.isLeader ? election.term : election.leaderTerm;
  a.hopCost     = ra_advert_cost(election.isLeader, route);
  a.memberCount = ra_member_count(members, ROUTER_MAX_MEMBERS);

  if (!sendControl(SENSOR_SYNC_MSG_ROUTER_ADV, advertSeq++, &a, sizeof(a), now))
    DEBUG_ERR("attach: advert send failed");
}

// Announce ourselves to the router we are bound to. Repeated as a keepalive, since that is what
// holds our membership lease open on the other side.
static void sendAttach(uint32_t now) {
  NodeAttach n{};
  n.routerId = route.routerId;
  lastAttachMs = now;

  if (!sendControl(SENSOR_SYNC_MSG_ATTACH, attachSeq++, &n, sizeof(n), now))
    DEBUG_ERR("attach: attach send failed");
}

// Accept `nodeId` as a member and tell it so, quoting the lease it now holds.
static void sendAttachAck(uint32_t nodeId, uint32_t now) {
  AttachAck ack{};
  ack.nodeId  = nodeId;
  ack.leaseMs = ROUTER_MEMBER_LEASE_MS;

  if (!sendControl(SENSOR_SYNC_MSG_ATTACH_ACK, ackSeq++, &ack, sizeof(ack), now))
    DEBUG_ERR("attach: ack send failed");
}

// Bind this module to the node's deviceId (see attach.h).
void attachBegin(uint32_t deviceId) {
  selfId = deviceId;
  memset(members, 0, sizeof(members));
  route = ra_route_init();
}

// Offer a received frame to the attach protocol (see attach.h).
bool attachHandleFrame(const SensorSyncHeader &h, const uint8_t *data, int dataLen, uint32_t now) {
  if (h.msgType != SENSOR_SYNC_MSG_ROUTER_ADV && h.msgType != SENSOR_SYNC_MSG_ATTACH &&
      h.msgType != SENSOR_SYNC_MSG_ATTACH_ACK)
    return false;

  // Our own broadcast echoing back is consumed and ignored — a node never attaches to itself.
  if (h.deviceId == selfId) return true;

  if (h.msgType == SENSOR_SYNC_MSG_ROUTER_ADV) {
    if (dataLen < (int)sizeof(RouterAdvert)) return true;
    RouterAdvert a;
    memcpy(&a, data, sizeof(a));

    // A changed route means a new (or no) upstream router: attach to it immediately rather than
    // waiting out the keepalive interval, so the gap in coverage stays short.
    const bool downstream = ra_member_attached(members, ROUTER_MAX_MEMBERS, h.deviceId);
    if (ra_on_advert(route, h.deviceId, a, downstream, now, ROUTER_ROUTE_HOLDDOWN_MS)) {
      if (ra_route_held(route)) {
        DEBUG1_HEXVAL("attach: bound to router 0x", route.routerId);
        DEBUG1_VALUELN(" hops=", (unsigned)route.hopCost);
        sendAttach(now);
      } else {
        DEBUG1_PRINTLN("attach: upstream router lost its path, route dropped");
      }
    }
    return true;
  }

  if (h.msgType == SENSOR_SYNC_MSG_ATTACH) {
    if (dataLen < (int)sizeof(NodeAttach)) return true;
    NodeAttach n;
    memcpy(&n, data, sizeof(n));

    // An attach must name us. Every attach this firmware sends carries the router it is bound to
    // (sendAttach is only reached with a route held), so there is no "any router" form to honour —
    // accepting one would admit the node at every router in earshot at once.
    if (n.routerId != selfId) return true;

    if (ra_member_attach(members, ROUTER_MAX_MEMBERS, h.deviceId, now)) {
      sendAttachAck(h.deviceId, now);
    } else {
      DEBUG_ERR("attach: membership table full, attach not acked");
    }
    return true;
  }

  if (dataLen < (int)sizeof(AttachAck)) return true;
  AttachAck ack;
  memcpy(&ack, data, sizeof(ack));
  if (ra_on_ack(route, h.deviceId, ack, selfId, now)) {
    // Adopt the router's lease as our keepalive base, but only when three keepalives actually fit
    // inside it. A shorter lease cannot be honoured — the cadence needed to hold it would be faster
    // than ROUTER_ATTACH_MIN_MS — so treat it as misconfiguration, say so, and keep our own cadence
    // rather than silently adopting an interval longer than the lease it is meant to satisfy.
    if (ack.leaseMs < 3 * ROUTER_ATTACH_MIN_MS) {
      routerLeaseMs = 0;
      DEBUG_ERR("attach: router quoted an implausibly short lease; keeping our own keepalive rate");
    } else {
      routerLeaseMs = ack.leaseMs;
    }

    DEBUG1_HEXVALLN("attach: membership confirmed by router 0x", route.routerId);
  }

  return true;
}

// Drive the protocol's timers (see attach.h).
void attachTick(const RouterElection &election, uint32_t now) {
  uint8_t dropped = ra_member_expire(members, ROUTER_MAX_MEMBERS, now, ROUTER_MEMBER_LEASE_MS);
  if (dropped) DEBUG1_VALUELN("attach: members released on lease expiry=", (unsigned)dropped);

  // The leader is the root of the tree, so it holds no upstream route of its own.
  if (election.isLeader) {
    if (ra_route_held(route)) ra_route_clear(route);
  } else if (ra_route_tick(route, now, ROUTER_ROUTE_TIMEOUT_MS)) {
    DEBUG1_PRINTLN("attach: upstream router timed out, route dropped");
  } else if (ra_route_break_loop(route, members, ROUTER_MAX_MEMBERS)) {
    DEBUG1_PRINTLN("attach: upstream router is also our member, loop broken");
  }

  if ((uint32_t)(now - lastAdvertMs) >= ROUTER_ADVERT_MS) {
    lastAdvertMs = now;
    sendAdvert(election, now);
  }

  if (!election.isLeader && ra_route_held(route) &&
      (uint32_t)(now - lastAttachMs) >= attachIntervalMs())
    sendAttach(now);
}

// The upstream route as a JSON object (see attach.h).
String attachRouteJson() {
  uint32_t now = millis();
  String j = "{";
  j += "\"routerId\":\"" + String(route.routerId, HEX) + "\",";
  j += "\"leaderId\":\"" + String(route.leaderId, HEX) + "\",";
  j += "\"hopCost\":" + String((unsigned)route.hopCost) + ",";
  j += "\"attached\":" + String(route.attached ? "true" : "false") + ",";
  j += "\"ageMs\":" + String(ra_route_held(route) ? now - route.lastHeardMs : 0);
  j += "}";
  return j;
}

// The attached-member table as a JSON array (see attach.h).
String attachMembersJson() {
  uint32_t now = millis();
  String j = "[";
  bool first = true;
  for (uint8_t i = 0; i < ROUTER_MAX_MEMBERS; i++) {
    if (!members[i].used) continue;
    if (!first) j += ",";
    first = false;
    j += "{\"id\":\"" + String(members[i].deviceId, HEX) +
         "\",\"ageMs\":" + String(now - members[i].lastSeenMs) + "}";
  }
  j += "]";
  return j;
}

// How many nodes are attached to us (see attach.h).
uint8_t attachMemberCount() {
  return ra_member_count(members, ROUTER_MAX_MEMBERS);
}

uint32_t attachSendFails()  { return espnowSendFails; }
int      attachLastSendRc() { return espnowLastSendRc; }
