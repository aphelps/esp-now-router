#pragma once
//
// attach.h — the Arduino side of the attach protocol: the frames on the radio and the timers that
// drive them. The decision logic it calls is pure and lives in router_attach.h, so everything here
// is transport glue — building AMPS control frames, handing received ones to that logic, and
// exposing the resulting state as JSON for the HTTP endpoints.
//
// The router plays both roles. It advertises its own distance to the timebase leader and accepts
// attaches from the nodes below it, and it attaches upward to whichever router it ranks best, which
// is how a multi-router backbone builds a tree rooted at the leader. Implementation in attach.cpp.
//
#include <Arduino.h>

#include "sensor_sync_protocol.h"   // SensorSyncHeader + the attach payload structs
#include "router_attach.h"          // RouteChoice, AttachedNode (the pure logic this drives)
#include "router_election.h"        // RouterElection — supplies the leader identity we advertise

#ifndef ROUTER_ADVERT_MS
#define ROUTER_ADVERT_MS 1000        // how often we advertise our routing metric
#endif
#ifndef ROUTER_ATTACH_MS
#define ROUTER_ATTACH_MS 2000        // how often we re-send the attach keepalive to our router
#endif
#ifndef ROUTER_ATTACH_MIN_MS
#define ROUTER_ATTACH_MIN_MS 250     // floor on the keepalive cadence derived from a router's lease
#endif
#ifndef ROUTER_MEMBER_LEASE_MS
#define ROUTER_MEMBER_LEASE_MS 6000  // a member with no attach within this is released (3 keepalives)
#endif
#ifndef ROUTER_ROUTE_TIMEOUT_MS
#define ROUTER_ROUTE_TIMEOUT_MS 3500 // our router going silent this long drops the route (~3.5 adverts)
#endif
#ifndef ROUTER_ROUTE_HOLDDOWN_MS
#define ROUTER_ROUTE_HOLDDOWN_MS 5000 // after a switch, only a shorter path may preempt for this long
#endif
#ifndef ROUTER_MAX_MEMBERS
#define ROUTER_MAX_MEMBERS 32        // attached nodes tracked at once (edges + downstream routers)
#endif

// Bind this module to the node's deviceId. Call once from setup(), after the id is derived.
void attachBegin(uint32_t deviceId);

// Offer a received frame to the attach protocol. Returns true if it was an attach-protocol frame
// and has been consumed, false if the caller should go on handling it (a snapshot to relay).
bool attachHandleFrame(const SensorSyncHeader &h, const uint8_t *data, int dataLen, uint32_t now);

// Drive the protocol's timers: release expired members, drop a silent upstream router, then
// advertise our metric and refresh our own attach. Safe to call every loop; the sends are gated by
// their own intervals.
void attachTick(const RouterElection &election, uint32_t now);

// The upstream route as a JSON object, for GET /routes.
String attachRouteJson();

// The attached-member table as a JSON array, for GET /routes.
String attachMembersJson();

// How many nodes are attached to us — also the load term we advertise.
uint8_t attachMemberCount();

// ESP-NOW send failures on the attach/advert path, and the last non-zero rc (QuickEspNow
// comms_send_error_t; -3 = queue full, which is what a wrong-channel radio produces). Always
// compiled: a silent total-failure mode is the thing these exist to expose.
uint32_t attachSendFails();
int      attachLastSendRc();
