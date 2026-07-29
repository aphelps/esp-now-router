#pragma once
// HTTP OTA for the esp-now-router: WiFi bring-up + a POST /update handler
// (Update.h). HTTP, not espota/UDP, so it works from macOS and the FTDI cable is used exactly once.
// The WebServer itself is owned by main.cpp (top-level server config); this module just brings up
// WiFi and registers the /update route on it. Implementation in ota.cpp.
#include <WebServer.h>

void otaWifiBegin();                    // WiFi-STA (build-flag creds) or fallback SoftAP; + mark-valid
void otaRegisterUpdate(WebServer &s);   // register the POST /update route on the caller's server
bool otaWifiUp();                       // true once associated to infra WiFi (STA) — false in AP mode
