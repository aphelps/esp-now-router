// HTTP OTA for the esp-now-router — see ota.h.
//
// Mirrors WLED's curl-friendly flow (`POST /update` with the firmware binary), so the same
// tooling works: `curl -F "file=@firmware.bin" http://<router-ip>/update`. Streams the upload
// straight into the ESP32 Update partition (the inactive OTA slot), so a failed/aborted flash
// never touches the running image — that's the anti-brick guard. On success it reboots into the
// new slot and marks it valid.
//
// WiFi creds are build flags (ROUTER_WIFI_SSID / ROUTER_WIFI_PASS) — set them in a gitignored
// platformio_override.ini for a real install. With no SSID the router still relays (ESP-NOW is
// channel-based) but the OTA endpoint is unreachable until creds are provided.
//
// Coexistence: once associated, the radio locks to the AP's channel — the edges must share that
// AP/channel for ESP-NOW to reach them. BACKBONE_ROUTER.md covers the ordering of
// quickEspNow.begin() vs association.
#include "ota.h"

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include <esp_ota_ops.h>
#include <Debug.h>

#ifndef ROUTER_WIFI_SSID
#define ROUTER_WIFI_SSID ""
#endif
#ifndef ROUTER_WIFI_PASS
#define ROUTER_WIFI_PASS ""
#endif
// Fallback SoftAP (used when no infra-WiFi creds are set) so the OTA endpoint is ALWAYS reachable
// with a trivially-rememberable name: join "esp-now-router" / "meshrouter" and browse
// http://192.168.4.1/update. Overridable per install.
#ifndef ROUTER_AP_SSID
#define ROUTER_AP_SSID "esp-now-router"
#endif
#ifndef ROUTER_AP_PASS
#define ROUTER_AP_PASS "meshrouter"     // >=8 chars for WPA2
#endif
#ifndef ROUTER_ESPNOW_CHANNEL
#define ROUTER_ESPNOW_CHANNEL 1
#endif

static WebServer *s_server = nullptr;   // owned by main.cpp; set in otaRegisterUpdate()
static bool       haveCreds = false;

// Called after the upload completes: report result and reboot into the new slot on success.
static void handleUpdateResult() {
  bool ok = !Update.hasError();
  s_server->send(ok ? 200 : 500, "text/plain", ok ? "OK — rebooting into new image\n" : "UPDATE FAILED\n");
  if (ok) { delay(200); ESP.restart(); }
}

// Streams each chunk of the multipart upload into the inactive OTA partition.
static void handleUpdateUpload() {
  HTTPUpload &up = s_server->upload();
  switch (up.status) {
    case UPLOAD_FILE_START:
      DEBUG1_VALUELN("OTA: start ", up.filename);
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { DEBUG_ERR("OTA: begin failed"); DEBUG1_COMMAND(Update.printError(Serial)); }
      break;
    case UPLOAD_FILE_WRITE:
      if (Update.write(up.buf, up.currentSize) != up.currentSize) { DEBUG_ERR("OTA: write failed"); DEBUG1_COMMAND(Update.printError(Serial)); }
      break;
    case UPLOAD_FILE_END:
      if (Update.end(true)) { DEBUG1_VALUELN("OTA: wrote bytes=", (unsigned)up.totalSize); }
      else                  { DEBUG_ERR("OTA: end failed"); DEBUG1_COMMAND(Update.printError(Serial)); }
      break;
    default:
      break;
  }
}

// Bring up WiFi (infra STA if creds are set, else the fallback SoftAP) and confirm the running
// OTA image. Does NOT touch the web server — main.cpp owns it (see otaRegisterUpdate).
void otaWifiBegin() {
  const char *ssid = ROUTER_WIFI_SSID;
  haveCreds = (ssid[0] != '\0');
  if (haveCreds) {
    WiFi.begin(ROUTER_WIFI_SSID, ROUTER_WIFI_PASS);   // join infra WiFi (non-blocking; relay runs meanwhile)
  } else {
    // No infra creds: stand up the fallback SoftAP on the ESP-NOW channel so OTA is always
    // reachable and the mesh keeps sharing the channel. See BACKBONE_ROUTER.md for the
    // STA+AP vs quickEspNow ordering caveat.
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(ROUTER_AP_SSID, ROUTER_AP_PASS, ROUTER_ESPNOW_CHANNEL);
  }

  // If we booted from an OTA slot pending verification, confirm it now that we've come up cleanly
  // (cancels a bootloader rollback where enabled; harmless otherwise).
  esp_ota_mark_app_valid_cancel_rollback();
}

// Register the POST /update route on the caller-owned server (main.cpp owns top-level config).
void otaRegisterUpdate(WebServer &s) {
  s_server = &s;
  s.on("/update", HTTP_POST, handleUpdateResult, handleUpdateUpload);
}

bool otaWifiUp() {
  return haveCreds && WiFi.status() == WL_CONNECTED;
}
