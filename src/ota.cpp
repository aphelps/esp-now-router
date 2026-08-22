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
#include <mbedtls/md.h>
#include <mbedtls/sha256.h>
#include <esp_random.h>
#include "ota_auth.h"
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

// --- OTA sender authentication ------------------------------------------------------------------
// Enabled only when a secret is compiled in. With none, behaviour is exactly as before — this must
// not silently start rejecting uploads on a build that never configured it, because the result
// would be a device that can only be recovered with a cable.
#ifndef OTA_AUTH_SECRET
#define OTA_AUTH_SECRET ""
#endif
static OtaAuthNonce s_nonce;
static mbedtls_sha256_context s_imgSha;     // running hash of the image as it streams past
static bool s_imgShaOpen = false;
static char s_presentedSig[OTA_AUTH_SIG_HEX] = {0};
static bool s_authFailed = false;

static bool otaAuthEnabled() { return OTA_AUTH_SECRET[0] != '\0'; }

static void otaHmacHex(const char *msg, char *outHex) {
  uint8_t mac[OTA_AUTH_SIG_LEN];
  const mbedtls_md_info_t *info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
  mbedtls_md_hmac(info, (const uint8_t *)OTA_AUTH_SECRET, strlen(OTA_AUTH_SECRET),
                  (const uint8_t *)msg, strlen(msg), mac);
  ota_auth_hex(mac, sizeof(mac), outHex);
}

// Called after the upload completes: report result and reboot into the new slot on success.
static void handleUpdateResult() {
  if (s_authFailed) {
    s_authFailed = false;
    s_server->send(401, "text/plain", "REJECTED — bad or missing signature\n");
    return;
  }
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
      s_authFailed = false;
      if (otaAuthEnabled()) {
        // The signature travels as a header so it is available before the body — it has to be, since
        // we hash the body as it streams rather than buffering an image we have no RAM for.
        String sig = s_server->hasHeader("X-OTA-Signature") ? s_server->header("X-OTA-Signature") : String();
        strncpy(s_presentedSig, sig.c_str(), sizeof(s_presentedSig) - 1);
        s_presentedSig[sizeof(s_presentedSig) - 1] = 0;
        mbedtls_sha256_init(&s_imgSha);
        mbedtls_sha256_starts(&s_imgSha, 0);
        s_imgShaOpen = true;
      }
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { DEBUG_ERR("OTA: begin failed"); DEBUG1_COMMAND(Update.printError(Serial)); }
      break;
    case UPLOAD_FILE_WRITE:
      if (s_imgShaOpen) mbedtls_sha256_update(&s_imgSha, up.buf, up.currentSize);
      if (Update.write(up.buf, up.currentSize) != up.currentSize) { DEBUG_ERR("OTA: write failed"); DEBUG1_COMMAND(Update.printError(Serial)); }
      break;
    case UPLOAD_FILE_END:
      if (otaAuthEnabled()) {
        uint8_t digest[32]; char digestHex[65];
        mbedtls_sha256_finish(&s_imgSha, digest);
        mbedtls_sha256_free(&s_imgSha);
        s_imgShaOpen = false;
        ota_auth_hex(digest, sizeof(digest), digestHex);

        char msg[OTA_AUTH_NONCE_HEX + 1 + 65];
        char expect[OTA_AUTH_SIG_HEX];
        ota_auth_result_t r = OTA_AUTH_MALFORMED;
        if (ota_auth_message(s_nonce.hex, digestHex, msg, sizeof(msg)) > 0) {
          otaHmacHex(msg, expect);
          r = ota_auth_verify(&s_nonce, s_presentedSig, expect, millis());
        } else {
          ota_auth_nonce_clear(&s_nonce);
        }
        memset(s_presentedSig, 0, sizeof(s_presentedSig));
        if (r != OTA_AUTH_OK) {
          // Abort BEFORE Update.end(): ending would mark the image valid and the device would boot
          // an unauthenticated build on the next restart. This is the step that makes the whole
          // scheme worth anything.
          DEBUG1_VALUELN("OTA: REJECTED, auth result=", (int)r);
          Update.abort();
          s_authFailed = true;
          break;
        }
        DEBUG1_PRINTLN("OTA: signature ok");
      }
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
  // The signature header must be collected explicitly — WebServer discards headers it was not told
  // to keep, and it would arrive empty with no indication why.
  static const char *keep[] = { "X-OTA-Signature" };
  s.collectHeaders(keep, 1);
  s.on("/update", HTTP_POST, handleUpdateResult, handleUpdateUpload);

  // Issue a challenge. Single-use and short-lived; see ota_auth.h for why both matter.
  s.on("/auth/nonce", HTTP_GET, [](){
    if (!otaAuthEnabled()) {
      s_server->send(404, "application/json", "{\"error\":\"auth not configured in this build\"}");
      return;
    }
    uint8_t rnd[OTA_AUTH_NONCE_LEN];
    esp_fill_random(rnd, sizeof(rnd));      // hardware RNG, not the Arduino PRNG
    ota_auth_nonce_issue(&s_nonce, rnd, millis());
    String j = String("{\"nonce\":\"") + s_nonce.hex + "\",\"ttlMs\":" + String(OTA_AUTH_NONCE_TTL_MS) + "}";
    s_server->send(200, "application/json", j);
  });
}

bool otaHaveInfraCreds() {
  return ROUTER_WIFI_SSID[0] != '\0';
}

bool otaWifiUp() {
  return haveCreds && WiFi.status() == WL_CONNECTED;
}
