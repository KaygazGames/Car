#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>

// ==========================
// RC Car Controller (ESP32)
// ==========================
// Hardware:
// - ESP32
// - 1x analog joystick (VRx, VRy, SW)
// - 1x extra E-STOP button
//
// Behavior:
// - On boot enables controller mode on car
// - Sends periodic /api/controller/input payload
// - Joystick center is hard-coded
// - Joy SW button => horn
// - E-STOP button => estop true in payload

// -------- Car network --------
const char* CAR_AP_SSID = "Car-Control";
const char* CAR_AP_PASS = "car12345";
const char* CAR_BASE_URL = "http://car.local"; // fallback below if mDNS fails
const char* CAR_BASE_URL_FALLBACK = "http://192.168.4.1";

// -------- Joystick pins --------
const int JOY_X_PIN = 34;    // ADC input
const int JOY_Y_PIN = 35;    // ADC input
const int JOY_SW_PIN = 25;   // button (active low with INPUT_PULLUP)
const int ESTOP_PIN = 26;    // button (active low with INPUT_PULLUP)

// -------- Hard-coded center --------
const int JOY_CENTER_X = 2048;
const int JOY_CENTER_Y = 2048;
const int ADC_MIN = 0;
const int ADC_MAX = 4095;

// -------- Timing --------
const unsigned long SEND_INTERVAL_MS = 100;
const unsigned long WIFI_RETRY_MS = 1500;

unsigned long lastSendMs = 0;
unsigned long lastWifiRetryMs = 0;
bool controllerModeEnabled = false;
bool useFallbackIp = false;

String makeUrl(const String& path) {
  if (useFallbackIp) {
    return String(CAR_BASE_URL_FALLBACK) + path;
  }
  return String(CAR_BASE_URL) + path;
}

bool postJson(const String& path, const String& body, int expectedCode = 200) {
  if (WiFi.status() != WL_CONNECTED) return false;

  HTTPClient http;
  String url = makeUrl(path);
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  int code = http.POST(body);
  http.end();

  if (code == expectedCode) return true;

  // If mDNS name failed, try fallback IP once next time.
  if (!useFallbackIp && code < 0) {
    useFallbackIp = true;
  }

  return false;
}

void connectWifi() {
  if (WiFi.status() == WL_CONNECTED) return;

  if (millis() - lastWifiRetryMs < WIFI_RETRY_MS) return;
  lastWifiRetryMs = millis();

  Serial.println("[CTRL] WiFi baglaniliyor...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(CAR_AP_SSID, CAR_AP_PASS);
}

void enableControllerMode() {
  if (controllerModeEnabled || WiFi.status() != WL_CONNECTED) return;

  bool ok = postJson("/api/controller/mode", "{\"enabled\":true}");
  if (ok) {
    controllerModeEnabled = true;
    Serial.println("[CTRL] Controller mode aktif edildi.");
  } else {
    Serial.println("[CTRL] Controller mode aktif edilemedi, tekrar denenecek.");
  }
}

int readAdcClamped(int pin) {
  int v = analogRead(pin);
  if (v < ADC_MIN) return ADC_MIN;
  if (v > ADC_MAX) return ADC_MAX;
  return v;
}

void sendControllerInput() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (!controllerModeEnabled) return;

  if (millis() - lastSendMs < SEND_INTERVAL_MS) return;
  lastSendMs = millis();

  int x = readAdcClamped(JOY_X_PIN);
  int y = readAdcClamped(JOY_Y_PIN);

  // Buttons are active-low
  bool horn = (digitalRead(JOY_SW_PIN) == LOW);
  bool estop = (digitalRead(ESTOP_PIN) == LOW);

  String body = "{";
  body += "\"x\":" + String(x) + ",";
  body += "\"y\":" + String(y) + ",";
  body += "\"estop\":" + String(estop ? "true" : "false") + ",";
  body += "\"horn\":" + String(horn ? "true" : "false");
  body += "}";

  bool ok = postJson("/api/controller/input", body);
  if (!ok) {
    // Force re-enable in case car reset due to failsafe
    controllerModeEnabled = false;
  }

  Serial.print("[CTRL] x=");
  Serial.print(x);
  Serial.print(" y=");
  Serial.print(y);
  Serial.print(" horn=");
  Serial.print(horn ? "1" : "0");
  Serial.print(" estop=");
  Serial.print(estop ? "1" : "0");
  Serial.print(" ok=");
  Serial.println(ok ? "1" : "0");
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(JOY_SW_PIN, INPUT_PULLUP);
  pinMode(ESTOP_PIN, INPUT_PULLUP);

  // Use full 12-bit ADC range on ESP32
  analogReadResolution(12);

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(CAR_AP_SSID, CAR_AP_PASS);

  Serial.println("[CTRL] Basladi.");
  Serial.println("[CTRL] Joy SW = horn, ESTOP butonu = emergency stop");
  Serial.println("[CTRL] Joystick center hard-coded: x=2048 y=2048");
}

void loop() {
  connectWifi();

  if (WiFi.status() == WL_CONNECTED) {
    enableControllerMode();
    sendControllerInput();
  } else {
    controllerModeEnabled = false;
  }

  delay(5);
}
