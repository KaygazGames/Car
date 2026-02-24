#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_NeoPixel.h>

// ====== WiFi AP ======
const char* AP_SSID = "Car-Control";
const char* AP_PASS = "car12345";

// ====== Motor config (2x L298N) ======
const int LEFT_ENA = 5;
const int LEFT_IN1 = 6;
const int LEFT_IN2 = 7;
const int RIGHT_ENB = 8;
const int RIGHT_IN3 = 9;
const int RIGHT_IN4 = 10;

const int PWM_CHANNEL_LEFT = 0;
const int PWM_CHANNEL_RIGHT = 1;
const int PWM_FREQ = 15000;
const int PWM_RESOLUTION = 8;
const int PWM_LIMIT = 200;

// ====== Horn (Passive buzzer) ======
const int BUZZER_PIN = 4;
const int BUZZER_PWM_CHANNEL = 3;
const int BUZZER_FREQ = 2200;
const int BUZZER_DUTY = 120;

// ====== RGB (ESP32-S3 integrated WS2812 chain) ======
// Requested spec:
// rgb_order: RBG, pin: GPIO48, num_leds: 3, rmt_channel: 2, chipset: ws2812
const int RGB_PIN = 48;
const int RGB_LED_COUNT = 3;
Adafruit_NeoPixel rgb(RGB_LED_COUNT, RGB_PIN, NEO_GRB + NEO_KHZ800);
unsigned long lastRgbMs = 0;
uint8_t rgbPhase = 0;

// ====== ToF ======
Adafruit_VL53L0X lox;
bool tofEnabled = true;
uint16_t stopDistanceMm = 250;
uint16_t lastDistanceMm = 8190;
unsigned long lastTofReadMs = 0;
const unsigned long TOF_READ_INTERVAL_MS = 80;

// ====== Runtime state ======
WebServer server(80);

enum MotionCommand { CMD_STOP, CMD_FORWARD, CMD_BACKWARD, CMD_LEFT, CMD_RIGHT };
MotionCommand currentCommand = CMD_STOP;

bool flipLeft = false;
bool flipRight = false;
bool autoStopTriggered = false;
bool hornOn = false;

// ====== Controller mode ======
bool controllerMode = false;
const int JOY_CENTER_X = 2048;
const int JOY_CENTER_Y = 2048;
const int JOY_DEADZONE = 320;
int controllerX = JOY_CENTER_X;
int controllerY = JOY_CENTER_Y;
bool controllerEstop = false;
bool controllerHorn = false;

// ====== Failsafe ======
unsigned long lastPingMs = 0;
const unsigned long PING_TIMEOUT_MS = 1200;
bool failsafeTriggered = false;
bool resetScheduled = false;
unsigned long resetAtMs = 0;

const char MOBILE_HTML[] PROGMEM = R"rawliteral(
<!doctype html><html lang="tr"><head>
<meta charset="utf-8"/><meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>RC Car Mobile</title>
<style>
:root{--bg:#0a1122;--card:#121b37;--accent:#4f8cff;--danger:#ff5a7a;--ok:#38d39f;--text:#ecf2ff}
*{box-sizing:border-box;font-family:Inter,system-ui,Arial,sans-serif}
body{margin:0;background:radial-gradient(circle at top,#1d2a55 0,#0a1122 62%);color:var(--text);min-height:100vh;display:flex;align-items:center;justify-content:center;padding:14px}
.card{width:min(480px,100%);background:rgba(18,27,55,.85);border:1px solid #2a3a73;border-radius:18px;padding:16px;box-shadow:0 20px 50px rgba(0,0,0,.35)}
.grid{display:grid;grid-template-columns:1fr 1fr 1fr;gap:10px}.row{display:flex;gap:10px;margin-top:10px}
button{border:none;border-radius:12px;padding:16px;font-weight:700;color:white;background:#223568;cursor:pointer}
button:active,.pressed{background:var(--accent)}.stop{background:var(--danger)}.horn{background:#ffb648;color:#212}.status{opacity:.9;font-size:.92rem}
</style></head><body><div class="card">
<h2>🚗 Mobil Kontrol</h2><div id="st" class="status">Bağlanıyor...</div>
<div class="grid">
<div></div><button data-cmd="forward">⬆ İleri</button><div></div>
<button data-cmd="left">⬅ Sol</button><button class="stop" data-cmd="stop">■ Dur</button><button data-cmd="right">Sağ ➡</button>
<div></div><button data-cmd="backward">⬇ Geri</button><div></div>
</div>
<div class="row"><button class="horn" id="hornBtn">📣 Korna</button></div>
</div>
<script>
async function post(u,d={}){return fetch(u,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(d)})}
function bindHold(btn){const c=btn.dataset.cmd;const dn=()=>{btn.classList.add('pressed');post('/api/cmd',{cmd:c})};const up=()=>{btn.classList.remove('pressed');if(c!=='stop')post('/api/cmd',{cmd:'stop'})};
btn.addEventListener('touchstart',e=>{e.preventDefault();dn()},{passive:false});btn.addEventListener('touchend',e=>{e.preventDefault();up()},{passive:false});btn.addEventListener('mousedown',dn);btn.addEventListener('mouseup',up);btn.addEventListener('mouseleave',up)}
[...document.querySelectorAll('button[data-cmd]')].forEach(bindHold);
const hb=document.getElementById('hornBtn');
hb.addEventListener('mousedown',()=>post('/api/horn',{on:true}));hb.addEventListener('mouseup',()=>post('/api/horn',{on:false}));hb.addEventListener('mouseleave',()=>post('/api/horn',{on:false}));
hb.addEventListener('touchstart',e=>{e.preventDefault();post('/api/horn',{on:true})},{passive:false});hb.addEventListener('touchend',e=>{e.preventDefault();post('/api/horn',{on:false})},{passive:false});
setInterval(()=>post('/api/ping').catch(()=>{}),300);
setInterval(async()=>{try{const s=await (await fetch('/api/state')).json();document.getElementById('st').textContent=`Komut:${s.command} | ToF:${s.distance_mm}mm | AutoStop:${s.auto_stop?'Açık':'Kapalı'} | Controller:${s.controller_mode?'Açık':'Kapalı'}`;}catch(e){document.getElementById('st').textContent='Bağlantı hatası';}},450);
</script></body></html>
)rawliteral";

const char DESKTOP_HTML[] PROGMEM = R"rawliteral(
<!doctype html><html lang="tr"><head><meta charset="utf-8"/><meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>RC Car Desktop</title><style>
body{margin:0;min-height:100vh;display:grid;place-items:center;background:#0a1020;color:#eaf0ff;font-family:Inter,Arial}
.wrap{background:#121a33;border:1px solid #2b3a72;border-radius:16px;padding:22px;width:min(640px,94%)}
.kbd{display:grid;grid-template-columns:repeat(3,72px);gap:10px;justify-content:center}.key{height:72px;border-radius:12px;background:#213163;display:grid;place-items:center;font-weight:700}
.on{background:#4f8cff}.row{display:flex;gap:10px;margin-top:14px}button{border:none;border-radius:10px;padding:10px 14px;font-weight:700;cursor:pointer}
.horn{background:#ffb648}.danger{background:#ff5a7a;color:#fff}
</style></head><body><div class="wrap">
<h2>🖥️ Desktop WASD</h2><p>W/A/S/D sürüş, Space stop, H korna.</p>
<div class="kbd"><div></div><div id="kW" class="key">W</div><div></div><div id="kA" class="key">A</div><div id="kS" class="key">S</div><div id="kD" class="key">D</div></div>
<div class="row"><button id="horn" class="horn">📣 Horn</button><button id="e" class="danger">⛔ E-STOP</button></div>
<p id="state"></p></div><script>
const m={w:'forward',a:'left',s:'backward',d:'right'};let active=null;const horn=document.getElementById('horn');
async function post(u,d={}){return fetch(u,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(d)})}
function setKey(k,on){const el=document.getElementById('k'+k.toUpperCase());if(el)el.classList.toggle('on',on)}
window.addEventListener('keydown',e=>{const k=e.key.toLowerCase();if(k===' '){active=null;Object.keys(m).forEach(x=>setKey(x,false));post('/api/cmd',{cmd:'stop'});return;}if(k==='h'){post('/api/horn',{on:true});return;}if(!m[k]||active===k)return;active=k;Object.keys(m).forEach(x=>setKey(x,x===k));post('/api/cmd',{cmd:m[k]});});
window.addEventListener('keyup',e=>{const k=e.key.toLowerCase();if(k==='h'){post('/api/horn',{on:false});return;}if(k!==active)return;active=null;setKey(k,false);post('/api/cmd',{cmd:'stop'});});
horn.onmousedown=()=>post('/api/horn',{on:true});horn.onmouseup=()=>post('/api/horn',{on:false});horn.onmouseleave=()=>post('/api/horn',{on:false});
document.getElementById('e').onclick=()=>post('/api/estop',{active:true});
setInterval(()=>post('/api/ping').catch(()=>{}),300);
setInterval(async()=>{const s=await (await fetch('/api/state')).json();document.getElementById('state').textContent=`Komut:${s.command} | Failsafe:${s.failsafe?'Evet':'Hayır'} | Controller:${s.controller_mode?'Açık':'Kapalı'}`},450);
</script></body></html>
)rawliteral";

const char DEBUG_HTML[] PROGMEM = R"rawliteral(
<!doctype html><html lang="tr"><head><meta charset="utf-8"/><meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>RC Car Debug</title><style>
body{margin:0;background:#0c1122;color:#f1f5ff;font-family:Inter,Arial;padding:20px}.panel{max-width:780px;margin:auto;background:#151d39;border:1px solid #2d3b71;border-radius:14px;padding:16px}
.row{display:flex;gap:10px;flex-wrap:wrap;align-items:center;margin:10px 0}input[type=number]{background:#0e1630;border:1px solid #3a4b89;color:#fff;padding:8px;border-radius:8px;width:140px}
button{background:#4f8cff;border:none;color:white;border-radius:10px;padding:10px 14px;font-weight:600;cursor:pointer}.danger{background:#ff5a7a}
pre{background:#091029;padding:10px;border-radius:10px;overflow:auto}</style></head><body><div class="panel"><h2>🧪 Debug</h2>
<div class="row"><label><input id="flipL" type="checkbox"/> Left Flip</label><label><input id="flipR" type="checkbox"/> Right Flip</label><button onclick="saveFlip()">Kaydet</button></div>
<div class="row"><label><input id="tofEn" type="checkbox"/> ToF Active</label><input id="tofDist" type="number" min="80" max="2000"/> mm <button onclick="saveTof()">ToF Kaydet</button></div>
<div class="row"><label><input id="ctrlMode" type="checkbox"/> Controller Mode</label><button onclick="saveCtrl()">Controller Mode Kaydet</button><button class="danger" onclick="estop()">E-STOP</button></div>
<pre id="dump"></pre></div><script>
async function post(u,d={}){return fetch(u,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(d)})}
async function refresh(){const s=await (await fetch('/api/state')).json();flipL.checked=s.flip_left;flipR.checked=s.flip_right;tofEn.checked=s.tof_enabled;tofDist.value=s.stop_distance_mm;ctrlMode.checked=s.controller_mode;dump.textContent=JSON.stringify(s,null,2)}
async function saveFlip(){await post('/api/config/flip',{left:flipL.checked,right:flipR.checked});refresh()}
async function saveTof(){await post('/api/config/tof',{enabled:tofEn.checked,distance_mm:Number(tofDist.value)});refresh()}
async function saveCtrl(){await post('/api/controller/mode',{enabled:ctrlMode.checked});refresh()}
async function estop(){await post('/api/estop',{active:true});refresh()}
setInterval(()=>post('/api/ping').catch(()=>{}),300);setInterval(refresh,700);refresh();
</script></body></html>
)rawliteral";

String commandToString(MotionCommand c) {
  if (c == CMD_FORWARD) return "forward";
  if (c == CMD_BACKWARD) return "backward";
  if (c == CMD_LEFT) return "left";
  if (c == CMD_RIGHT) return "right";
  return "stop";
}

void setRgbRBG(uint8_t r, uint8_t b, uint8_t g) {
  // Adafruit_NeoPixel constructor is GRB; remap requested RBG logical order.
  uint32_t color = rgb.Color(r, g, b);
  for (int i = 0; i < RGB_LED_COUNT; i++) rgb.setPixelColor(i, color);
  rgb.show();
}

void updateRgb() {
  if (failsafeTriggered) {
    setRgbRBG(255, 0, 0);
    return;
  }
  if (controllerMode) {
    if (millis() - lastRgbMs > 45) {
      lastRgbMs = millis();
      rgbPhase++;
      uint8_t r = (sin((rgbPhase + 0) * 0.1f) * 127) + 128;
      uint8_t g = (sin((rgbPhase + 85) * 0.1f) * 127) + 128;
      uint8_t b = (sin((rgbPhase + 170) * 0.1f) * 127) + 128;
      setRgbRBG(r, b, g);
    }
    return;
  }
  setRgbRBG(0, 20, 80);
}

void hornSet(bool on) {
  hornOn = on;
  if (on) {
    ledcWriteTone(BUZZER_PWM_CHANNEL, BUZZER_FREQ);
    ledcWrite(BUZZER_PWM_CHANNEL, BUZZER_DUTY);
  } else {
    ledcWrite(BUZZER_PWM_CHANNEL, 0);
  }
}

void driveSide(int inA, int inB, int pwmChannel, int speedValue, bool reverse) {
  bool forward = speedValue >= 0;
  int pwm = abs(speedValue);
  if (pwm > PWM_LIMIT) pwm = PWM_LIMIT;
  if (reverse) forward = !forward;
  digitalWrite(inA, forward ? HIGH : LOW);
  digitalWrite(inB, forward ? LOW : HIGH);
  ledcWrite(pwmChannel, pwm);
}

void applyMotion(MotionCommand cmd) {
  if (autoStopTriggered && cmd == CMD_FORWARD) cmd = CMD_STOP;

  int left = 0;
  int right = 0;
  if (cmd == CMD_FORWARD) {
    left = PWM_LIMIT;
    right = PWM_LIMIT;
  } else if (cmd == CMD_BACKWARD) {
    left = -PWM_LIMIT;
    right = -PWM_LIMIT;
  } else if (cmd == CMD_LEFT) {
    left = -PWM_LIMIT;
    right = PWM_LIMIT;
  } else if (cmd == CMD_RIGHT) {
    left = PWM_LIMIT;
    right = -PWM_LIMIT;
  }

  if (left == 0) ledcWrite(PWM_CHANNEL_LEFT, 0);
  else driveSide(LEFT_IN1, LEFT_IN2, PWM_CHANNEL_LEFT, left, flipLeft);

  if (right == 0) ledcWrite(PWM_CHANNEL_RIGHT, 0);
  else driveSide(RIGHT_IN3, RIGHT_IN4, PWM_CHANNEL_RIGHT, right, flipRight);

  currentCommand = cmd;
}

void stopCar() { applyMotion(CMD_STOP); }

void updateTof() {
  if (!tofEnabled) {
    autoStopTriggered = false;
    return;
  }

  if (millis() - lastTofReadMs < TOF_READ_INTERVAL_MS) return;
  lastTofReadMs = millis();

  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) {
    lastDistanceMm = measure.RangeMilliMeter;
    autoStopTriggered = (lastDistanceMm <= stopDistanceMm);
    if (autoStopTriggered && currentCommand == CMD_FORWARD) stopCar();
  }
}

void triggerFailsafeReset() {
  failsafeTriggered = true;
  stopCar();
  hornSet(false);
  resetScheduled = true;
  resetAtMs = millis() + 500;
}

void handleFailsafe() {
  if (!failsafeTriggered && millis() - lastPingMs > PING_TIMEOUT_MS) {
    triggerFailsafeReset();
  }
  if (resetScheduled && millis() >= resetAtMs) {
    ESP.restart();
  }
}

void refreshAliveSignal() {
  lastPingMs = millis();
  failsafeTriggered = false;
  resetScheduled = false;
}

void onPing() {
  refreshAliveSignal();
  server.send(200, "application/json", "{\"ok\":true}");
}

void onCmd() {
  if (controllerMode) {
    server.send(423, "application/json", "{\"error\":\"controller_mode_active\"}");
    return;
  }
  String body = server.arg("plain");
  MotionCommand cmd = CMD_STOP;
  if (body.indexOf("forward") >= 0) cmd = CMD_FORWARD;
  else if (body.indexOf("backward") >= 0) cmd = CMD_BACKWARD;
  else if (body.indexOf("left") >= 0) cmd = CMD_LEFT;
  else if (body.indexOf("right") >= 0) cmd = CMD_RIGHT;
  applyMotion(cmd);
  server.send(200, "application/json", "{\"ok\":true}");
}

void onHorn() {
  String body = server.arg("plain");
  hornSet(body.indexOf("true") >= 0);
  server.send(200, "application/json", "{\"ok\":true}");
}

void onEstop() {
  stopCar();
  hornSet(false);
  controllerEstop = true;
  server.send(200, "application/json", "{\"ok\":true}");
}

void onFlipConfig() {
  String body = server.arg("plain");
  if (body.indexOf("\"left\":true") >= 0) flipLeft = true;
  if (body.indexOf("\"left\":false") >= 0) flipLeft = false;
  if (body.indexOf("\"right\":true") >= 0) flipRight = true;
  if (body.indexOf("\"right\":false") >= 0) flipRight = false;
  server.send(200, "application/json", "{\"ok\":true}");
}

void onTofConfig() {
  String body = server.arg("plain");
  if (body.indexOf("\"enabled\":true") >= 0) tofEnabled = true;
  if (body.indexOf("\"enabled\":false") >= 0) tofEnabled = false;
  int idx = body.indexOf("distance_mm");
  if (idx >= 0) {
    int colon = body.indexOf(':', idx);
    int end = body.indexOf('}', colon);
    int val = body.substring(colon + 1, end).toInt();
    if (val >= 80 && val <= 2000) stopDistanceMm = val;
  }
  server.send(200, "application/json", "{\"ok\":true}");
}

void onControllerMode() {
  String body = server.arg("plain");
  controllerMode = body.indexOf("true") >= 0;
  if (!controllerMode) {
    controllerEstop = false;
    stopCar();
  }
  refreshAliveSignal();
  server.send(200, "application/json", "{\"ok\":true}");
}

void onControllerInput() {
  // Example payload:
  // {"x":2100,"y":1300,"estop":false,"horn":true}
  String body = server.arg("plain");
  if (!controllerMode) {
    server.send(409, "application/json", "{\"error\":\"controller_mode_off\"}");
    return;
  }

  auto readInt = [&](const String& key, int fallback) {
    int idx = body.indexOf(key);
    if (idx < 0) return fallback;
    int colon = body.indexOf(':', idx);
    int end = body.indexOf(',', colon);
    if (end < 0) end = body.indexOf('}', colon);
    if (colon < 0 || end < 0) return fallback;
    return body.substring(colon + 1, end).toInt();
  };

  controllerX = readInt("\"x\"", controllerX);
  controllerY = readInt("\"y\"", controllerY);
  controllerEstop = body.indexOf("\"estop\":true") >= 0;
  controllerHorn = body.indexOf("\"horn\":true") >= 0;

  refreshAliveSignal();
  server.send(200, "application/json", "{\"ok\":true}");
}

void evaluateControllerDrive() {
  if (!controllerMode) return;

  if (controllerEstop) {
    stopCar();
    hornSet(false);
    return;
  }

  hornSet(controllerHorn);

  int dx = controllerX - JOY_CENTER_X;
  int dy = controllerY - JOY_CENTER_Y;

  if (abs(dx) < JOY_DEADZONE && abs(dy) < JOY_DEADZONE) {
    stopCar();
    return;
  }

  if (dy < -JOY_DEADZONE) applyMotion(CMD_FORWARD);
  else if (dy > JOY_DEADZONE) applyMotion(CMD_BACKWARD);
  else if (dx < -JOY_DEADZONE) applyMotion(CMD_LEFT);
  else if (dx > JOY_DEADZONE) applyMotion(CMD_RIGHT);
  else stopCar();
}

void onState() {
  String json = "{";
  json += "\"command\":\"" + commandToString(currentCommand) + "\",";
  json += "\"distance_mm\":" + String(lastDistanceMm) + ",";
  json += "\"auto_stop\":" + String(autoStopTriggered ? "true" : "false") + ",";
  json += "\"tof_enabled\":" + String(tofEnabled ? "true" : "false") + ",";
  json += "\"stop_distance_mm\":" + String(stopDistanceMm) + ",";
  json += "\"flip_left\":" + String(flipLeft ? "true" : "false") + ",";
  json += "\"flip_right\":" + String(flipRight ? "true" : "false") + ",";
  json += "\"horn\":" + String(hornOn ? "true" : "false") + ",";
  json += "\"controller_mode\":" + String(controllerMode ? "true" : "false") + ",";
  json += "\"failsafe\":" + String(failsafeTriggered ? "true" : "false");
  json += "}";
  server.send(200, "application/json", json);
}

void setupServer() {
  server.on("/mobile", HTTP_GET, []() { server.send_P(200, "text/html", MOBILE_HTML); });
  server.on("/desktop", HTTP_GET, []() { server.send_P(200, "text/html", DESKTOP_HTML); });
  server.on("/debug", HTTP_GET, []() { server.send_P(200, "text/html", DEBUG_HTML); });

  server.on("/api/ping", HTTP_POST, onPing);
  server.on("/api/cmd", HTTP_POST, onCmd);
  server.on("/api/horn", HTTP_POST, onHorn);
  server.on("/api/estop", HTTP_POST, onEstop);
  server.on("/api/state", HTTP_GET, onState);
  server.on("/api/config/flip", HTTP_POST, onFlipConfig);
  server.on("/api/config/tof", HTTP_POST, onTofConfig);
  server.on("/api/controller/mode", HTTP_POST, onControllerMode);
  server.on("/api/controller/input", HTTP_POST, onControllerInput);

  server.onNotFound([]() {
    server.sendHeader("Location", "/mobile", true);
    server.send(302, "text/plain", "Redirecting");
  });

  server.begin();
}

void setupMotors() {
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN3, OUTPUT);
  pinMode(RIGHT_IN4, OUTPUT);

  ledcSetup(PWM_CHANNEL_LEFT, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(LEFT_ENA, PWM_CHANNEL_LEFT);
  ledcSetup(PWM_CHANNEL_RIGHT, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(RIGHT_ENB, PWM_CHANNEL_RIGHT);

  ledcSetup(BUZZER_PWM_CHANNEL, BUZZER_FREQ, 8);
  ledcAttachPin(BUZZER_PIN, BUZZER_PWM_CHANNEL);

  stopCar();
  hornSet(false);
}

void setupRgb() {
  rgb.begin();
  rgb.setBrightness(70);
  setRgbRBG(0, 20, 80);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  setupMotors();
  setupRgb();

  if (!lox.begin()) {
    Serial.println("ToF init failed, sensor disabled.");
    tofEnabled = false;
  }

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  IPAddress ip = WiFi.softAPIP();

  if (MDNS.begin("car")) {
    MDNS.addService("http", "tcp", 80);
  }

  lastPingMs = millis();
  setupServer();

  Serial.print("AP IP: ");
  Serial.println(ip);
  Serial.println("Open: http://car.local/mobile");
}

void loop() {
  server.handleClient();
  updateTof();
  evaluateControllerDrive();
  updateRgb();
  handleFailsafe();
}
