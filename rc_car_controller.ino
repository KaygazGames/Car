#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_NeoPixel.h>

const char* AP_SSID = "Car-Control";
const char* AP_PASS = "car12345";

// 4 motors via 2x L298N (Board1: left front/right front, Board2: left rear/right rear)
const int M_LF_EN = 5;
const int M_LF_IN1 = 6;
const int M_LF_IN2 = 7;

const int M_RF_EN = 8;
const int M_RF_IN1 = 9;
const int M_RF_IN2 = 10;

const int M_LR_EN = 11;
const int M_LR_IN1 = 12;
const int M_LR_IN2 = 13;

const int M_RR_EN = 14;
const int M_RR_IN1 = 15;
const int M_RR_IN2 = 16;

const int PWM_FREQ = 15000;
const int PWM_RESOLUTION = 8;
const int PWM_LIMIT = 200;

const int BUZZER_PIN = 4;
const int BUZZER_FREQ = 2200;
const int BUZZER_DUTY = 120;

const int RGB_PIN = 48;
const int RGB_LED_COUNT = 3;
Adafruit_NeoPixel rgb(RGB_LED_COUNT, RGB_PIN, NEO_GRB + NEO_KHZ800);
unsigned long lastRgbMs = 0;
uint8_t rgbPhase = 0;

Adafruit_VL53L0X lox;
bool tofEnabled = true;
uint16_t stopDistanceMm = 250;
uint16_t lastDistanceMm = 8190;
unsigned long lastTofReadMs = 0;
const unsigned long TOF_READ_INTERVAL_MS = 80;

WebServer server(80);
enum MotionCommand { CMD_STOP, CMD_FORWARD, CMD_BACKWARD, CMD_LEFT, CMD_RIGHT };
MotionCommand currentCommand = CMD_STOP;

bool flipLeft = false;
bool flipRight = false;
bool autoStopTriggered = false;
bool hornOn = false;

bool controllerMode = false;
const int JOY_CENTER_X = 2048;
const int JOY_CENTER_Y = 2048;
const int JOY_DEADZONE = 320;
int controllerX = JOY_CENTER_X;
int controllerY = JOY_CENTER_Y;
bool controllerEstop = false;
bool controllerHorn = false;

unsigned long lastPingMs = 0;
const unsigned long PING_TIMEOUT_MS = 1200;
bool failsafeTriggered = false;
bool resetScheduled = false;
unsigned long resetAtMs = 0;

const char MOBILE_HTML[] PROGMEM = R"rawliteral(
<!doctype html><html lang="tr"><head><meta charset="utf-8"/><meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>RC Car Mobile</title><style>
:root{--bg:#0a1122;--card:#121b37;--accent:#4f8cff;--danger:#ff5a7a;--text:#ecf2ff}
*{box-sizing:border-box;font-family:Inter,system-ui,Arial,sans-serif}
body{margin:0;background:radial-gradient(circle at top,#1d2a55 0,#0a1122 62%);color:var(--text);min-height:100vh;display:flex;align-items:center;justify-content:center;padding:14px}
.card{width:min(500px,100%);background:rgba(18,27,55,.9);border:1px solid #2a3a73;border-radius:18px;padding:16px}
.grid{display:grid;grid-template-columns:1fr 1fr 1fr;gap:10px}.row{display:flex;gap:10px;margin-top:10px}
button{border:none;border-radius:12px;padding:16px;font-weight:700;color:white;background:#223568;cursor:pointer}
.stop{background:var(--danger)}.horn{background:#ffb648;color:#222}
</style></head><body><div class="card"><h2>🚗 Mobil Kontrol</h2><div id="st">...</div>
<div class="grid"><div></div><button data-cmd="forward">⬆ İleri</button><div></div><button data-cmd="left">⬅ Sol</button><button class="stop" data-cmd="stop">■ Dur</button><button data-cmd="right">Sağ ➡</button><div></div><button data-cmd="backward">⬇ Geri</button><div></div></div>
<div class="row"><button id="hornBtn" class="horn">📣 Horn</button><button onclick="post('/api/estop',{active:true})" class="stop">⛔ E-STOP</button></div></div>
<script>
async function post(u,d={}){return fetch(u,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(d)})}
function bindHold(btn){const c=btn.dataset.cmd;const dn=()=>post('/api/cmd',{cmd:c});const up=()=>{if(c!=='stop')post('/api/cmd',{cmd:'stop'})};btn.onmousedown=dn;btn.onmouseup=up;btn.onmouseleave=up;btn.ontouchstart=e=>{e.preventDefault();dn()};btn.ontouchend=e=>{e.preventDefault();up()};}
[...document.querySelectorAll('button[data-cmd]')].forEach(bindHold);
const hb=document.getElementById('hornBtn');hb.onmousedown=()=>post('/api/horn',{on:true});hb.onmouseup=()=>post('/api/horn',{on:false});hb.onmouseleave=()=>post('/api/horn',{on:false});hb.ontouchstart=e=>{e.preventDefault();post('/api/horn',{on:true})};hb.ontouchend=e=>{e.preventDefault();post('/api/horn',{on:false})};
setInterval(()=>post('/api/ping').catch(()=>{}),300);
setInterval(async()=>{try{const s=await (await fetch('/api/state')).json();document.getElementById('st').textContent=`Komut:${s.command} ToF:${s.distance_mm} AutoStop:${s.auto_stop?'on':'off'} Controller:${s.controller_mode?'on':'off'}`;}catch(e){document.getElementById('st').textContent='bağlantı hatası'}},400);
</script></body></html>
)rawliteral";

const char DESKTOP_HTML[] PROGMEM = R"rawliteral(
<!doctype html><html lang="tr"><head><meta charset="utf-8"/><meta name="viewport" content="width=device-width,initial-scale=1"/><title>Desktop</title></head>
<body style="font-family:Inter,Arial;background:#0a1020;color:#fff"><h2>WASD / Space Stop / H Horn / E E-Stop</h2><pre id="s"></pre>
<script>
const m={w:'forward',a:'left',s:'backward',d:'right'};let active='';
async function post(u,d={}){return fetch(u,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(d)})}
window.onkeydown=e=>{const k=e.key.toLowerCase();if(k===' '){active='';post('/api/cmd',{cmd:'stop'});return;}if(k==='h'){post('/api/horn',{on:true});return;}if(k==='e'){post('/api/estop',{active:true});return;}if(!m[k]||active===k)return;active=k;post('/api/cmd',{cmd:m[k]});};
window.onkeyup=e=>{const k=e.key.toLowerCase();if(k==='h'){post('/api/horn',{on:false});return;}if(k===active){active='';post('/api/cmd',{cmd:'stop'});}};
setInterval(()=>post('/api/ping').catch(()=>{}),300);setInterval(async()=>{const j=await (await fetch('/api/state')).json();document.getElementById('s').textContent=JSON.stringify(j,null,2)},500);
</script></body></html>
)rawliteral";

const char DEBUG_HTML[] PROGMEM = R"rawliteral(
<!doctype html><html><body style="font-family:Inter,Arial;background:#111;color:#eee;padding:16px"><h2>Debug</h2><label><input id="fl" type="checkbox">flip left</label><label><input id="fr" type="checkbox">flip right</label><button onclick="sf()">save flip</button><br><label><input id="te" type="checkbox">tof</label><input id="td" type="number" value="250"><button onclick="st()">save tof</button><br><label><input id="cm" type="checkbox">controller mode</label><button onclick="scm()">save mode</button><button onclick="post('/api/estop',{active:true})">E-STOP</button><pre id="d"></pre>
<script>
async function post(u,d={}){return fetch(u,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(d)})}
async function r(){const s=await (await fetch('/api/state')).json();fl.checked=s.flip_left;fr.checked=s.flip_right;te.checked=s.tof_enabled;td.value=s.stop_distance_mm;cm.checked=s.controller_mode;d.textContent=JSON.stringify(s,null,2)}
function sf(){post('/api/config/flip',{left:fl.checked,right:fr.checked}).then(r)}
function st(){post('/api/config/tof',{enabled:te.checked,distance_mm:Number(td.value)}).then(r)}
function scm(){post('/api/controller/mode',{enabled:cm.checked}).then(r)}
setInterval(()=>post('/api/ping').catch(()=>{}),300);setInterval(r,700);r();
</script></body></html>
)rawliteral";

String commandToString(MotionCommand c){if(c==CMD_FORWARD)return"forward";if(c==CMD_BACKWARD)return"backward";if(c==CMD_LEFT)return"left";if(c==CMD_RIGHT)return"right";return"stop";}

void setRgbRBG(uint8_t r, uint8_t b, uint8_t g){
  uint32_t color = rgb.Color(r, g, b); // logical RBG remap
  for(int i=0;i<RGB_LED_COUNT;i++) rgb.setPixelColor(i,color);
  rgb.show();
}

void updateRgb(){
  if(failsafeTriggered){setRgbRBG(255,0,0);return;}
  if(controllerMode){
    if(millis()-lastRgbMs>45){
      lastRgbMs=millis(); rgbPhase++;
      uint8_t r=(sin((rgbPhase+0)*0.1f)*127)+128;
      uint8_t g=(sin((rgbPhase+85)*0.1f)*127)+128;
      uint8_t b=(sin((rgbPhase+170)*0.1f)*127)+128;
      setRgbRBG(r,b,g);
    }
    return;
  }
  setRgbRBG(0,20,80);
}

void motorWrite(int enPin,int in1,int in2,int speedValue,bool reverse){
  bool forward = speedValue >= 0;
  int pwm = abs(speedValue);
  if (pwm > PWM_LIMIT) pwm = PWM_LIMIT;
  if (reverse) forward = !forward;
  digitalWrite(in1, forward ? HIGH : LOW);
  digitalWrite(in2, forward ? LOW : HIGH);
  ledcWrite(enPin, pwm);
}

void setLeftPair(int speed){
  if(speed==0){ledcWrite(M_LF_EN,0);ledcWrite(M_LR_EN,0);return;}
  motorWrite(M_LF_EN,M_LF_IN1,M_LF_IN2,speed,flipLeft);
  motorWrite(M_LR_EN,M_LR_IN1,M_LR_IN2,speed,flipLeft);
}

void setRightPair(int speed){
  if(speed==0){ledcWrite(M_RF_EN,0);ledcWrite(M_RR_EN,0);return;}
  motorWrite(M_RF_EN,M_RF_IN1,M_RF_IN2,speed,flipRight);
  motorWrite(M_RR_EN,M_RR_IN1,M_RR_IN2,speed,flipRight);
}

void hornSet(bool on){
  hornOn = on;
  if(on){ledcWriteTone(BUZZER_PIN, BUZZER_FREQ);ledcWrite(BUZZER_PIN, BUZZER_DUTY);} else ledcWrite(BUZZER_PIN,0);
}

void applyMotion(MotionCommand cmd){
  if(autoStopTriggered && cmd==CMD_FORWARD) cmd = CMD_STOP;
  int left=0,right=0;
  if(cmd==CMD_FORWARD){left=PWM_LIMIT;right=PWM_LIMIT;}
  else if(cmd==CMD_BACKWARD){left=-PWM_LIMIT;right=-PWM_LIMIT;}
  else if(cmd==CMD_LEFT){left=-PWM_LIMIT;right=PWM_LIMIT;}
  else if(cmd==CMD_RIGHT){left=PWM_LIMIT;right=-PWM_LIMIT;}
  setLeftPair(left); setRightPair(right); currentCommand=cmd;
}

void stopCar(){applyMotion(CMD_STOP);}

void updateTof(){
  if(!tofEnabled){autoStopTriggered=false;return;}
  if(millis()-lastTofReadMs<TOF_READ_INTERVAL_MS) return;
  lastTofReadMs=millis();
  VL53L0X_RangingMeasurementData_t measure; lox.rangingTest(&measure,false);
  if(measure.RangeStatus!=4){
    lastDistanceMm = measure.RangeMilliMeter;
    autoStopTriggered = (lastDistanceMm <= stopDistanceMm);
    if(autoStopTriggered && currentCommand==CMD_FORWARD) stopCar();
  }
}

void refreshAliveSignal(){lastPingMs=millis();failsafeTriggered=false;resetScheduled=false;}
void triggerFailsafeReset(){failsafeTriggered=true;stopCar();hornSet(false);resetScheduled=true;resetAtMs=millis()+500;}
void handleFailsafe(){if(!failsafeTriggered && millis()-lastPingMs>PING_TIMEOUT_MS) triggerFailsafeReset(); if(resetScheduled && millis()>=resetAtMs) ESP.restart();}

void onPing(){refreshAliveSignal();server.send(200,"application/json","{\"ok\":true}");}
void onHorn(){hornSet(server.arg("plain").indexOf("true")>=0);server.send(200,"application/json","{\"ok\":true}");}
void onEstop(){stopCar();hornSet(false);controllerEstop=true;server.send(200,"application/json","{\"ok\":true}");}

void onCmd(){
  if(controllerMode){server.send(423,"application/json","{\"error\":\"controller_mode_active\"}");return;}
  String b=server.arg("plain"); MotionCommand cmd=CMD_STOP;
  if(b.indexOf("forward")>=0)cmd=CMD_FORWARD; else if(b.indexOf("backward")>=0)cmd=CMD_BACKWARD; else if(b.indexOf("left")>=0)cmd=CMD_LEFT; else if(b.indexOf("right")>=0)cmd=CMD_RIGHT;
  applyMotion(cmd); server.send(200,"application/json","{\"ok\":true}");
}

void onFlipConfig(){String b=server.arg("plain"); if(b.indexOf("\"left\":true")>=0)flipLeft=true; if(b.indexOf("\"left\":false")>=0)flipLeft=false; if(b.indexOf("\"right\":true")>=0)flipRight=true; if(b.indexOf("\"right\":false")>=0)flipRight=false; server.send(200,"application/json","{\"ok\":true}");}

void onTofConfig(){
  String b=server.arg("plain");
  if(b.indexOf("\"enabled\":true")>=0)tofEnabled=true;
  if(b.indexOf("\"enabled\":false")>=0)tofEnabled=false;
  int idx=b.indexOf("distance_mm");
  if(idx>=0){
    int colon=b.indexOf(':',idx);
    int end=b.indexOf(',',colon); if(end<0) end=b.indexOf('}',colon);
    if(colon>=0 && end>colon){ int val=b.substring(colon+1,end).toInt(); if(val>=80 && val<=2000) stopDistanceMm=(uint16_t)val; }
  }
  server.send(200,"application/json","{\"ok\":true}");
}

void onControllerMode(){
  controllerMode = server.arg("plain").indexOf("true")>=0;
  if(!controllerMode){controllerEstop=false;controllerHorn=false;stopCar();}
  refreshAliveSignal();
  server.send(200,"application/json","{\"ok\":true}");
}

void onControllerInput(){
  String body=server.arg("plain");
  if(!controllerMode){server.send(409,"application/json","{\"error\":\"controller_mode_off\"}");return;}

  auto readInt = [&](const String& key, int fallback) -> int {
    int idx = body.indexOf(key);
    if (idx < 0) return fallback;
    int colon = body.indexOf(':', idx);
    int end = body.indexOf(',', colon);
    if (end < 0) end = body.indexOf('}', colon);
    if (colon < 0 || end < 0) return fallback;
    long parsed = body.substring(colon + 1, end).toInt();
    return (int)parsed;
  };

  controllerX = readInt("\"x\"", controllerX);
  controllerY = readInt("\"y\"", controllerY);
  controllerEstop = body.indexOf("\"estop\":true") >= 0;
  controllerHorn = body.indexOf("\"horn\":true") >= 0;
  refreshAliveSignal();
  server.send(200,"application/json","{\"ok\":true}");
}

void evaluateControllerDrive(){
  if(!controllerMode) return;
  if(controllerEstop){stopCar();hornSet(false);return;}
  hornSet(controllerHorn);
  int dx=controllerX-JOY_CENTER_X, dy=controllerY-JOY_CENTER_Y;
  if(abs(dx)<JOY_DEADZONE && abs(dy)<JOY_DEADZONE){stopCar();return;}
  if(dy < -JOY_DEADZONE) applyMotion(CMD_FORWARD);
  else if(dy > JOY_DEADZONE) applyMotion(CMD_BACKWARD);
  else if(dx < -JOY_DEADZONE) applyMotion(CMD_LEFT);
  else if(dx > JOY_DEADZONE) applyMotion(CMD_RIGHT);
  else stopCar();
}

void onState(){
  String j="{";
  j += "\"command\":\"" + commandToString(currentCommand) + "\",";
  j += "\"distance_mm\":" + String(lastDistanceMm) + ",";
  j += "\"auto_stop\":" + String(autoStopTriggered?"true":"false") + ",";
  j += "\"tof_enabled\":" + String(tofEnabled?"true":"false") + ",";
  j += "\"stop_distance_mm\":" + String(stopDistanceMm) + ",";
  j += "\"flip_left\":" + String(flipLeft?"true":"false") + ",";
  j += "\"flip_right\":" + String(flipRight?"true":"false") + ",";
  j += "\"horn\":" + String(hornOn?"true":"false") + ",";
  j += "\"controller_mode\":" + String(controllerMode?"true":"false") + ",";
  j += "\"failsafe\":" + String(failsafeTriggered?"true":"false") + "}";
  server.send(200,"application/json",j);
}

void setupServer(){
  server.on("/mobile",HTTP_GET,[](){server.send_P(200,"text/html",MOBILE_HTML);});
  server.on("/desktop",HTTP_GET,[](){server.send_P(200,"text/html",DESKTOP_HTML);});
  server.on("/debug",HTTP_GET,[](){server.send_P(200,"text/html",DEBUG_HTML);});
  server.on("/api/ping",HTTP_POST,onPing);
  server.on("/api/cmd",HTTP_POST,onCmd);
  server.on("/api/horn",HTTP_POST,onHorn);
  server.on("/api/estop",HTTP_POST,onEstop);
  server.on("/api/state",HTTP_GET,onState);
  server.on("/api/config/flip",HTTP_POST,onFlipConfig);
  server.on("/api/config/tof",HTTP_POST,onTofConfig);
  server.on("/api/controller/mode",HTTP_POST,onControllerMode);
  server.on("/api/controller/input",HTTP_POST,onControllerInput);
  server.onNotFound([](){server.sendHeader("Location","/mobile",true);server.send(302,"text/plain","Redirecting");});
  server.begin();
}

void setupMotorPins(){
  pinMode(M_LF_IN1, OUTPUT); pinMode(M_LF_IN2, OUTPUT);
  pinMode(M_RF_IN1, OUTPUT); pinMode(M_RF_IN2, OUTPUT);
  pinMode(M_LR_IN1, OUTPUT); pinMode(M_LR_IN2, OUTPUT);
  pinMode(M_RR_IN1, OUTPUT); pinMode(M_RR_IN2, OUTPUT);

  ledcAttach(M_LF_EN, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(M_RF_EN, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(M_LR_EN, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(M_RR_EN, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(BUZZER_PIN, BUZZER_FREQ, 8);

  stopCar();
  hornSet(false);
}

void setupRgb(){ rgb.begin(); rgb.setBrightness(70); setRgbRBG(0,20,80); }

void setup(){
  Serial.begin(115200);
  Wire.begin(); // requested explicit wire begin

  setupMotorPins();
  setupRgb();

  if(!lox.begin()){Serial.println("ToF init failed, sensor disabled.");tofEnabled=false;}

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  IPAddress ip=WiFi.softAPIP();
  if(MDNS.begin("car")) MDNS.addService("http","tcp",80);

  lastPingMs=millis();
  setupServer();

  Serial.print("AP IP: "); Serial.println(ip);
  Serial.println("Open: http://car.local/mobile");
}

void loop(){
  server.handleClient();
  updateTof();
  evaluateControllerDrive();
  updateRgb();
  handleFailsafe();
}
