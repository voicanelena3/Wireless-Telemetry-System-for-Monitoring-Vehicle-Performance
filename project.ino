#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MLX90614.h>

const int STBY = 27;
const int AIN1 = 26;
const int AIN2 = 25;
const int PWMA = 33;
const int BIN1 = 32;
const int BIN2 = 14;
const int PWMB = 13;

const int SENZOR_STANGA = 34;
const int SENZOR_DREAPTA = 35;
const int GAURI_DISC = 20;             
const float DIAMETRU_ROATA_CM = 6.5;  /
const bool DEBUG_SERIAL = true;    

const char* ssid = "MasinaTelemetrie";
const char* password = "12345678";

WebServer server(80);
Adafruit_MPU6050 mpu;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

volatile float accX = 0, accY = 0, accZ = 0;
volatile float gyroX = 0, gyroY = 0, gyroZ = 0;
volatile float tempObj = 0, tempAmbient = 0;
bool mpuOk = false;
bool mlxOk = false;               
volatile bool mlxValid = false;   

volatile unsigned long pulsStanga = 0;
volatile unsigned long pulsDreapta = 0;
volatile unsigned long totalStanga = 0;
volatile unsigned long totalDreapta = 0;
volatile unsigned long lastPulsL = 0;
volatile unsigned long lastPulsR = 0;
volatile float vitezaStanga = 0, vitezaDreapta = 0;   // rotatii pe secunda
volatile float vitezaKmh = 0;
unsigned long lastCalcTime = 0;

// Oprire de siguranta: fara comanda in 400 ms -> motoarele se opresc
volatile unsigned long lastControlTime = 0;
volatile bool motoarePornite = false;
const unsigned long TIMEOUT_COMANDA_MS = 400;

// ISR cu filtrare: ignora pulsurile mai rapide de 3 ms (zgomot)
void IRAM_ATTR onPulsStanga() {
  unsigned long t = micros();
  if (t - lastPulsL > 3000) {
    pulsStanga++;
    totalStanga++;
    lastPulsL = t;
  }
}

void IRAM_ATTR onPulsDreapta() {
  unsigned long t = micros();
  if (t - lastPulsR > 3000) {
    pulsDreapta++;
    totalDreapta++;
    lastPulsR = t;
  }
}

void motorStanga(int speed) {
  speed = constrain(speed, -255, 255);
  digitalWrite(AIN1, speed >= 0 ? HIGH : LOW);
  digitalWrite(AIN2, speed >= 0 ? LOW : HIGH);
  ledcWrite(PWMA, abs(speed));
}

void motorDreapta(int speed) {
  speed = constrain(speed, -255, 255);
  digitalWrite(BIN1, speed >= 0 ? HIGH : LOW);
  digitalWrite(BIN2, speed >= 0 ? LOW : HIGH);
  ledcWrite(PWMB, abs(speed));
}

void opresteMotoare() {
  motorStanga(0);
  motorDreapta(0);
  motoarePornite = false;
}

const char htmlPage[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta http-equiv="Content-Type" content="text/html; charset=utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
<title>Telemetry System</title>
<style>
  * { box-sizing: border-box; -webkit-tap-highlight-color: transparent; }
  html { background:#0a0a0a; color-scheme: dark; }
  body {
    margin:0; height:100vh; overflow:hidden; touch-action:none;
    background-color:#0a0a0a;
    background-image:
      repeating-linear-gradient(45deg, rgba(255,255,255,0.015) 0px, rgba(255,255,255,0.015) 2px, transparent 2px, transparent 8px),
      radial-gradient(circle at 50% 30%, #1a1a1a 0%, #0a0a0a 100%);
    font-family: 'Segoe UI', Roboto, sans-serif; color:#fff;
  }
  #topbar {
    position:fixed; top:0; left:0; width:100%; padding:14px 20px;
    display:flex; justify-content:space-between; align-items:center;
    background: linear-gradient(180deg, rgba(0,0,0,0.8), transparent);
    z-index:10;
  }
  #logo { font-size:15px; font-weight:700; letter-spacing:2px; color:#e10600; }
  #status-dot { width:8px; height:8px; border-radius:50%; background:#666; display:inline-block; margin-left:8px; }
  #status-dot.ok { background:#2ecc40; box-shadow:0 0 8px #2ecc40; }
  #status-dot.bad { background:#e10600; box-shadow:0 0 8px #e10600; }

  #speedometer {
    position:fixed; top:70px; left:50%; transform:translateX(-50%);
    text-align:center; z-index:5;
  }
  #speedVal { font-size:56px; font-weight:800; font-variant-numeric: tabular-nums; line-height:1; text-shadow:0 0 20px rgba(225,6,0,0.5); }
  #speedUnit { font-size:12px; letter-spacing:3px; color:#888; margin-top:2px; }
  #gear { font-size:13px; color:#e10600; letter-spacing:2px; margin-top:6px; font-weight:600; }
  #throttle { font-size:11px; color:#888; letter-spacing:2px; margin-top:4px; }

  #telemetryLeft, #telemetryRight {
    position:fixed; top:50%; transform:translateY(-50%);
    font-size:11px; color:#999; letter-spacing:1px; line-height:1.8;
    background:rgba(0,0,0,0.3); padding:10px 12px; border-radius:8px;
    border:1px solid #2a2a2a; z-index:5;
  }
  #telemetryLeft { left:10px; }
  #telemetryRight { right:10px; text-align:right; }
  .tval { color:#fff; font-weight:700; font-variant-numeric: tabular-nums; }
  .tlabel { color:#e10600; font-size:9px; letter-spacing:2px; display:block; margin-top:6px; }

  #tempBox {
    position:fixed; top:225px; left:50%; transform:translateX(-50%);
    text-align:center; z-index:5;
    background:rgba(0,0,0,0.3); padding:8px 16px; border-radius:8px;
    border:1px solid #2a2a2a; min-width:110px;
  }
  #tempVal { font-size:22px; font-weight:800; color:#ff8800; }
  #tempVal.err { color:#e10600; }
  #tempLabel { font-size:9px; letter-spacing:2px; color:#666; }

  #wheelSpeeds {
    position:fixed; bottom:10px; left:50%; transform:translateX(-50%);
    display:flex; gap:20px; z-index:5;
  }
  .wsBox {
    background:rgba(0,0,0,0.3); padding:6px 14px; border-radius:8px;
    border:1px solid #2a2a2a; text-align:center; min-width:110px;
  }
  .wsVal { font-size:16px; font-weight:800; color:#4ade80; font-variant-numeric: tabular-nums; }
  .wsLabel { font-size:8px; letter-spacing:1px; color:#666; }
  .wsPuls { font-size:9px; color:#888; margin-top:2px; }

  #padZone {
    position:fixed; left:0; bottom:0; width:100%; height:65%;
    display:flex; align-items:center; justify-content:center;
  }
  #padBase {
    width:220px; height:220px; border-radius:50%;
    background: radial-gradient(circle, #1c1c1c, #0d0d0d);
    border: 2px solid #333;
    box-shadow: inset 0 0 30px rgba(0,0,0,0.8), 0 0 20px rgba(225,6,0,0.15);
    position:relative;
  }
  #padBase::before {
    content:''; position:absolute; top:50%; left:8px; right:8px; height:1px; background:rgba(255,255,255,0.08); transform:translateY(-50%);
  }
  #padBase::after {
    content:''; position:absolute; left:50%; top:8px; bottom:8px; width:1px; background:rgba(255,255,255,0.08); transform:translateX(-50%);
  }
  #padStick {
    width:80px; height:80px; border-radius:50%;
    background: linear-gradient(145deg, #2a2a2a, #111);
    border: 2px solid #e10600;
    position:absolute; left:50%; top:50%; transform:translate(-50%,-50%);
    box-shadow: 0 0 16px rgba(225,6,0,0.6);
  }
  #padLabel { position:absolute; bottom:-30px; left:50%; transform:translateX(-50%); font-size:11px; letter-spacing:2px; color:#666; white-space:nowrap; }
</style>
</head>
<body>

<div id="topbar">
  <div id="logo">TELEMETRY<span id="status-dot"></span></div>
  <div style="font-size:11px;color:#666;letter-spacing:1px">ESP32 &middot; LIVE</div>
</div>

<div id="speedometer">
  <div id="speedVal">0.0</div>
  <div id="speedUnit">KM/H</div>
  <div id="gear">N</div>
  <div id="throttle">THROTTLE 0%</div>
</div>

<div id="tempBox">
  <div id="tempVal">--</div>
  <div id="tempLabel">TIRE TEMP</div>
</div>

<div id="telemetryLeft">
  <span class="tlabel">ACCEL X</span><span class="tval" id="ax">0.00</span>
  <span class="tlabel">ACCEL Y</span><span class="tval" id="ay">0.00</span>
  <span class="tlabel">ACCEL Z</span><span class="tval" id="az">0.00</span>
</div>

<div id="telemetryRight">
  <span class="tlabel">GYRO X</span><span class="tval" id="gx">0.00</span>
  <span class="tlabel">GYRO Y</span><span class="tval" id="gy">0.00</span>
  <span class="tlabel">GYRO Z</span><span class="tval" id="gz">0.00</span>
</div>

<div id="wheelSpeeds">
  <div class="wsBox">
    <div class="wsVal" id="wsL">0.0</div>
    <div class="wsLabel">WHEEL L (RPS)</div>
    <div class="wsPuls">pulses: <span id="pL">0</span></div>
  </div>
  <div class="wsBox">
    <div class="wsVal" id="wsR">0.0</div>
    <div class="wsLabel">WHEEL R (RPS)</div>
    <div class="wsPuls">pulses: <span id="pR">0</span></div>
  </div>
</div>

<div id="padZone">
  <div id="padBase">
    <div id="padStick"></div>
    <div id="padLabel">DRIVE</div>
  </div>
</div>

<script>
const padBase = document.getElementById('padBase');
const padStick = document.getElementById('padStick');
const throttleEl = document.getElementById('throttle');
const gearLabel = document.getElementById('gear');
const dot = document.getElementById('status-dot');
const tempEl = document.getElementById('tempVal');

let active = false;
let rawX = 0, rawY = 0;
let smX = 0, smY = 0;
let sendTimer = null, smoothTimer = null;
let cmdBusy = false;

const SMOOTH_FACTOR = 0.2;

function startLoop() {
  if (!sendTimer) sendTimer = setInterval(sendCommand, 80);
  if (!smoothTimer) smoothTimer = setInterval(smoothStep, 30);
}

function sendStop() {
  for (let i = 0; i < 3; i++) {
    setTimeout(() => {
      fetch('/control?l=0&r=0&t=' + Date.now(), {cache:'no-store'}).catch(() => {});
    }, i * 100);
  }
}

function releasePad() {
  active = false;
  rawX = 0; rawY = 0; smX = 0; smY = 0;
  clearInterval(sendTimer); sendTimer = null;
  clearInterval(smoothTimer); smoothTimer = null;
  padStick.style.transform = 'translate(-50%,-50%)';
  throttleEl.textContent = 'THROTTLE 0%';
  gearLabel.textContent = 'N';
  sendStop();
}

function smoothStep() {
  smX += (rawX - smX) * SMOOTH_FACTOR;
  smY += (rawY - smY) * SMOOTH_FACTOR;
  const maxDist = 70;
  padStick.style.transform = 'translate(calc(-50% + ' + ((smX/255)*maxDist) + 'px), calc(-50% + ' + ((-smY/255)*maxDist) + 'px))';
  const mag = Math.min(1, Math.sqrt(smX*smX + smY*smY) / 255);
  throttleEl.textContent = 'THROTTLE ' + Math.round(mag * 100) + '%';
  if (Math.abs(smY) < 8 && Math.abs(smX) > 8) gearLabel.textContent = 'TURN';
  else if (smY > 8) gearLabel.textContent = 'FWD';
  else if (smY < -8) gearLabel.textContent = 'REV';
  else gearLabel.textContent = 'N';
}

function updatePad(clientX, clientY) {
  const r = padBase.getBoundingClientRect();
  const cx = r.left + r.width/2;
  const cy = r.top + r.height/2;
  const maxDist = r.width/2 - 40;
  let dx = clientX - cx;
  let dy = clientY - cy;
  const dist = Math.min(Math.sqrt(dx*dx+dy*dy), maxDist);
  const angle = Math.atan2(dy, dx);
  dx = Math.cos(angle) * dist;
  dy = Math.sin(angle) * dist;
  const nx = dx / maxDist;
  const ny = -dy / maxDist;
  const curvedX = Math.sign(nx) * Math.pow(Math.abs(nx), 1.5);
  const curvedY = Math.sign(ny) * Math.pow(Math.abs(ny), 1.3);
  rawX = Math.round(curvedX * 255);
  rawY = Math.round(curvedY * 255);
}

function sendCommand() {
  if (!active || cmdBusy) return;
  let left = smY + smX;
  let right = smY - smX;
  left = Math.max(-255, Math.min(255, Math.round(left)));
  right = Math.max(-255, Math.min(255, Math.round(right)));
  cmdBusy = true;
  fetch('/control?l=' + left + '&r=' + right + '&t=' + Date.now(), {cache:'no-store'})
    .catch(() => {})
    .finally(() => { cmdBusy = false; });
}

padBase.addEventListener('touchstart', e => { active = true; const t=e.touches[0]; updatePad(t.clientX,t.clientY); startLoop(); e.preventDefault(); }, {passive:false});
padBase.addEventListener('touchmove', e => { if(active){ const t=e.touches[0]; updatePad(t.clientX,t.clientY);} e.preventDefault(); }, {passive:false});
padBase.addEventListener('touchend', e => { releasePad(); e.preventDefault(); }, {passive:false});
padBase.addEventListener('touchcancel', e => { releasePad(); });

padBase.addEventListener('mousedown', e => { active = true; updatePad(e.clientX,e.clientY); startLoop(); });
window.addEventListener('mousemove', e => { if(active) updatePad(e.clientX,e.clientY); });
window.addEventListener('mouseup', e => { if(active) releasePad(); });

window.addEventListener('blur', () => { if(active) releasePad(); });
document.addEventListener('visibilitychange', () => { if(document.hidden && active) releasePad(); });

function setText(id, txt) { document.getElementById(id).textContent = txt; }

function pollTelemetry() {
  fetch('/telemetry?t=' + Date.now(), {cache:'no-store'})
    .then(r => r.json())
    .then(d => {
      setText('ax', d.ax.toFixed(2));
      setText('ay', d.ay.toFixed(2));
      setText('az', d.az.toFixed(2));
      setText('gx', d.gx.toFixed(2));
      setText('gy', d.gy.toFixed(2));
      setText('gz', d.gz.toFixed(2));
      if (d.mlxOk) {
        tempEl.textContent = d.tempObj.toFixed(1) + '\u00B0C';
        tempEl.className = '';
      } else {
        tempEl.textContent = 'ERR';
        tempEl.className = 'err';
      }
      setText('wsL', d.vL.toFixed(1));
      setText('wsR', d.vR.toFixed(1));
      setText('pL', d.pL);
      setText('pR', d.pR);
      setText('speedVal', d.kmh.toFixed(1)); // Aici se actualizeaza vitezometrul central
      dot.className = 'ok';
    })
    .catch(() => { dot.className = 'bad'; })
    .finally(() => { setTimeout(pollTelemetry, 500); });
}
pollTelemetry();
</script>
</body>
</html>
)rawliteral";

void handleRoot() {
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "text/html; charset=utf-8", htmlPage);
}

void handleControl() {
  int l = server.arg("l").toInt();
  int r = server.arg("r").toInt();
  lastControlTime = millis();          
  motorStanga(l);
  motorDreapta(r);
  motoarePornite = (l != 0 || r != 0);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", "OK");
}

float safeF(float v) {
  return (isnan(v) || isinf(v)) ? 0.0 : v;
}

void handleTelemetry() {
  String json = "{";
  json += "\"ax\":" + String(safeF(accX), 3) + ",";
  json += "\"ay\":" + String(safeF(accY), 3) + ",";
  json += "\"az\":" + String(safeF(accZ), 3) + ",";
  json += "\"gx\":" + String(safeF(gyroX), 3) + ",";
  json += "\"gy\":" + String(safeF(gyroY), 3) + ",";
  json += "\"gz\":" + String(safeF(gyroZ), 3) + ",";
  json += "\"tempObj\":" + String(safeF(tempObj), 2) + ",";
  json += "\"tempAmbient\":" + String(safeF(tempAmbient), 2) + ",";
  json += "\"mlxOk\":" + String(mlxValid ? 1 : 0) + ",";
  json += "\"vL\":" + String(safeF(vitezaStanga), 3) + ",";
  json += "\"vR\":" + String(safeF(vitezaDreapta), 3) + ",";
  json += "\"kmh\":" + String(safeF(vitezaKmh), 2) + ",";
  json += "\"pL\":" + String(totalStanga) + ",";
  json += "\"pR\":" + String(totalDreapta);
  json += "}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

void safetyTask(void* p) {
  for (;;) {
    if (motoarePornite && (millis() - lastControlTime > TIMEOUT_COMANDA_MS)) {
      opresteMotoare();
      Serial.println("SAFETY: nicio comanda -> motoare oprite");
    }
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

void serverTask(void* p) {
  for (;;) {
    server.handleClient();
    vTaskDelay(1);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(STBY, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  ledcAttach(PWMA, 5000, 8);
  ledcAttach(PWMB, 5000, 8);
  digitalWrite(STBY, HIGH);
  opresteMotoare();

  pinMode(SENZOR_STANGA, INPUT);
  pinMode(SENZOR_DREAPTA, INPUT);
  attachInterrupt(digitalPinToInterrupt(SENZOR_STANGA), onPulsStanga, FALLING);
  attachInterrupt(digitalPinToInterrupt(SENZOR_DREAPTA), onPulsDreapta, FALLING);

  Wire.begin(21, 22);     // SDA, SCL - bus comun MPU6050 + MLX90614
  Wire.setClock(50000);   // 50 kHz pentru stabilitate I2C

  mpuOk = mpu.begin();
  if (mpuOk) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("MPU6050 initializat cu succes.");
  } else {
    Serial.println("MPU6050 NU a fost gasit!");
  }

  mlxOk = mlx.begin();
  if (mlxOk) {
    Serial.println("MLX90614 initializat cu succes.");
  } else {
    Serial.println("MLX90614 NU a fost gasit!");
  }

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);   
  WiFi.softAP(ssid, password);
  Serial.print("WiFi creat. Conecteaza-te la: ");
  Serial.println(ssid);
  Serial.print("Adresa IP: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/control", handleControl);
  server.on("/telemetry", handleTelemetry);
  server.begin();

  lastControlTime = millis();
  lastCalcTime = millis();

  xTaskCreatePinnedToCore(safetyTask, "safety", 3072, NULL, 3, NULL, 0);  
  xTaskCreatePinnedToCore(serverTask, "web",    8192, NULL, 1, NULL, 1);
}

unsigned long lastMpuRead = 0;
unsigned long lastMlxRead = 0;
unsigned long lastMlxProbe = 0;
unsigned long lastDbg = 0;
int mlxErrors = 0;

bool mlxPrezent() {
  Wire.beginTransmission(0x5A);
  return Wire.endTransmission() == 0;
}

void loop() {
  // Citire MPU6050 - 10 Hz
  if (mpuOk && millis() - lastMpuRead > 100) {
    lastMpuRead = millis();
    sensors_event_t a, g, temp;
    if (mpu.getEvent(&a, &g, &temp)) {
      accX = a.acceleration.x;
      accY = a.acceleration.y;
      accZ = a.acceleration.z;
      gyroX = g.gyro.x;
      gyroY = g.gyro.y;
      gyroZ = g.gyro.z;
    }
  }

  // Citire MLX90614 - ~3 Hz
  if (mlxOk) {
    if (millis() - lastMlxRead > 300) {
      lastMlxRead = millis();
      float o = mlx.readObjectTempC();
      float a = mlx.readAmbientTempC();

      if (isnan(o) || isnan(a) || isinf(o) || isinf(a)) {
        mlxValid = false;
        mlxErrors++;
        if (mlxErrors >= 5) {   
          Wire.end();
          Wire.begin(21, 22);
          Wire.setClock(50000);
          mlxOk = mlx.begin();
          mlxErrors = 0;
        }
      } else {
        mlxValid = true;
        mlxErrors = 0;
        tempObj = o;
        tempAmbient = a;
      }
    }
  } else {
    mlxValid = false;
    if (millis() - lastMlxProbe > 2000) {
      lastMlxProbe = millis();
      if (mlxPrezent()) {
        mlxOk = mlx.begin();
      }
    }
  }

  // Calcul vitezometru roti - la fiecare 500 ms
  unsigned long now = millis();
  if (now - lastCalcTime >= 500) {
    float interval = (now - lastCalcTime) / 1000.0;

    noInterrupts();
    unsigned long ps = pulsStanga;
    unsigned long pd = pulsDreapta;
    pulsStanga = 0;
    pulsDreapta = 0;
    interrupts();

    vitezaStanga = (ps / (float)GAURI_DISC) / interval;    // rotatii/secunda
    vitezaDreapta = (pd / (float)GAURI_DISC) / interval;

    float rpsMediu = (vitezaStanga + vitezaDreapta) / 2.0;
    float circumferintaM = 3.14159265 * (DIAMETRU_ROATA_CM / 100.0);
    vitezaKmh = rpsMediu * circumferintaM * 3.6; // Vitezometrul calculat in km/h

    lastCalcTime = now;
  }

  if (DEBUG_SERIAL && millis() - lastDbg > 500) {
    lastDbg = millis();
    Serial.printf("Viteza: %.2f km/h | accZ=%.2f | MLX obj=%.2f C | Puls L=%lu R=%lu\n",
                  (float)vitezaKmh, (float)accZ, (float)tempObj, totalStanga, totalDreapta);
  }

  delay(2);
}
