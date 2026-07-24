/*
  DUAL-AXIS SOLAR TRACKER + POWER LOGGER + WIFI CONTROL PANEL
  Arduino Nano ESP32

  Overview:
  Four light sensors behind a "+" shade wall detect which side the sun is on; two
  servos rotate the panel until the readings balance. An INA219 measures the panel's
  output power. A colour TFT shows a live status board. The ESP32 hosts its own WiFi
  network and serves a web dashboard that both displays the data and controls the
  tracker (mode selection, manual servo control, and live tuning).

  Wiring:
    sensors TL/TR/BL/BR -> A0/A1/A2/A3   (3.3 V reference)
    pan servo -> D2   tilt servo -> D3   (5 V supply, common ground)
    INA219  SDA -> A4   SCL -> A5         (I2C)
    ST7735  SCK -> D13  MOSI -> D11  CS -> D10  DC -> D9  RES -> D8  BL -> D7   (SPI)

  Access: connect to WiFi network "SolarTracker" (password track1234),
  then open http://192.168.4.1
*/

// Hardware and networking libraries. WiFi and WebServer are part of the ESP32 core
// and require no separate installation.
#include <string.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <Adafruit_INA219.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WebServer.h>

// Light sensors on analog pins A0-A3, mapped to the panel corners.
const int PIN_TL = A0, PIN_TR = A1, PIN_BL = A2, PIN_BR = A3;
// Servo signal pins: pan (left/right) and tilt (up/down).
const int PAN_PIN = D2, TILT_PIN = D3;
// TFT control pins. Clock and data use the hardware SPI pins (D13/D11).
#define TFT_CS D10
#define TFT_DC D9
#define TFT_RST D8
#define TFT_BL D7
// Display controller variant. If the screen shows a picture but the colours are
// wrong or the image is shifted, change this to INITR_GREENTAB or INITR_REDTAB.
#define TFT_TAB INITR_BLACKTAB

// Hardware interface objects for the display, power sensor, and two servos.
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
Adafruit_INA219 ina219;
Servo panServo, tiltServo;

// Access-point credentials. The device creates its own network, so no router or
// internet connection is needed. WPA2 requires a password of at least 8 characters.
const char *AP_SSID = "SolarTracker";
const char *AP_PASS = "track1234";
// Web server on port 80 (the default HTTP port, so no port is needed in the URL).
WebServer server(80);

// Fixed operating limits: loop pacing, safe servo angle range, axis direction signs,
// stall threshold, overnight park position, and screen refresh interval.
const int SETTLE_MS = 20, ANGLE_MIN = 10, ANGLE_MAX = 170, PAN_DIR = +1, TILT_DIR = +1;
const int STALL_CYCLES = 40, PARK_PAN = ANGLE_MIN, PARK_TILT = 60;
const unsigned long DRAW_EVERY = 400;
const unsigned long SERIAL_EVERY = 1000; // status line interval on the serial monitor

// Runtime-adjustable parameters (variables, not const) so they can be changed live
// from the dashboard. The values below are the startup defaults.
int deadzone = 60;   // minimum light difference required before the panel moves
int stepDeg = 1;     // degrees moved per loop (smaller = smoother)
int darkLevel = 350; // average reading below this is treated as night
int smoothPct = 30;  // sensor smoothing strength, percent (lower = calmer)
int fixedPan = 90;   // hold angles used during the fixed-panel comparison
int fixedTilt = 90;

// Operating modes. AUTO follows the sun, FIXEDMODE holds the panel flat for
// comparison, MANUAL applies servo positions sent from the dashboard.
enum Mode
{
  AUTO,
  FIXEDMODE,
  MANUAL
};
Mode mode = AUTO;
int manualPan = 90, manualTilt = 90; // target angles set by the dashboard in MANUAL
bool isNight = false;                // set true when AUTO detects darkness
const char *stateLabel = "TRACKING"; // text shown on the screen and dashboard

// Values updated every loop. Each sensor is kept in two forms: the raw pin reading
// and a smoothed value. Steering uses the smoothed values to avoid jitter.
int panAngle = 90, tiltAngle = 90;
int rawTL, rawTR, rawBL, rawBR;       // raw pin readings
float tl = 0, tr = 0, bl = 0, br = 0; // smoothed readings (used for steering)
int hErr, vErr, panStall = 0, tiltStall = 0;
bool panStalled = false, tiltStalled = false;

// Power readings and the two comparison buckets. Energy accumulates into the bucket
// matching the active mode; the elapsed-time totals allow average power (and the
// percentage gain of tracking over fixed) to be calculated.
float busV = 0, current_mA = 0, power_mW = 0;
bool inaOK = false;          // true only when the INA219 answered on the I2C bus
char i2cList[64] = "";       // addresses found on the I2C bus, shown on the dashboard
bool servosAttached = false; // true if both servo objects attached successfully
double energyTrack_mWh = 0, energyFixed_mWh = 0, msTrack = 0, msFixed = 0;
unsigned long lastEnergyMs = 0, lastDrawMs = 0, lastSerialMs = 0;
bool layoutDrawn = false; // false = redraw the static screen layout on the next refresh

// Function prototypes (definitions appear below).
void readSensors();
void updateDisplay();
void i2cScan();
void serialStatus();
void drawLayout();
uint16_t stateColor();
float gainPct();
const char *modeName();
void handleRoot();
void handleData();
void handleMode();
void handleMove();
void handleSet();
void handleReset();
void handleServoTest();

// Web dashboard (HTML, CSS, and JavaScript) stored as a single text block in flash
// and served directly from the device, so it works without internet access. The
// script polls /data once per second to refresh the readings and graph, and sends
// requests to /mode, /move, /set, and /reset when the controls are used.
const char PAGE[] = R"HTML(
<!doctype html><html><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1"><title>Solar Tracker</title>
<style>
*{box-sizing:border-box}body{font-family:system-ui,Segoe UI,Roboto,sans-serif;
background:linear-gradient(160deg,#0a0e15,#0f1722);color:#e6eef7;margin:0 auto;padding:18px;max-width:560px}
h1{font-size:20px;margin:0;display:flex;align-items:center;gap:8px}
.sub{color:#6f8398;font-size:12px;margin:2px 0 14px}
.badge{font-size:12px;font-weight:800;padding:4px 10px;border-radius:20px;color:#06222a}
.card{background:#121b27;border:1px solid #1e2c3c;border-radius:14px;padding:16px;margin:12px 0;box-shadow:0 4px 14px rgba(0,0,0,.25)}
.label{color:#7d92a6;font-size:12px;text-transform:uppercase;letter-spacing:.5px}
.big{font-size:40px;font-weight:800;color:#39d98a;line-height:1.1}.unit{font-size:16px;color:#7d92a6;font-weight:600}
.gain{font-size:30px;font-weight:800;color:#f5b301}
.row{display:flex;justify-content:space-between;align-items:center;margin:7px 0;font-size:14px}.k{color:#7d92a6}.v{font-weight:600}
canvas{width:100%;height:90px;display:block;margin-top:10px}
.modes{display:flex;gap:8px}.modes button{flex:1;padding:11px;border:1px solid #27384b;border-radius:10px;background:#16212f;color:#cfe0f0;font-weight:700;font-size:14px}
.modes button.on{background:#39d98a;color:#06222a;border-color:#39d98a}
.slider{margin:12px 0}.slider .top{display:flex;justify-content:space-between;font-size:13px;margin-bottom:4px}
input[type=range]{width:100%;accent-color:#39d98a}
.btn{width:100%;padding:12px;border:0;border-radius:10px;background:#27384b;color:#cfe0f0;font-weight:700;font-size:14px;margin-top:6px}
.hide{display:none}.grid{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:8px}
.stat{background:#0e1722;border:1px solid #1c2a3a;border-radius:10px;padding:10px}.stat .n{font-size:18px;font-weight:700}
</style></head><body>
<h1>&#9728;&#65039; Solar Tracker <span id=badge class=badge>--</span></h1>
<div class=sub>live control panel &middot; 192.168.4.1</div>

<div class=card><div class=label>Power now</div>
<div class=big id=power>--<span class=unit> mW</span></div>
<canvas id=spark width=520 height=90></canvas></div>

<div class=card>
<div class=modes>
<button id=mAuto onclick="setMode('auto')">Auto</button>
<button id=mFixed onclick="setMode('fixed')">Fixed</button>
<button id=mManual onclick="setMode('manual')">Manual</button></div>
<div id=manual class=hide>
<div class=slider><div class=top><span class=k>Pan</span><span id=mpL>90&deg;</span></div>
<input type=range id=mp min=10 max=170 value=90 oninput="move()"></div>
<div class=slider><div class=top><span class=k>Tilt</span><span id=mtL>90&deg;</span></div>
<input type=range id=mt min=10 max=170 value=90 oninput="move()"></div></div></div>

<div class=card><div class=label>Tracking vs Fixed</div>
<div class=gain id=gain>--</div>
<div class=grid>
<div class=stat><div class=k>Track avg</div><div class=n id=pT>--</div></div>
<div class=stat><div class=k>Fixed avg</div><div class=n id=pF>--</div></div>
<div class=stat><div class=k>Energy track</div><div class=n id=eT>--</div></div>
<div class=stat><div class=k>Energy fixed</div><div class=n id=eF>--</div></div></div>
<button class=btn onclick="reset()">Reset comparison</button></div>

<div class=card>
<div class=row><span class=k>State</span><span class=v id=state>--</span></div>
<div class=row><span class=k>Voltage</span><span class=v id=volts>--</span></div>
<div class=row><span class=k>Pan / Tilt</span><span class=v id=ang>--</span></div>
<div class=row><span class=k>Sensors TL TR BL BR</span><span class=v id=sens>--</span></div></div>

<div class=card><div class=label>Diagnostics</div>
<div class=row><span class=k>I2C devices found</span><span class=v id=i2c>--</span></div>
<div class=row><span class=k>INA219</span><span class=v id=ina>--</span></div>
<div class=row><span class=k>Servos attached</span><span class=v id=sv>--</span></div>
<button class=btn onclick="servoTest()">Test servos now (forced sweep)</button></div>

<div class=card><div class=label>Tuning (live)</div>
<div class=slider><div class=top><span class=k>Deadzone</span><span id=dzL>--</span></div>
<input type=range id=dz min=5 max=300 oninput="setv('deadzone','dz','dzL','')"></div>
<div class=slider><div class=top><span class=k>Step size</span><span id=stL>--</span></div>
<input type=range id=st min=1 max=5 oninput="setv('step','st','stL','')"></div>
<div class=slider><div class=top><span class=k>Smoothing</span><span id=smL>--</span></div>
<input type=range id=sm min=5 max=90 oninput="setv('smooth','sm','smL','%')"></div>
<div class=slider><div class=top><span class=k>Night level</span><span id=dkL>--</span></div>
<input type=range id=dk min=0 max=2000 oninput="setv('dark','dk','dkL','')"></div>
<div class=slider><div class=top><span class=k>Fixed pan</span><span id=fpL>--</span></div>
<input type=range id=fp min=10 max=170 oninput="setv('fpan','fp','fpL','&deg;')"></div>
<div class=slider><div class=top><span class=k>Fixed tilt</span><span id=ftL>--</span></div>
<input type=range id=ft min=10 max=170 oninput="setv('ftilt','ft','ftL','&deg;')"></div></div>

<script>
let hist=[],first=true,last={},$=id=>document.getElementById(id);
const bc={TRACKING:'#39d98a',NIGHT:'#6aa6ff',FIXED:'#f5b301',MANUAL:'#ff7ac6'};
async function tick(){try{
let d=await(await fetch('/data')).json();last=d;
$('power').innerHTML=d.power.toFixed(1)+'<span class=unit> mW</span>';
$('badge').textContent=d.state;$('badge').style.background=bc[d.state]||'#7d92a6';
$('gain').textContent=(d.gain>=0?'+':'')+d.gain.toFixed(1)+'%';
$('pT').textContent=d.pTrack.toFixed(1)+' mW';$('pF').textContent=d.pFixed.toFixed(1)+' mW';
$('eT').textContent=d.eTrack.toFixed(2)+' mWh';$('eF').textContent=d.eFixed.toFixed(2)+' mWh';
$('state').textContent=d.state+(d.stall?'  (stall)':'');
$('volts').textContent=d.volts.toFixed(2)+' V';$('ang').textContent=d.pan+'\u00b0 / '+d.tilt+'\u00b0';
$('sens').textContent=d.s.join('  ');
$('i2c').textContent=d.i2c;
$('ina').textContent=d.ina?'FOUND':'NOT FOUND';$('ina').style.color=d.ina?'#39d98a':'#ff6b6b';
$('sv').textContent=d.sv?'yes':'NO';$('sv').style.color=d.sv?'#39d98a':'#ff6b6b';
['mAuto','mFixed','mManual'].forEach(x=>$(x).classList.remove('on'));
$({auto:'mAuto',fixed:'mFixed',manual:'mManual'}[d.mode]).classList.add('on');
$('manual').classList.toggle('hide',d.mode!='manual');
if(first){
 $('dz').value=d.deadzone;$('dzL').textContent=d.deadzone;
 $('st').value=d.step;$('stL').textContent=d.step;
 $('sm').value=d.smooth;$('smL').textContent=d.smooth+'%';
 $('dk').value=d.dark;$('dkL').textContent=d.dark;
 $('fp').value=d.fpan;$('fpL').innerHTML=d.fpan+'&deg;';
 $('ft').value=d.ftilt;$('ftL').innerHTML=d.ftilt+'&deg;';
 first=false;}
hist.push(d.power);if(hist.length>120)hist.shift();draw();
}catch(e){}}
function draw(){let c=$('spark'),x=c.getContext('2d'),w=c.width,h=c.height;
x.clearRect(0,0,w,h);let m=Math.max(1,...hist);x.strokeStyle='#39d98a';x.lineWidth=2;x.beginPath();
hist.forEach((v,i)=>{let px=i/119*w,py=h-(v/m)*(h-8)-4;i?x.lineTo(px,py):x.moveTo(px,py);});x.stroke();}
function setMode(m){fetch('/mode?m='+m);
 if(m=='manual'&&last.pan!=null){$('mp').value=last.pan;$('mt').value=last.tilt;
  $('mpL').innerHTML=last.pan+'&deg;';$('mtL').innerHTML=last.tilt+'&deg;';}}
function move(){let p=$('mp').value,t=$('mt').value;
 $('mpL').innerHTML=p+'&deg;';$('mtL').innerHTML=t+'&deg;';fetch('/move?pan='+p+'&tilt='+t);}
function setv(key,sid,lid,suf){let v=$(sid).value;$(lid).innerHTML=v+suf;fetch('/set?k='+key+'&v='+v);}
function reset(){fetch('/reset');}
function servoTest(){fetch('/servotest');}
setInterval(tick,1000);tick();
</script></body></html>
)HTML";

void setup()
{
  Serial.begin(115200); // serial output for debugging
  delay(3000);

  Serial.println("Booting up...");
  analogReadResolution(12); // 12-bit ADC range (0-4095)
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH); // enable display backlight

  Wire.begin(); // start I2C for the INA219
  i2cScan();    // list every device answering, so a wiring fault is obvious
  inaOK = ina219.begin();
  if (inaOK)
  {
    // The library defaults to its widest range (32 V / 2 A), where the power
    // reading moves in 2 mW steps -- small panel outputs round down to zero.
    // This narrower range resolves roughly 1 mW and doubles current resolution.
    ina219.setCalibration_16V_400mA();
    Serial.println("INA219 ready (16V / 400mA range)");
  }
  else
    Serial.println("INA219 NOT FOUND -- check SDA on A4, SCL on A5, VCC on 3.3V");

  // initialise the display and show a splash with the connection details
  tft.initR(TFT_TAB);
  tft.setRotation(3);
  // Comms check: these three flashes prove the display is receiving data.
  // If the screen stays white through all of them it is powered but not
  // listening -- check SDA on D11, SCL on D13, CS on D10, DC on D9, RES on D8.
  Serial.println("Display test: red, green, blue...");
  tft.fillScreen(ST77XX_RED);
  delay(350);
  tft.fillScreen(ST77XX_GREEN);
  delay(350);
  tft.fillScreen(ST77XX_BLUE);
  delay(350);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setTextSize(2);
  tft.setCursor(8, 28);
  tft.print("SOLAR");
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(8, 56);
  tft.print("WiFi: ");
  tft.print(AP_SSID);
  tft.setCursor(8, 70);
  tft.print("http://192.168.4.1");

  // reserve PWM timers, attach the servos, and centre them
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  panServo.setPeriodHertz(50);
  tiltServo.setPeriodHertz(50);
  panServo.attach(PAN_PIN, 500, 2400);
  tiltServo.attach(TILT_PIN, 500, 2400);
  servosAttached = panServo.attached() && tiltServo.attached();
  Serial.println(servosAttached ? "Servos attached" : "SERVO ATTACH FAILED");
  panServo.write(panAngle);
  tiltServo.write(tiltAngle);

  // start the access point and map each URL to its handler
  WiFi.softAP(AP_SSID, AP_PASS);
  server.on("/", handleRoot);               // serves the dashboard page
  server.on("/data", handleData);           // returns live values as JSON
  server.on("/mode", handleMode);           // mode selection (Auto/Fixed/Manual)
  server.on("/move", handleMove);           // manual servo positions
  server.on("/set", handleSet);             // live tuning values
  server.on("/reset", handleReset);         // clears the comparison
  server.on("/servotest", handleServoTest); // forced servo sweep, ignores mode
  server.begin();
  layoutDrawn = false; // dashboard layout draws once on the first refresh

  lastEnergyMs = millis(); // initialise the energy timing reference
  delay(1200);
}

void loop()
{
  // Step 1: read and smooth the sensors, then compute overall brightness for day/night detection.
  readSensors();
  float avg = (tl + tr + bl + br) / 4.0;

  // Step 2: read the panel and accumulate energy (power x elapsed time) into the bucket
  // for the active mode. Manual mode is excluded from the comparison.
  if (inaOK)
  {
    busV = ina219.getBusVoltage_V();
    current_mA = ina219.getCurrent_mA();
    power_mW = ina219.getPower_mW();
    if (power_mW < 0)
      power_mW = 0;
  }

  unsigned long now = millis();
  double dms = now - lastEnergyMs;
  lastEnergyMs = now;
  if (mode == AUTO)
  {
    energyTrack_mWh += power_mW * (dms / 3600000.0);
    msTrack += dms;
  }
  else if (mode == FIXEDMODE)
  {
    energyFixed_mWh += power_mW * (dms / 3600000.0);
    msFixed += dms;
  }

  // Step 3: select the state and act on it.
  isNight = false;
  if (mode == MANUAL)
  {
    // apply the dashboard-set angles directly
    panAngle = manualPan;
    tiltAngle = manualTilt;
    stateLabel = "MANUAL";
    panStalled = tiltStalled = false;
  }
  else if (mode == FIXEDMODE)
  {
    // hold the panel flat as the fixed reference
    panAngle = fixedPan;
    tiltAngle = fixedTilt;
    stateLabel = "FIXED";
    panStalled = tiltStalled = false;
  }
  else
  { // AUTO
    if (avg < darkLevel)
    {
      // too dark to track: park aimed east and low, ready for sunrise
      panAngle = PARK_PAN;
      tiltAngle = PARK_TILT;
      stateLabel = "NIGHT";
      isNight = true;
      panStalled = tiltStalled = false;
    }
    else
    {
      // tracking: steer toward the brighter side on each axis
      stateLabel = "TRACKING";
      hErr = ((tl + bl) / 2) - ((tr + br) / 2); // left minus right
      vErr = ((tl + tr) / 2) - ((bl + br) / 2); // top minus bottom
      // move only when the difference exceeds the deadzone, otherwise hold to avoid jitter
      int panNudge = (abs(hErr) > deadzone) ? PAN_DIR * ((hErr > 0) ? stepDeg : -stepDeg) : 0;
      int tiltNudge = (abs(vErr) > deadzone) ? TILT_DIR * ((vErr > 0) ? stepDeg : -stepDeg) : 0;
      // stall guard: cancel a nudge that would push further past a reached limit, and flag a
      // sustained stall so the dashboard can report it
      if ((panAngle >= ANGLE_MAX && panNudge > 0) || (panAngle <= ANGLE_MIN && panNudge < 0))
      {
        if (++panStall > STALL_CYCLES)
          panStalled = true;
        panNudge = 0;
      }
      else
      {
        panStall = 0;
        panStalled = false;
      }
      if ((tiltAngle >= ANGLE_MAX && tiltNudge > 0) || (tiltAngle <= ANGLE_MIN && tiltNudge < 0))
      {
        if (++tiltStall > STALL_CYCLES)
          tiltStalled = true;
        tiltNudge = 0;
      }
      else
      {
        tiltStall = 0;
        tiltStalled = false;
      }
      // apply the nudge, constrained to the safe angle range
      panAngle = constrain(panAngle + panNudge, ANGLE_MIN, ANGLE_MAX);
      tiltAngle = constrain(tiltAngle + tiltNudge, ANGLE_MIN, ANGLE_MAX);
    }
  }

  panServo.write(panAngle);
  tiltServo.write(tiltAngle); // output the final angles

  // Step 4: refresh the display on its interval and service any web client.
  if (now - lastDrawMs >= DRAW_EVERY)
  {
    updateDisplay();
    lastDrawMs = now;
  }
  if (now - lastSerialMs >= SERIAL_EVERY)
  {
    serialStatus();
    lastSerialMs = now;
  }
  server.handleClient();
  delay(SETTLE_MS); // allow time for servo movement
}

// Lists every device that answers on the I2C bus. The INA219 normally appears
// at 0x40. An empty list means the bus is not wired or not powered.
void i2cScan()
{
  Serial.println("I2C scan:");
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++)
  {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0)
    {
      Serial.print("  device at 0x");
      Serial.println(addr, HEX);
      char one[10];
      snprintf(one, sizeof(one), "%s0x%02X", found ? " " : "", addr);
      strncat(i2cList, one, sizeof(i2cList) - strlen(i2cList) - 1);
      found++;
    }
  }
  if (found == 0)
  {
    Serial.println("  no devices on the bus");
    snprintf(i2cList, sizeof(i2cList), "none");
  }
}

// One status line per second covering every subsystem. hErr and vErr are the
// left/right and top/bottom light differences; the panel only moves when one of
// them exceeds the deadzone, so comparing them here explains a stationary panel.
void serialStatus()
{
  Serial.print("sensors ");
  Serial.print(rawTL);
  Serial.print(" ");
  Serial.print(rawTR);
  Serial.print(" ");
  Serial.print(rawBL);
  Serial.print(" ");
  Serial.print(rawBR);
  Serial.print(" | hErr ");
  Serial.print(hErr);
  Serial.print(" vErr ");
  Serial.print(vErr);
  Serial.print(" deadzone ");
  Serial.print(deadzone);
  Serial.print(" | pan ");
  Serial.print(panAngle);
  Serial.print(" tilt ");
  Serial.print(tiltAngle);
  Serial.print(" | ");
  if (inaOK)
  {
    Serial.print(busV, 2);
    Serial.print("V ");
    Serial.print(current_mA, 2);
    Serial.print("mA ");
    Serial.print(power_mW, 2);
    Serial.print("mW");
  }
  else
    Serial.print("INA219 offline");
  Serial.print(" | ");
  Serial.print(stateLabel);
  Serial.print(" / ");
  Serial.println(modeName());
}

// Reads the four sensors and eases each smoothed value toward its new reading.
// This exponential smoothing averages out noise; smoothPct sets the step size.
void readSensors()
{
  rawTL = analogRead(PIN_TL);
  rawTR = analogRead(PIN_TR);
  rawBL = analogRead(PIN_BL);
  rawBR = analogRead(PIN_BR);
  float a = smoothPct / 100.0;
  tl += (rawTL - tl) * a;
  tr += (rawTR - tr) * a;
  bl += (rawBL - bl) * a;
  br += (rawBR - br) * a;
}

// Returns the badge colour matching the current state.
uint16_t stateColor()
{
  if (mode == MANUAL)
    return 0xFB56; // pink
  if (mode == FIXEDMODE)
    return 0xFD20; // amber
  if (isNight)
    return 0x5BDF; // blue
  return 0x07E0;   // green = actively tracking
}

// Returns the mode as the text tag used by the dashboard.
const char *modeName() { return mode == AUTO ? "auto" : mode == FIXEDMODE ? "fixed"
                                                                          : "manual"; }

// Returns the percentage gain of tracking over fixed. Average power per mode is
// energy divided by run time; returns 0 until a fixed-mode sample exists.
float gainPct()
{
  double hT = msTrack / 3600000.0, hF = msFixed / 3600000.0;
  double pT = hT > 0 ? energyTrack_mWh / hT : 0;
  double pF = hF > 0 ? energyFixed_mWh / hF : 0;
  return pF > 0 ? (float)((pT - pF) / pF * 100.0) : 0;
}

// Draws the status board on the TFT without flicker: the static layout is drawn
// once, then each refresh only overwrites the changing values. Text is printed
// with an explicit background colour so old digits are erased in place --
// no full-screen wipe, so the display no longer flashes every refresh.
void drawLayout()
{
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(4, 2);
  tft.print("SOLAR");
  tft.setTextSize(1);
  tft.setTextColor(0x7BEF);
  tft.setCursor(4, 26);
  tft.print("POWER");
  tft.setTextColor(0x7BEF);
  tft.setCursor(4, 60);
  tft.print("Volt");
  tft.setCursor(92, 60);
  tft.print("P/T");
  tft.setCursor(4, 80);
  tft.print("E trk");
  tft.setCursor(4, 92);
  tft.print("E fix");
  tft.setCursor(4, 104);
  tft.print("Gain");
  tft.setTextColor(0x5BDF);
  tft.setCursor(4, 120);
  tft.print("WiFi ");
  tft.print(AP_SSID);
  tft.print(" .4.1");
  layoutDrawn = true;
}

void updateDisplay()
{
  if (!layoutDrawn)
    drawLayout();
  uint16_t col = stateColor();
  // state badge (redrawn each time -- small, no visible flash)
  tft.fillRoundRect(92, 2, 64, 16, 3, col);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_BLACK);
  tft.setCursor(97, 6);
  tft.print(stateLabel);
  // live power: black background colour erases the previous digits in place
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
  tft.setCursor(4, 36);
  tft.print(power_mW, 1);
  tft.setTextSize(1);
  tft.print(" mW   ");
  // voltage and pan/tilt angles
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setCursor(40, 60);
  tft.print(busV, 2);
  tft.print("V  ");
  tft.setCursor(116, 60);
  tft.print(panAngle);
  tft.print("/");
  tft.print(tiltAngle);
  tft.print("  ");
  // energy totals and gain
  tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  tft.setCursor(46, 80);
  tft.print(energyTrack_mWh, 2);
  tft.print("  ");
  tft.setCursor(46, 92);
  tft.print(energyFixed_mWh, 2);
  tft.print("  ");
  tft.setTextColor(0xFD20, ST77XX_BLACK);
  tft.setCursor(46, 104);
  tft.print(gainPct(), 1);
  tft.print("%  ");
}

// --- Web handlers (each runs when the matching URL is requested) ---

// Serves the dashboard page.
void handleRoot() { server.send(200, "text/html", PAGE); }

// Returns all current values as a single JSON object; polled once per second.
void handleData()
{
  double hT = msTrack / 3600000.0, hF = msFixed / 3600000.0;
  double pT = hT > 0 ? energyTrack_mWh / hT : 0;
  double pF = hF > 0 ? energyFixed_mWh / hF : 0;
  char buf[760];
  snprintf(buf, sizeof(buf),
           "{\"power\":%.1f,\"volts\":%.2f,\"pan\":%d,\"tilt\":%d,"
           "\"eTrack\":%.3f,\"eFixed\":%.3f,\"pTrack\":%.1f,\"pFixed\":%.1f,\"gain\":%.1f,"
           "\"s\":[%d,%d,%d,%d],\"state\":\"%s\",\"mode\":\"%s\",\"stall\":%s,"
           "\"deadzone\":%d,\"step\":%d,\"smooth\":%d,\"dark\":%d,\"fpan\":%d,\"ftilt\":%d,"
           "\"i2c\":\"%s\",\"ina\":%s,\"sv\":%s}",
           power_mW, busV, panAngle, tiltAngle, energyTrack_mWh, energyFixed_mWh, pT, pF, gainPct(),
           rawTL, rawTR, rawBL, rawBR, stateLabel, modeName(),
           (panStalled || tiltStalled) ? "true" : "false",
           deadzone, stepDeg, smoothPct, darkLevel, fixedPan, fixedTilt,
           i2cList, inaOK ? "true" : "false", servosAttached ? "true" : "false");
  server.send(200, "application/json", buf);
}

// Sets the operating mode. Switching to Manual seeds the manual targets with the
// current angles to prevent a jump.
void handleMode()
{
  String m = server.arg("m");
  if (m == "auto")
    mode = AUTO;
  else if (m == "fixed")
    mode = FIXEDMODE;
  else if (m == "manual")
  {
    mode = MANUAL;
    manualPan = panAngle;
    manualTilt = tiltAngle;
  }
  server.send(200, "text/plain", "ok");
}

// Stores manual servo positions, constrained to the safe range.
void handleMove()
{
  if (server.hasArg("pan"))
    manualPan = constrain(server.arg("pan").toInt(), ANGLE_MIN, ANGLE_MAX);
  if (server.hasArg("tilt"))
    manualTilt = constrain(server.arg("tilt").toInt(), ANGLE_MIN, ANGLE_MAX);
  server.send(200, "text/plain", "ok");
}

// Updates one tuning parameter: "k" selects the field, "v" is the value. Each value
// is constrained to a valid range.
void handleSet()
{
  String k = server.arg("k");
  int v = server.arg("v").toInt();
  if (k == "deadzone")
    deadzone = constrain(v, 5, 300);
  else if (k == "step")
    stepDeg = constrain(v, 1, 5);
  else if (k == "smooth")
    smoothPct = constrain(v, 5, 90);
  else if (k == "dark")
    darkLevel = constrain(v, 0, 2000);
  else if (k == "fpan")
    fixedPan = constrain(v, ANGLE_MIN, ANGLE_MAX);
  else if (k == "ftilt")
    fixedTilt = constrain(v, ANGLE_MIN, ANGLE_MAX);
  server.send(200, "text/plain", "ok");
}

// Forced servo sweep for testing. Runs immediately regardless of the active mode,
// so it proves whether the servos and their power supply work at all.
void handleServoTest()
{
  server.send(200, "text/plain", "sweeping");
  for (int a = 60; a <= 120; a += 2)
  {
    panServo.write(a);
    tiltServo.write(a);
    delay(20);
  }
  for (int a = 120; a >= 60; a -= 2)
  {
    panServo.write(a);
    tiltServo.write(a);
    delay(20);
  }
  panServo.write(90);
  tiltServo.write(90);
  manualPan = manualTilt = 90;
}

// Clears both energy buckets to restart the comparison.
void handleReset()
{
  energyTrack_mWh = 0;
  energyFixed_mWh = 0;
  msTrack = 0;
  msFixed = 0;
  server.send(200, "text/plain", "ok");
}