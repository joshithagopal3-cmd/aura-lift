// =====================================================================
//  AURA-LIFT — Smart Elevator Drive Diagnostics — ESP32 firmware v1.2
//  Team Immortal · KIT · Kone Elevate PS-06
//
//  Target : ESP32 Dev Module (ESP32-D0WD-V3), Arduino-ESP32 core 3.x
//  Wiring : AURA-LIFT_Wiring_Checklist.md (encoder on GPIO27, NC limit
//           switches, BTS7960 VCC on 3.3 V, ACS712 via 10k/10k divider)
//
//  What it does
//   • Runs a real trip profile: START → ACCEL → CRUISE → DECEL → LEVEL → STOP
//     (phases are the controller's actual state, not a guess from a timer)
//   • Samples bus voltage, bus current and speed, and accumulates
//     statistics PER PHASE for every trip (the PPT's phase-scheduled idea)
//   • Keeps the last UP and last DOWN cruise currents and reports their
//     sum/difference (the PPT's UP/DOWN pairing, prototype version)
//   • Web dashboard + JSON API + CSV download; Serial commands for bench use
//   • Safety: limit switches (NC, fail-safe), over-current with start-up
//     blanking, under-voltage, trip timeout, driver disabled when stopped
//
//  Serial commands (115200, "New Line"):  u = up   d = down   s = stop
//                                          c = re-zero current sensor   ? = status
// =====================================================================
#include <WiFi.h>
#include <WebServer.h>

// ============================ USER CONFIG ============================
// ---- WiFi: tries your router first, falls back to its own hotspot ----
// Leave both empty to skip the router. After about 15 s the board starts its
// own hotspot (AURA-LIFT) and serves the dashboard at http://192.168.4.1
const char*    WIFI_SSID        = "";   // your router / phone hotspot name (optional)
const char*    WIFI_PASSWORD    = "";   // its password (optional) - never commit real credentials
const uint32_t WIFI_TIMEOUT_MS  = 15000;
const char*    AP_SSID          = "AURA-LIFT";       // fallback hotspot name
const char*    AP_PASSWORD      = "auralift123";     // min 8 characters

// ---- Pins (match the wiring checklist) ----
const int PIN_RPWM   = 25;
const int PIN_LPWM   = 26;
const int PIN_REN    = 32;
const int PIN_LEN    = 33;
const int PIN_CURR   = 34;   // ACS712 OUT via 10k/10k divider (ADC1)
const int PIN_VOLT   = 35;   // voltage sensor S (ADC1)
const int PIN_TOP    = 18;   // top limit switch NC  (HIGH = reached / wire broken)
const int PIN_BOTTOM = 19;   // bottom limit switch NC
const int PIN_ENC    = 27;   // LM393 slot sensor DO

// ---- Motor / trip profile ----
const bool     INVERT_DIRECTION = false; // set true if "UP" moves the cabin down
const uint32_t PWM_FREQ_HZ      = 1000;
const uint8_t  PWM_BITS         = 8;     // duty range 0..255
const int      DUTY_START       = 60;    // break-away duty (START phase)
const int      DUTY_CRUISE      = 150;   // full-speed duty
const int      DUTY_LEVEL       = 70;    // slow approach to the floor
const uint32_t T_START_MS       = 300;   // START window (≈ PPT "brake pick" window)
const uint32_t T_ACCEL_MS       = 1500;  // ramp DUTY_START → DUTY_CRUISE
const uint32_t T_CRUISE_MS      = 3000;  // constant speed
const uint32_t T_DECEL_MS       = 1000;  // ramp DUTY_CRUISE → DUTY_LEVEL
const uint32_t T_LEVEL_MAX_MS   = 5000;  // creep until the limit switch; give up after this
const uint32_t TRIP_TIMEOUT_MS  = 15000; // absolute safety limit per trip

// ---- Sensors (measured / datasheet values) ----
const float VOLT_DIVIDER   = 5.05;   // measured: 12.23 V in → 2.42 V out
const float ACS_DIVIDER    = 2.0;    // 10k/10k divider on ACS712 OUT
const float ACS_MV_PER_A   = 100.0;  // ACS712-20A datasheet sensitivity
const int   ADC_SAMPLES    = 32;     // averaged per reading (smooths PWM ripple)
const int   ZERO_SAMPLES   = 400;    // samples for current-sensor zero at boot
const bool  ENCODER_FITTED = false;  // true once a slotted disc + LM393 sensor are installed
const float ENC_SLOTS      = 20.0;   // slots on your encoder disc
const uint32_t ENC_DEBOUNCE_US = 300;// ignore edges closer than this
const uint32_t SAMPLE_MS   = 100;    // sensor + statistics update period

// ---- Protection ----
const float    OVERCURRENT_A   = 3.0;  // stop above this (adapter is 2 A; motor inrush allowed by blanking)
const uint32_t OC_BLANK_MS     = 400;  // ignore over-current right after a start
const bool     CHECK_UNDERVOLT = true; // set false while the voltage sensor is not wired
const float    UNDERVOLT_V     = 9.0;

const int MAX_TRIPS = 20;              // trip history kept in RAM
// =====================================================================

WebServer server(80);

enum Phase : uint8_t { IDLE, START, ACCEL, CRUISE, DECEL, LEVEL, N_PHASES };
const char* PHASE_NAME[N_PHASES] = { "Idle", "Start", "Accel", "Cruise", "Decel", "Level" };

struct PhaseStats {
  uint32_t n = 0;
  float sumV = 0, sumI = 0, sumRPM = 0, peakI = 0;
  void add(float v, float i, float r) {
    n++; sumV += v; sumI += i; sumRPM += r;
    if (fabsf(i) > fabsf(peakI)) peakI = i;
  }
  float avgV()   const { return n ? sumV / n : 0; }
  float avgI()   const { return n ? sumI / n : 0; }
  float avgRPM() const { return n ? sumRPM / n : 0; }
};

struct Trip {
  uint32_t   number = 0;
  int8_t     dir = 0;              // +1 up, -1 down
  uint32_t   durationMs = 0;
  char       endReason[32] = "";
  PhaseStats ph[N_PHASES];
};

// ---------------- live state ----------------
volatile uint32_t encPulses = 0;
volatile uint32_t encLastUs = 0;

float    busV = 0, busI = 0, rpm = 0;
float    acsZero_mV = 0;
Phase    phase = IDLE;
int8_t   tripDir = 0;
int      duty = 0;
uint32_t tripStartMs = 0, phaseStartMs = 0, lastSampleMs = 0;
uint32_t tripCounter = 0;
String   status = "System ready";
String   fault  = "";

Trip     trips[MAX_TRIPS];
int      tripCount = 0;            // total stored (≤ MAX_TRIPS)
int      tripHead  = 0;            // next write index
Trip     cur;                      // trip in progress
float    lastCruiseUp = NAN, lastCruiseDown = NAN;
bool     apMode = false;
bool     zeroed = false;           // current-sensor zero taken with 12 V present?
uint32_t busOkSinceMs = 0;

// =====================================================================
//  Hardware helpers
// =====================================================================
void IRAM_ATTR onEncoder() {
  uint32_t now = micros();
  if (now - encLastUs >= ENC_DEBOUNCE_US) { encPulses = encPulses + 1; encLastUs = now; }
}

float readPin_mV(int pin) {
  uint32_t sum = 0;
  for (int i = 0; i < ADC_SAMPLES; i++) { sum += analogReadMilliVolts(pin); delayMicroseconds(100); }
  return sum / (float)ADC_SAMPLES;
}

float readBusVolts()  { return readPin_mV(PIN_VOLT) / 1000.0f * VOLT_DIVIDER; }
float readBusAmps()   { return (readPin_mV(PIN_CURR) - acsZero_mV) * ACS_DIVIDER / ACS_MV_PER_A; }

bool topReached()    { return digitalRead(PIN_TOP)    == HIGH; }
bool bottomReached() { return digitalRead(PIN_BOTTOM) == HIGH; }

void driverEnable(bool on) { digitalWrite(PIN_REN, on); digitalWrite(PIN_LEN, on); }

void applyDuty(int8_t dir, int d) {
  d = constrain(d, 0, (1 << PWM_BITS) - 1);
  bool fwd = (dir > 0) != INVERT_DIRECTION;
  if (dir == 0 || d == 0) { ledcWrite(PIN_RPWM, 0); ledcWrite(PIN_LPWM, 0); duty = 0; return; }
  if (fwd) { ledcWrite(PIN_LPWM, 0); ledcWrite(PIN_RPWM, d); }
  else     { ledcWrite(PIN_RPWM, 0); ledcWrite(PIN_LPWM, d); }
  duty = d;
}

void calibrateCurrentZero() {
  // Motor must be stopped. Averages ZERO_SAMPLES readings of the ACS712 output.
  double acc = 0;
  for (int i = 0; i < ZERO_SAMPLES; i++) { acc += analogReadMilliVolts(PIN_CURR); delay(2); }
  acsZero_mV = acc / ZERO_SAMPLES;
  Serial.printf("[cal] current zero = %.1f mV at GPIO34 (%.2f V at ACS712)\n",
                acsZero_mV, acsZero_mV * ACS_DIVIDER / 1000.0);
}

// =====================================================================
//  Trip control
// =====================================================================
void setPhase(Phase p) { phase = p; phaseStartMs = millis(); }

void endTrip(const char* reason, bool isFault) {
  applyDuty(0, 0);
  driverEnable(false);
  if (phase != IDLE) {
    cur.durationMs = millis() - tripStartMs;
    strncpy(cur.endReason, reason, sizeof(cur.endReason) - 1);
    float cruiseI = cur.ph[CRUISE].avgI();
    if (cur.ph[CRUISE].n > 0) { if (cur.dir > 0) lastCruiseUp = cruiseI; else lastCruiseDown = cruiseI; }
    trips[tripHead] = cur;
    tripHead = (tripHead + 1) % MAX_TRIPS;
    if (tripCount < MAX_TRIPS) tripCount++;
    Serial.printf("[trip %lu] %s  %lu ms  end: %s\n", (unsigned long)cur.number,
                  cur.dir > 0 ? "UP" : "DOWN", (unsigned long)cur.durationMs, reason);
  }
  phase = IDLE; tripDir = 0;
  status = reason;
  if (isFault) fault = reason;
}

bool startTrip(int8_t dir) {
  if (phase != IDLE)                    { Serial.println("[cmd] busy — stop first"); return false; }
  if (topReached() && bottomReached())  { fault = "Both limits open - check switch wiring"; status = fault; return false; }
  if (dir > 0 && topReached())          { status = "Already at top";    return false; }
  if (dir < 0 && bottomReached())       { status = "Already at bottom"; return false; }
  if (CHECK_UNDERVOLT && busV < UNDERVOLT_V) { fault = "Bus voltage low - is 12 V on?"; status = fault; return false; }
  if (!zeroed) { status = "Current sensor zeroing - wait 2 s"; return false; }

  fault = "";
  cur = Trip();
  cur.number = ++tripCounter;
  cur.dir = dir;
  tripDir = dir;
  tripStartMs = millis();
  driverEnable(true);
  setPhase(START);
  applyDuty(dir, DUTY_START);
  status = dir > 0 ? "Running UP" : "Running DOWN";
  Serial.printf("[trip %lu] start %s\n", (unsigned long)cur.number, dir > 0 ? "UP" : "DOWN");
  return true;
}

void updateTrip() {
  if (phase == IDLE) return;
  uint32_t now = millis(), inPhase = now - phaseStartMs;

  // ---- safety first ----
  bool arrived = (tripDir > 0) ? topReached() : bottomReached();
  if (arrived)                                              { endTrip(tripDir > 0 ? "Arrived at top" : "Arrived at bottom", false); return; }
  if (now - tripStartMs > TRIP_TIMEOUT_MS)                  { endTrip("FAULT: trip timeout", true); return; }
  if (now - tripStartMs > OC_BLANK_MS && fabsf(busI) > OVERCURRENT_A) { endTrip("FAULT: over-current", true); return; }
  if (CHECK_UNDERVOLT && busV < UNDERVOLT_V)                { endTrip("FAULT: under-voltage", true); return; }

  // ---- profile ----
  switch (phase) {
    case START:
      if (inPhase >= T_START_MS) setPhase(ACCEL);
      break;
    case ACCEL: {
      float k = min(1.0f, inPhase / (float)T_ACCEL_MS);
      applyDuty(tripDir, DUTY_START + (int)((DUTY_CRUISE - DUTY_START) * k));
      if (inPhase >= T_ACCEL_MS) setPhase(CRUISE);
      break; }
    case CRUISE:
      applyDuty(tripDir, DUTY_CRUISE);
      if (inPhase >= T_CRUISE_MS) setPhase(DECEL);
      break;
    case DECEL: {
      float k = min(1.0f, inPhase / (float)T_DECEL_MS);
      applyDuty(tripDir, DUTY_CRUISE - (int)((DUTY_CRUISE - DUTY_LEVEL) * k));
      if (inPhase >= T_DECEL_MS) setPhase(LEVEL);
      break; }
    case LEVEL:
      applyDuty(tripDir, DUTY_LEVEL);
      if (inPhase >= T_LEVEL_MAX_MS) endTrip("Level timeout (no limit hit)", false);
      break;
    default: break;
  }
}

// =====================================================================
//  Sampling
// =====================================================================
void updateSensors() {
  uint32_t now = millis();
  if (now - lastSampleMs < SAMPLE_MS) return;
  uint32_t dt = now - lastSampleMs;
  lastSampleMs = now;

  noInterrupts(); uint32_t p = encPulses; encPulses = 0; interrupts();
  rpm  = (ENCODER_FITTED && ENC_SLOTS > 0) ? (p * (60000.0f / dt) / ENC_SLOTS) : 0;
  busV = readBusVolts();
  busI = readBusAmps();

  // Auto-zero: the ACS712 is powered from the 12 V side (via LM2596), so its zero
  // can only be learned once 12 V is present and the motor is stopped.
  if (CHECK_UNDERVOLT && phase == IDLE) {
    if (busV >= UNDERVOLT_V) {
      if (!busOkSinceMs) busOkSinceMs = now;
      if (!zeroed && now - busOkSinceMs > 1500) {
        Serial.println("[cal] 12 V detected, motor stopped -> zeroing current sensor");
        calibrateCurrentZero(); zeroed = true; status = "Ready";
      }
    } else {
      if (zeroed) Serial.println("[cal] 12 V lost -> will re-zero when it returns");
      busOkSinceMs = 0; zeroed = false;
    }
  }

  if (phase != IDLE) cur.ph[phase].add(busV, busI, rpm);

  // CSV stream for the laptop: t_ms,trip,dir,phase,duty,V,A,rpm
  if (phase != IDLE)
    Serial.printf("DATA,%lu,%lu,%d,%s,%d,%.2f,%.3f,%.1f\n", (unsigned long)now,
                  (unsigned long)cur.number, tripDir, PHASE_NAME[phase], duty, busV, busI, rpm);
}

// =====================================================================
//  Web
// =====================================================================
String phaseJSON(const PhaseStats& s) {
  char b[120];
  snprintf(b, sizeof b, "{\"n\":%lu,\"v\":%.2f,\"i\":%.3f,\"rpm\":%.1f,\"peak\":%.3f}",
           (unsigned long)s.n, s.avgV(), s.avgI(), s.avgRPM(), s.peakI);
  return String(b);
}

String tripJSON(const Trip& t) {
  String j = "{\"n\":" + String(t.number) + ",\"dir\":\"" + (t.dir > 0 ? "UP" : "DOWN") +
             "\",\"ms\":" + String(t.durationMs) + ",\"end\":\"" + t.endReason + "\",\"phases\":{";
  for (int p = START; p < N_PHASES; p++) {
    j += "\""; j += PHASE_NAME[p]; j += "\":"; j += phaseJSON(t.ph[p]);
    if (p < N_PHASES - 1) j += ",";
  }
  return j + "}}";
}

void handleApi() {
  char b[520];
  // Bus-current convention (ACS712 sits in the 12 V supply line):
  //   UP   : drive supplies imbalance + losses      -> I_up   = L + F
  //   DOWN : gravity returns the imbalance           -> I_down = F - L
  //   so  (I_up - I_down)/2 = L (load / imbalance term)
  //       (I_up + I_down)/2 = F (loss / friction term)
  bool both = !(isnan(lastCruiseUp) || isnan(lastCruiseDown));
  float load = both ? (lastCruiseUp - lastCruiseDown) / 2 : NAN;
  float loss = both ? (lastCruiseUp + lastCruiseDown) / 2 : NAN;
  snprintf(b, sizeof b,
    "{\"v\":%.2f,\"i\":%.3f,\"rpm\":%.1f,\"duty\":%d,\"phase\":\"%s\",\"dir\":%d,"
    "\"trip\":%lu,\"status\":\"%s\",\"fault\":\"%s\",\"top\":%s,\"bottom\":%s,"
    "\"upI\":%s,\"downI\":%s,\"loadTerm\":%s,\"lossTerm\":%s,\"ap\":%s,\"enc\":%s}",
    busV, busI, rpm, duty, PHASE_NAME[phase], tripDir, (unsigned long)tripCounter,
    status.c_str(), fault.c_str(), topReached() ? "true" : "false", bottomReached() ? "true" : "false",
    isnan(lastCruiseUp)   ? "null" : String(lastCruiseUp, 3).c_str(),
    isnan(lastCruiseDown) ? "null" : String(lastCruiseDown, 3).c_str(),
    isnan(load) ? "null" : String(load, 3).c_str(),
    isnan(loss) ? "null" : String(loss, 3).c_str(),
    apMode ? "true" : "false", ENCODER_FITTED ? "true" : "false");
  server.send(200, "application/json", b);
}

void handleTrips() {
  String j = "[";
  for (int k = 0; k < tripCount; k++) {
    int idx = (tripHead - 1 - k + MAX_TRIPS) % MAX_TRIPS;   // newest first
    if (k) j += ",";
    j += tripJSON(trips[idx]);
  }
  server.send(200, "application/json", j + "]");
}

void handleCsv() {
  String c = "trip,dir,duration_ms,end,phase,samples,avg_V,avg_A,avg_rpm,peak_A\n";
  for (int k = tripCount - 1; k >= 0; k--) {                 // oldest first
    const Trip& t = trips[(tripHead - 1 - k + MAX_TRIPS) % MAX_TRIPS];
    for (int p = START; p < N_PHASES; p++) {
      const PhaseStats& s = t.ph[p];
      char b[160];
      snprintf(b, sizeof b, "%lu,%s,%lu,%s,%s,%lu,%.2f,%.3f,%.1f,%.3f\n",
               (unsigned long)t.number, t.dir > 0 ? "UP" : "DOWN", (unsigned long)t.durationMs,
               t.endReason, PHASE_NAME[p], (unsigned long)s.n, s.avgV(), s.avgI(), s.avgRPM(), s.peakI);
      c += b;
    }
  }
  server.sendHeader("Content-Disposition", "attachment; filename=auralift_trips.csv");
  server.send(200, "text/csv", c);
}

void handleCmd() {
  String c = server.arg("c");
  if      (c == "up")   startTrip(+1);
  else if (c == "down") startTrip(-1);
  else if (c == "stop") endTrip("User STOP", false);
  else if (c == "zero") { if (phase == IDLE) { calibrateCurrentZero(); zeroed = true; status = "Current re-zeroed"; } }
  else { server.send(400, "text/plain", "unknown"); return; }
  server.send(200, "text/plain", status);
}

const char PAGE[] PROGMEM = R"HTML(<!DOCTYPE html><html><head>
<meta name="viewport" content="width=device-width,initial-scale=1"><title>AURA-LIFT</title>
<style>
body{margin:0;font-family:Arial,sans-serif;background:#07111f;color:#e8eef6}
.h{background:#0d1b2e;padding:16px 20px;font-size:24px;font-weight:bold}
.h small{display:block;font-size:13px;color:#8fa8c2;font-weight:normal;margin-top:4px}
.w{padding:16px;max-width:1100px;margin:auto}
.g{display:grid;grid-template-columns:repeat(auto-fit,minmax(160px,1fr));gap:12px}
.c{background:#10233a;padding:14px;border-radius:12px}
.l{color:#91a7bd;font-size:13px}.v{font-size:28px;font-weight:bold;margin-top:6px}
.b{display:flex;gap:10px;flex-wrap:wrap;margin:16px 0}
button{padding:14px 26px;border:0;border-radius:10px;font-size:16px;font-weight:bold;color:#fff;cursor:pointer}
.up{background:#2196f3}.dn{background:#673ab7}.st{background:#e53935}.z{background:#455a64}
.s{background:#10233a;padding:16px;border-radius:12px;margin-top:14px}
table{width:100%;border-collapse:collapse;font-size:14px}td,th{padding:7px;border-bottom:1px solid #29415c;text-align:left}
.ph span{display:inline-block;padding:6px 12px;margin:3px;border-radius:14px;background:#1b3350;color:#91a7bd}
.ph span.on{background:#00c853;color:#032}
.ok{color:#00e676}.bad{color:#ff5252}.note{color:#8fa8c2;font-size:12px}
a{color:#64b5f6}
</style></head><body>
<div class="h">⚡ AURA-LIFT<small>Trip-synchronous drive diagnostics — prototype bench rig</small></div>
<div class="w">
<div class="g">
<div class="c"><div class="l">Bus current</div><div class="v"><span id="i">-</span> A</div></div>
<div class="c"><div class="l">Bus voltage</div><div class="v"><span id="v">-</span> V</div></div>
<div class="c"><div class="l">Motor speed (encoder)</div><div class="v"><span id="rpm">-</span> rpm</div></div>
<div class="c"><div class="l">PWM duty</div><div class="v"><span id="duty">-</span></div></div>
</div>
<div class="b">
<button class="up" onclick="cmd('up')">⬆ UP</button>
<button class="dn" onclick="cmd('down')">⬇ DOWN</button>
<button class="st" onclick="cmd('stop')">⛔ STOP</button>
<button class="z"  onclick="cmd('zero')">Re-zero current</button>
</div>
<div class="s"><b>Trip phase</b>
<div class="ph" id="ph"><span>Idle</span><span>Start</span><span>Accel</span><span>Cruise</span><span>Decel</span><span>Level</span></div>
<p>Status: <b id="status">-</b> <span id="fault" class="bad"></span></p>
<p>Top limit: <b id="top">-</b> &nbsp; Bottom limit: <b id="bot">-</b> &nbsp; Trips: <b id="trip">-</b></p>
</div>
<div class="s"><b>UP / DOWN pairing (cruise current)</b>
<table><tr><th>Last UP</th><th>Last DOWN</th><th>(UP−DOWN)/2 → load term</th><th>(UP+DOWN)/2 → loss term</th></tr>
<tr><td id="up">-</td><td id="dn">-</td><td id="load">-</td><td id="loss">-</td></tr></table>
<p class="note">Measured on the 12 V DC bus. Going UP the drive supplies imbalance + losses; going DOWN gravity gives the
imbalance back, so the drive supplies only losses − imbalance. The half-difference isolates the load term and the
half-sum isolates the loss term: the UP/DOWN separation of PPT slide 3, in bus-current form. A DOWN value can be
negative (energy returned). Needs one UP and one DOWN trip. All values are measured live on the bench rig.</p>
</div>
<div class="s"><b>Recent trips — per-phase averages</b> &nbsp; <a href="/csv">download CSV</a>
<table id="tt"><tr><th>#</th><th>Dir</th><th>End</th><th>Start A</th><th>Accel A</th><th>Cruise A</th><th>Cruise V</th><th>Cruise rpm</th><th>Level A</th></tr></table>
</div>
</div>
<script>
const $=id=>document.getElementById(id);
const f=(x,d)=>x===null||x===undefined?'-':Number(x).toFixed(d);
function cmd(c){fetch('/cmd?c='+c).then(r=>r.text()).then(t=>{$('status').innerText=t;});}
function live(){fetch('/api').then(r=>r.json()).then(d=>{
 $('i').innerText=f(d.i,2);$('v').innerText=f(d.v,2);$('rpm').innerText=d.enc?f(d.rpm,1):'n/a';$('duty').innerText=d.duty;
 $('status').innerText=d.status;$('fault').innerText=d.fault?(' ⚠ '+d.fault):'';
 $('top').innerText=d.top?'REACHED':'clear';$('bot').innerText=d.bottom?'REACHED':'clear';
 $('trip').innerText=d.trip;
 $('up').innerText=f(d.upI,3)+' A';$('dn').innerText=f(d.downI,3)+' A';
 $('load').innerText=f(d.loadTerm,3)+' A';$('loss').innerText=f(d.lossTerm,3)+' A';
 document.querySelectorAll('#ph span').forEach(s=>s.className=(s.innerText===d.phase)?'on':'');
}).catch(()=>{});}
function trips(){fetch('/trips').then(r=>r.json()).then(a=>{
 let h='<tr><th>#</th><th>Dir</th><th>End</th><th>Start A</th><th>Accel A</th><th>Cruise A</th><th>Cruise V</th><th>Cruise rpm</th><th>Level A</th></tr>';
 a.slice(0,10).forEach(t=>{const p=t.phases;h+=`<tr><td>${t.n}</td><td>${t.dir}</td><td>${t.end}</td>
 <td>${f(p.Start.i,2)}</td><td>${f(p.Accel.i,2)}</td><td>${f(p.Cruise.i,2)}</td><td>${f(p.Cruise.v,2)}</td>
 <td>${f(p.Cruise.rpm,1)}</td><td>${f(p.Level.i,2)}</td></tr>`;});
 $('tt').innerHTML=h;}).catch(()=>{});}
setInterval(live,500);setInterval(trips,3000);live();trips();
</script></body></html>)HTML";

void handleRoot() { server.send_P(200, "text/html", PAGE); }

// =====================================================================
//  WiFi
// =====================================================================
void startNetwork() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("[wifi] connecting to "); Serial.print(WIFI_SSID);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < WIFI_TIMEOUT_MS) { delay(400); Serial.print("."); }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    apMode = false;
    Serial.print("[wifi] connected. Open http://"); Serial.println(WiFi.localIP());
  } else {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    apMode = true;
    Serial.printf("[wifi] router not found -> hotspot \"%s\" (password %s)\n", AP_SSID, AP_PASSWORD);
    Serial.print("[wifi] join it, then open http://"); Serial.println(WiFi.softAPIP());
  }
}

// =====================================================================
//  Serial commands
// =====================================================================
void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'u': startTrip(+1); break;
      case 'd': startTrip(-1); break;
      case 's': endTrip("User STOP", false); break;
      case 'c': if (phase == IDLE) { calibrateCurrentZero(); zeroed = true; } break;
      case '?':
        Serial.printf("[status] %s | phase=%s duty=%d V=%.2f I=%.3f rpm=%.1f top=%s bottom=%s fault=%s\n",
                      status.c_str(), PHASE_NAME[phase], duty, busV, busI, rpm,
                      topReached() ? "REACHED" : "clear", bottomReached() ? "REACHED" : "clear",
                      fault.length() ? fault.c_str() : "none");
        break;
      default: break;   // ignore newline etc.
    }
  }
}

// =====================================================================
void setup() {
  // motor OFF before anything else
  pinMode(PIN_REN, OUTPUT); pinMode(PIN_LEN, OUTPUT);
  driverEnable(false);
  ledcAttach(PIN_RPWM, PWM_FREQ_HZ, PWM_BITS);
  ledcAttach(PIN_LPWM, PWM_FREQ_HZ, PWM_BITS);
  applyDuty(0, 0);

  pinMode(PIN_TOP, INPUT_PULLUP);
  pinMode(PIN_BOTTOM, INPUT_PULLUP);
  if (ENCODER_FITTED) {
    pinMode(PIN_ENC, INPUT);               // slot sensor module has its own pull-up
    attachInterrupt(digitalPinToInterrupt(PIN_ENC), onEncoder, RISING);
  }

  Serial.begin(115200);
  delay(1500);                             // let the Serial Monitor catch up
  Serial.println("\n=== AURA-LIFT firmware v1.2 ===");
  busV = readBusVolts();
  if (!CHECK_UNDERVOLT) {                  // no voltage sensor: zero now, as before
    calibrateCurrentZero(); zeroed = true;
  } else {
    Serial.printf("[cal] bus = %.2f V. Current sensor will auto-zero 1.5 s after 12 V is present.\n", busV);
    status = "Waiting for 12 V";
  }

  startNetwork();
  server.on("/",      handleRoot);
  server.on("/api",   handleApi);
  server.on("/trips", handleTrips);
  server.on("/csv",   handleCsv);
  server.on("/cmd",   handleCmd);
  server.begin();

  Serial.println("Serial commands: u=up d=down s=stop c=re-zero ?=status");
  lastSampleMs = millis();
}

void loop() {
  server.handleClient();
  handleSerial();
  updateSensors();
  updateTrip();
}
