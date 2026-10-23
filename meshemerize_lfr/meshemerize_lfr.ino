// =====================================================================
//  ESP32 + Makeblock 4-ch RGB line follower (I2C) + L298N
//  Meshemerize 2026-27 - dry run, left-hand rule, loop-breaking, end box
//
//  Robot geometry this was tuned for (see sim.py):
//    T-body, wheels 170 mm apart, sensor array 100 mm AHEAD of the axle.
//    Because the array is ahead of the axle, a junction is only "reached" by
//    the axle ~110 mm after the sensors first see it. So the FSM rolls
//    forward (JUNCTION state) before pivoting; pivoting early swings the
//    array through empty floor and never finds the branch.
//
//  Logic is a 1:1 port of class FW in sim.py. Validated in simulation on
//  400 random loop/dead-end courses across 10 hardware variants (391/400).
//
//  Bluetooth live tuning (Classic BT, device name "LFR"), send lines like:
//      kp 20   ki 6   kd 0.6   cruise 160   min 100   creep 90
//      turn 95   inner -0.9   adv 740        -> change a value live
//      get   -> print all values      save -> store in flash
//      go    -> start now             stop -> halt motors
//      telem 1 / telem 0 -> stream telemetry
//  The same commands work from the USB Serial Monitor.
// =====================================================================
#include <Wire.h>
#include <math.h>
#include <Preferences.h>
#include "BluetoothSerial.h"

// ---------- feature switches ----------
#define CALIBRATE_CREEP 0   // 1 = one-off: roll at CREEP speed for 1 s so you can measure mm/s
#define AUTO_START_MS   4000 // start by itself this long after boot (or send "go" earlier)

// ---------- I2C SENSOR ----------
#define SENSOR_ADDR  0x20
#define STATE_REG    0x07
#define I2C_SDA_PIN  21
#define I2C_SCL_PIN  22
const uint8_t WHITE_BACKGROUND_STATE = 0x0F;   // raw 1111 = all white

// ---------- MOTOR PINS (L298N) ----------
#define ENA 25       // Right motor PWM
#define ENB 13       // Left motor PWM
#define IN1 26
#define IN2 27
#define IN3 33
#define IN4 32
#define STATUS_LED  2   // blinks on sensor fault / lost
#define RED_LED     4   // lit when end box reached

const bool RIGHT_MOTOR_INVERTED = true;
const bool LEFT_MOTOR_INVERTED  = true;

// 20 kHz = silent, but the L298N is a slow bipolar driver. If the motors feel weak or the
// L298N runs hot at low PWM, drop this to 5000.
const int PWM_FREQ       = 20000;
const int PWM_RESOLUTION = 8;
const int MAX_PWM        = 255;

// ---------- live-tunable parameters (defaults = simulation optimum) ----------
struct Tune {
  float    kp = 20.0f, ki = 6.0f, kd = 0.6f;
  int      cruise = 160, minSpeed = 100, creep = 90, turnSpeed = 95;
  float    turnInner = -0.9f;          // inner wheel = turnSpeed * this. -1.0 = pivot, closer to 0 = wider arc
  uint32_t advanceMs = 740;            // creep time that puts the AXLE on the junction centre (not critical: 500-980 all work in sim)
} T;

// ---------- fixed constants ----------
const uint32_t CONTROL_PERIOD_US = 5000;           // 200 Hz
const float    DT = CONTROL_PERIOD_US / 1e6f;
const float    I_CLAMP = 40.0f, ERR_EMA = 0.5f, MAX_CORR = 110.0f, SPEED_DROP = 0.45f;
const uint32_t STRAIGHT_WINDOW_MS = 140;   // last part of the roll where "straight ahead" is sampled
const uint32_t END_HOLD_MS = 350;          // all-black held this long = end box
const uint32_t END_ROLL_MS = 600;          // keep rolling after that so the axle ends INSIDE the box
const uint32_t LOST_CONFIRM_MS = 40;
const float    LOST_CENTERED = 1.6f;       // |error| below this when the line vanishes = gap/dead end (else sharp curve)
const uint32_t GAP_MS = 450;               // roll straight this long before calling it a dead end
const uint32_t POST_TURN_SLOW_MS = 250;
const uint32_t TURN_TIMEOUT_MS = 3000, UTURN_TIMEOUT_MS = 5000;
const uint32_t SEARCH_FIRST_MS = 1200, SEARCH_TOTAL_MS = 3600;
const uint32_t START_WAIT_MS = 200, START_SEEK_MS = 3000;
const int      LAP_MIN = 3, LAP_MAX = 8;

// ---------- globals ----------
Preferences prefs;
BluetoothSerial SerialBT;

enum State { ST_START, ST_FOLLOW, ST_JUNCTION, ST_TURN, ST_SEARCH, ST_DONE, ST_GAP };
State    state = ST_START;
uint32_t stateSince = 0, lastControlUs = 0, runStartMs = 0;
bool     running = false, telem = false, sensorFault = false;
char     doneReason = 0;                   // 'E' end box, 'L' lost

uint8_t  hist[3] = {0, 0, 0};  bool filtInit = false;
float    errF = 0, prevErrF = 0, lastErr = 0, integ = 0;
bool     zeroTracking = false;  uint32_t zeroSince = 0;
uint32_t slowUntil = 0;

bool     jL = false, jR = false, jS = false;
bool     abTracking = false;   uint32_t abSince = 0;
bool     endRolling = false;   uint32_t endUntil = 0;

int      turnDir = 1, turnPhase = 0, leaveCnt = 0, acqCnt = 0;  bool turnIsU = false;
int      searchDir = 1, searchAcq = 0;
int      lastL = 0, lastR = 0;

#define MAX_TURNS 256
char turnLog[MAX_TURNS]; int turnCount = 0, guardStart = 0, loopBreaks = 0;

// ---------- helpers ----------
inline int popcount4(uint8_t m) { return (m & 1) + ((m >> 1) & 1) + ((m >> 2) & 1) + ((m >> 3) & 1); }

void say(const String &s) { Serial.println(s); if (SerialBT.hasClient()) SerialBT.println(s); }

bool readSensorState(uint8_t &raw) {
  Wire.beginTransmission(SENSOR_ADDR);
  Wire.write(STATE_REG);
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom((uint8_t)SENSOR_ADDR, (uint8_t)1) != 1) return false;
  raw = Wire.read() & 0x0F;
  return true;
}
// bit0 = R1 (left) ... bit3 = R4 (right); 1 = black line under that sensor
uint8_t getLineMask(uint8_t raw) { return (WHITE_BACKGROUND_STATE ^ raw) & 0x0F; }

uint8_t filterMask(uint8_t m) {            // 3-sample majority, per bit
  if (!filtInit) { hist[0] = hist[1] = hist[2] = m; filtInit = true; return m; }
  hist[0] = hist[1]; hist[1] = hist[2]; hist[2] = m;
  return ((hist[0] & hist[1]) | (hist[0] & hist[2]) | (hist[1] & hist[2])) & 0x0F;
}

// positive = line is LEFT of centre. returns false if no sensor sees the line
bool lineError(uint8_t m, float &e) {
  const int pos[4] = {3, 1, -1, -3};
  int sum = 0, n = 0;
  for (int i = 0; i < 4; i++) if (m & (1 << i)) { sum += pos[i]; n++; }
  if (!n) return false;
  e = (float)sum / n; return true;
}

// ---------- motors ----------
void setMotor(int pwmPin, int inA, int inB, int speed, bool inverted) {
  speed = constrain(speed, -MAX_PWM, MAX_PWM);
  if (speed == 0) { ledcWrite(pwmPin, 0); digitalWrite(inA, LOW); digitalWrite(inB, LOW); return; }
  bool fwd = speed > 0; if (inverted) fwd = !fwd;
  digitalWrite(inA, fwd ? HIGH : LOW); digitalWrite(inB, fwd ? LOW : HIGH);
  ledcWrite(pwmPin, abs(speed));
}
void driveMotors(int l, int r) {
  lastL = l; lastR = r;
  setMotor(ENA, IN1, IN2, r, RIGHT_MOTOR_INVERTED);
  setMotor(ENB, IN3, IN4, l, LEFT_MOTOR_INVERTED);
}
void stopMotors() { driveMotors(0, 0); }

// ---------- path log + LSRB simplification (removes dead ends) ----------
void logTurn(char c) { if (turnCount < MAX_TURNS) turnLog[turnCount++] = c; }

bool lapping() {                           // last k moves repeat the k before them, or 4 identical turns
  int n = turnCount - guardStart;
  for (int k = LAP_MIN; k <= LAP_MAX; k++) {
    if (n < 2 * k) break;
    bool same = true, hasTurn = false;
    for (int i = 1; i <= k; i++) {
      char a = turnLog[turnCount - i], b = turnLog[turnCount - i - k];
      if (a != b) { same = false; break; }
      if (a == 'L' || a == 'R') hasTurn = true;
    }
    if (same && hasTurn) return true;
  }
  if (n >= 4) {
    char c = turnLog[turnCount - 1];
    if ((c == 'L' || c == 'R') && turnLog[turnCount - 2] == c && turnLog[turnCount - 3] == c && turnLog[turnCount - 4] == c) return true;
  }
  return false;
}

String simplifiedPath() {
  char p[MAX_TURNS]; int n = turnCount; memcpy(p, turnLog, n);
  bool changed = true;
  while (changed) {
    changed = false;
    for (int i = 1; i + 1 < n; i++) {
      if (p[i] != 'U') continue;
      char a = p[i - 1], b = p[i + 1], r = 0;
      if      (a == 'L' && b == 'R') r = 'U';
      else if (a == 'L' && b == 'S') r = 'R';
      else if (a == 'R' && b == 'L') r = 'U';
      else if (a == 'S' && b == 'L') r = 'R';
      else if (a == 'S' && b == 'S') r = 'U';
      else if (a == 'L' && b == 'L') r = 'S';
      if (!r) continue;
      p[i - 1] = r;
      for (int j = i; j + 2 < n; j++) p[j] = p[j + 2];
      n -= 2; changed = true; break;
    }
  }
  return String(p).substring(0, n);
}

// ---------- state entries ----------
void enter(State s, uint32_t now) { state = s; stateSince = now; }

void resumeFollowing(uint8_t m, uint32_t now) {
  float e; errF = prevErrF = lineError(m, e) ? e : 0.0f;
  integ = 0; zeroTracking = false;
  slowUntil = now + POST_TURN_SLOW_MS;
  enter(ST_FOLLOW, now);
}
void beginJunction(uint32_t now) {
  jL = jR = jS = false; abTracking = false; endRolling = false; integ = 0;
  enter(ST_JUNCTION, now); driveMotors(T.creep, T.creep);
}
void beginTurn(int dir, uint32_t now, bool isU = false) {
  turnDir = dir; turnPhase = 0; leaveCnt = 0; acqCnt = 0; turnIsU = isU;
  enter(ST_TURN, now);
}
void beginSearch(int dir, uint32_t now) { searchDir = dir; searchAcq = 0; enter(ST_SEARCH, now); }

void decide(uint32_t now, uint8_t m) {     // Left Hand Rule  L > S > R > U, with loop breaker
  const char *order = "LSR";
  if (lapping()) {
    guardStart = turnCount;
    order = (loopBreaks % 2 == 0) ? "SRL" : "RSL";
    loopBreaks++;
  }
  for (int i = 0; i < 3; i++) {
    char c = order[i];
    bool ok = (c == 'L') ? jL : (c == 'R') ? jR : jS;
    if (!ok) continue;
    logTurn(c);
    if (c == 'L') beginTurn(1, now);
    else if (c == 'R') beginTurn(-1, now);
    else resumeFollowing(m, now);
    return;
  }
  logTurn('U'); beginTurn(1, now, true);   // dead end: pivot left
}

void turnDrive(int dir, int speed) {
  int inner = (int)(speed * T.turnInner);
  if (dir > 0) driveMotors(inner, speed); else driveMotors(speed, inner);
}

// ---------- the controller (one tick every 5 ms) ----------
void controlTick(uint8_t m, uint32_t now) {
  uint32_t el = now - stateSince;
  switch (state) {

    case ST_DONE:
      stopMotors();
      break;

    case ST_START:
      if (el < START_WAIT_MS) stopMotors();
      else if (m) resumeFollowing(m, now);
      else if (el - START_WAIT_MS > START_SEEK_MS) stopMotors();
      else driveMotors(T.creep, T.creep);
      break;

    case ST_FOLLOW: {
      if (popcount4(m) >= 3) { beginJunction(now); break; }      // wide pattern = junction / corner / end box
      if (m == 0) {                                              // line vanished
        if (!zeroTracking) { zeroTracking = true; zeroSince = now; }
        if (now - zeroSince >= LOST_CONFIRM_MS) {
          zeroTracking = false;
          if (fabsf(lastErr) < LOST_CENTERED) enter(ST_GAP, now);      // went straight into nothing: gap or dead end
          else beginSearch(lastErr > 0 ? 1 : -1, now);                 // ran off sideways: curve, search that way
        } else driveMotors(T.creep, T.creep);
        break;
      }
      zeroTracking = false;
      float e; lineError(m, e); lastErr = e;
      errF += ERR_EMA * (e - errF);
      float d = (errF - prevErrF) / DT; prevErrF = errF;
      integ = constrain(integ + errF * DT * T.ki, -I_CLAMP, I_CLAMP);          // anti-windup clamp
      float corr = constrain(T.kp * errF + integ + T.kd * d, -MAX_CORR, MAX_CORR);
      float base = T.cruise - (T.cruise - T.minSpeed) * fminf(1.0f, fabsf(errF) * SPEED_DROP);
      if (now < slowUntil) base = T.minSpeed;
      int b = (int)base, c = (int)corr;
      driveMotors(b - c, b + c);                                 // positive error (line left) -> left slower
      break;
    }

    case ST_GAP:                                                 // roll straight: a real gap closes, a dead end doesn't
      if (m) resumeFollowing(m, now);
      else if (el >= GAP_MS) { logTurn('U'); beginTurn(1, now, true); }
      else driveMotors(T.creep, T.creep);
      break;

    case ST_JUNCTION:
      if (endRolling) {                                          // end box confirmed: roll axle inside, stop
        if (now >= endUntil) { stopMotors(); enter(ST_DONE, now); doneReason = 'E'; }
        else driveMotors(T.creep, T.creep);
        break;
      }
      if (popcount4(m) >= 3) {                                   // a real bar crossing (drift of the main line only gives 2)
        if (m & 0b0001) jL = true;
        if (m & 0b1000) jR = true;
      }
      if (m == 0x0F) {
        if (!abTracking) { abTracking = true; abSince = now; }
        else if (now - abSince >= END_HOLD_MS) { endRolling = true; endUntil = now + END_ROLL_MS; break; }
      } else abTracking = false;
      if (el + STRAIGHT_WINDOW_MS >= T.advanceMs && (m & 0b0110)) jS = true;
      if (el >= T.advanceMs) decide(now, m);
      else driveMotors(T.creep, T.creep);
      break;

    case ST_TURN: {
      uint8_t centre = m & 0b0110;
      if (turnPhase == 0) {                                      // 1) leave the old line
        leaveCnt = (centre == 0) ? leaveCnt + 1 : 0;
        if (leaveCnt >= 3) turnPhase = 1;
      } else {                                                   // 2) acquire the new one (centre sensor on, outers clear)
        acqCnt = (centre != 0 && (m & 0b1001) == 0) ? acqCnt + 1 : 0;
        if (acqCnt >= 2) { resumeFollowing(m, now); break; }
      }
      if (el >= (turnIsU ? UTURN_TIMEOUT_MS : TURN_TIMEOUT_MS)) beginSearch(turnDir, now);
      else turnDrive(turnDir, T.turnSpeed);
      break;
    }

    case ST_SEARCH: {
      searchAcq = m ? searchAcq + 1 : 0;
      if (searchAcq >= 2) { resumeFollowing(m, now); break; }
      if (el >= SEARCH_TOTAL_MS) { stopMotors(); enter(ST_DONE, now); doneReason = 'L'; break; }
      int d = (el < SEARCH_FIRST_MS) ? searchDir : -searchDir;
      int sp = T.creep + 10;
      if (d > 0) driveMotors(-sp, sp); else driveMotors(sp, -sp);
      break;
    }
  }
}

// ---------- live tuning ----------
void saveTune() {
  prefs.begin("lfr", false);
  prefs.putBytes("tune", &T, sizeof(T));
  prefs.end();
}
void loadTune() {
  prefs.begin("lfr", true);
  if (prefs.getBytesLength("tune") == sizeof(T)) prefs.getBytes("tune", &T, sizeof(T));
  prefs.end();
}
void printTune() {
  say("kp=" + String(T.kp, 2) + " ki=" + String(T.ki, 2) + " kd=" + String(T.kd, 2) +
      " cruise=" + T.cruise + " min=" + T.minSpeed + " creep=" + T.creep + " turn=" + T.turnSpeed +
      " inner=" + String(T.turnInner, 2) + " adv=" + T.advanceMs);
}
void handleCommand(String line) {
  line.trim(); line.toLowerCase();
  if (!line.length()) return;
  int sp = line.indexOf(' ');
  String k = sp < 0 ? line : line.substring(0, sp);
  float v = sp < 0 ? 0 : line.substring(sp + 1).toFloat();
  if      (k == "kp") T.kp = v;       else if (k == "ki") T.ki = v;      else if (k == "kd") T.kd = v;
  else if (k == "cruise") T.cruise = constrain((int)v, 0, 255);
  else if (k == "min")    T.minSpeed = constrain((int)v, 0, 255);
  else if (k == "creep")  T.creep = constrain((int)v, 0, 255);
  else if (k == "turn")   T.turnSpeed = constrain((int)v, 0, 255);
  else if (k == "inner")  T.turnInner = constrain(v, -1.0f, 0.5f);
  else if (k == "adv")    T.advanceMs = (uint32_t)constrain(v, 100.0f, 3000.0f);
  else if (k == "telem")  telem = v != 0;
  else if (k == "save")   { saveTune(); say("saved"); return; }
  else if (k == "go")     { if (!running) { running = true; runStartMs = millis(); state = ST_START; stateSince = 0; } say("go"); return; }
  else if (k == "stop")   { stopMotors(); running = false; say("stopped"); return; }
  else if (k != "get")    { say("? kp ki kd cruise min creep turn inner adv telem save go stop get"); return; }
  printTune();
}
void pollCommands() {
  static String buf[2];
  Stream *ports[2] = {&Serial, &SerialBT};
  for (int p = 0; p < 2; p++) {
    while (ports[p]->available()) {
      char c = ports[p]->read();
      if (c == '\n' || c == '\r') { handleCommand(buf[p]); buf[p] = ""; }
      else if (buf[p].length() < 48) buf[p] += c;
    }
  }
}

// ---------- setup / loop ----------
void setup() {
  Serial.begin(115200);
  SerialBT.begin("LFR");
  loadTune();

  pinMode(STATUS_LED, OUTPUT); digitalWrite(STATUS_LED, LOW);
  pinMode(RED_LED, OUTPUT);    digitalWrite(RED_LED, LOW);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  digitalWrite(ENA, LOW); digitalWrite(ENB, LOW);

  if (!ledcAttach(ENA, PWM_FREQ, PWM_RESOLUTION) || !ledcAttach(ENB, PWM_FREQ, PWM_RESOLUTION)) {
    while (true) { digitalWrite(STATUS_LED, !digitalRead(STATUS_LED)); delay(200); }   // PWM init failed
  }
  stopMotors();
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);
  delay(300);

#if CALIBRATE_CREEP
  say("CALIBRATE: rolling at creep=" + String(T.creep) + " for 1000 ms in 3 s");
  delay(3000); driveMotors(T.creep, T.creep); delay(1000); stopMotors();
  say("Measure distance rolled (mm) = speed in mm/s.  advanceMs ~ 110000 / speed");
  while (true) delay(1000);
#endif
  printTune();
  say("send 'go' or wait " + String(AUTO_START_MS / 1000) + " s");
}

void loop() {
  pollCommands();
  if (!running) {
    if (millis() > AUTO_START_MS) { running = true; runStartMs = millis(); state = ST_START; stateSince = 0; }
    return;
  }
  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastControlUs) < CONTROL_PERIOD_US) return;
  lastControlUs = nowUs;
  uint32_t now = millis() - runStartMs;

  uint8_t raw;
  if (!readSensorState(raw)) {                                   // I2C fault: stop and blink until it returns
    stopMotors(); sensorFault = true; filtInit = false;
    digitalWrite(STATUS_LED, ((now / 150) % 2) ? HIGH : LOW);
    return;
  }
  if (sensorFault) { sensorFault = false; digitalWrite(STATUS_LED, LOW); enter(ST_START, now); }

  uint8_t m = filterMask(getLineMask(raw));
  State before = state;
  controlTick(m, now);

  if (state == ST_DONE && before != ST_DONE) {                   // announce result once
    if (doneReason == 'E') {
      digitalWrite(RED_LED, HIGH);
      say("END BOX reached. path   = " + String(turnLog).substring(0, turnCount));
      say("                simplified = " + simplifiedPath());
    } else {
      say("LINE LOST. path so far = " + String(turnLog).substring(0, turnCount));
    }
  }
  if (state == ST_DONE && doneReason == 'L') digitalWrite(STATUS_LED, ((now / 300) % 2) ? HIGH : LOW);

  if (telem) {
    static uint32_t lastT = 0;
    if (now - lastT >= 50) { lastT = now; say(String(now) + "," + String(m, BIN) + "," + (int)state + "," + String(errF, 2) + "," + lastL + "," + lastR); }
  }
}
