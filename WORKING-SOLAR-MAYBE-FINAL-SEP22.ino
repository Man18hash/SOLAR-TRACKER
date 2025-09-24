#include <Servo.h>
#include <EEPROM.h>

/* ====== CONFIG ====== */
#define HAS_FEEDBACK true   // actuator has feedback pot on A4

/* ----- Pins (your wiring) ----- */
// LDRs
const uint8_t PIN_UP    = A0;
const uint8_t PIN_DOWN  = A1;
const uint8_t PIN_LEFT  = A2;
const uint8_t PIN_RIGHT = A3;

// Actuator feedback
const uint8_t PIN_ACT_FB = A4;   // actuator pot signal

// BTS7960 (EN pins tied to +5V)
const uint8_t PIN_RPWM = 5;      // extend (increase %)
const uint8_t PIN_LPWM = 6;      // retract (decrease %)

// Pan servo
const uint8_t PIN_SERVO_PAN = 9;

/* ----- Tracker tuning ----- */
// LDR smoothing (moving average)
const uint8_t AVG_SAMPLES = 5;     // <-- you requested 5
const int     DEAD_BAND   = 16;    // ignore tiny differences
const int     HYSTERESIS  = 6;     // extra margin to prevent ping-pong
const int     LOW_LIGHT_THR = 80;  // too dark -> hold
const int     LOOP_DELAY_MS = 50;

// Servo pan limits & rate
const int SERVO_MIN = 20, SERVO_MAX = 160;

// Soft-start/anti-jerk
const unsigned long ARM_DELAY_MS = 1200;     // keep servo DETACHED this long
const unsigned long SAFE_START_MS = 2000;    // be extra conservative early

// ===== CHANGED: make max pan speed 2°/s (was 6°/s) =====
const float PAN_DEG_PER_SEC_MAX = 2.0f;      // final speed cap (slow)
const float PAN_SPEED_RAMP      = 1.5f;      // deg/s^2 ramp after attach (gentle)
// Keep the same proportional step gain; cap enforces the 2°/s limit
const float PAN_STEP_GAIN       = 0.016f;    // converts L/R diff → deg step

/* ----- Actuator feedback calibration ----- */
int ACT_FB_MIN = 200;   // analog @ fully retracted  (EDIT with your real value)
int ACT_FB_MAX = 800;   // analog @ fully extended   (EDIT with your real value)

// Hard position limits
const int MIN_TILT_PCT = 0;
const int MAX_TILT_PCT = 70;     // <-- never exceed 70%

/* Actuator control (smooth) */
const int ACT_TOLERANCE   = 1;    // acceptable error margin in %
const int ACT_SPEED_MIN   = 45;   // lowest PWM
const int ACT_SPEED_MAX   = 180;  // highest PWM (gentle)
const uint16_t ACT_SETTLE_MS = 250; // must be stable in tolerance this long
const float TILT_SLEW_PCT_PER_S = 16.0f; // how fast target may change (%/s)
const float FB_LP_ALPHA = 0.18f;  // feedback low-pass (0..1)

/* ====== EEPROM (remember last angle) ====== */
const int EE_ADDR_ANGLE = 0; // stores float (4 bytes)

void saveAngle(float a){ EEPROM.put(EE_ADDR_ANGLE, a); }
float loadAngle(){
  float a = 90.0f;
  EEPROM.get(EE_ADDR_ANGLE, a);
  if (isnan(a) || a < SERVO_MIN || a > SERVO_MAX) a = 90.0f;
  return a;
}

/* ====== Globals ====== */
Servo servoPan;
bool servoAttached = false;  // we will attach after ARM_DELAY_MS
float panAngle = 90.0f;      // will be overwritten by EEPROM

// soft-start speed cap
float panSpeedCap = 0.0f;

// moving averages for LDRs
int upBuf[AVG_SAMPLES], dnBuf[AVG_SAMPLES], lfBuf[AVG_SAMPLES], rtBuf[AVG_SAMPLES];
uint8_t upIdx=0, dnIdx=0, lfIdx=0, rtIdx=0, filled=0;
long upSum=0, dnSum=0, lfSum=0, rtSum=0;

// actuator targets / feedback
float actTargetPct = 50.0f;      // commanded target (%)
float actSlewPct   = 50.0f;      // slew-limited target
float fbPctFilt    = 50.0f;      // filtered feedback (%)
unsigned long settleSince = 0;
unsigned long tPrev = 0, tPrint = 0;

/* ====== Helpers ====== */
inline void actStop() { analogWrite(PIN_RPWM, 0); analogWrite(PIN_LPWM, 0); }

int actFbToPercent(int fb) {
  int lo = min(ACT_FB_MIN, ACT_FB_MAX);
  int hi = max(ACT_FB_MIN, ACT_FB_MAX);
  fb = constrain(fb, lo, hi);
  long pct = map(fb, ACT_FB_MIN, ACT_FB_MAX, 0, 100);
  return (int)constrain(pct, 0, 100);
}

void servoAttachIfNeeded() {
  if (!servoAttached && millis() >= ARM_DELAY_MS) {
    servoPan.attach(PIN_SERVO_PAN);           // attach AFTER delay → no early pulses
    servoPan.write((int)roundf(panAngle));    // command the SAME angle we loaded
    servoAttached = true;
    panSpeedCap = 0.0f;                       // start ramp from 0
  }
}

void servoWriteSafe(float deg) {
  deg = constrain(deg, (float)SERVO_MIN, (float)SERVO_MAX);
  if (fabs(deg - panAngle) >= 0.1f) {
    panAngle = deg;
    if (servoAttached) servoPan.write((int)roundf(panAngle)); // only send pulses after attach
  }
}

int pushAvg(int val, int* buf, long& sum, uint8_t& idx) {
  if (filled < AVG_SAMPLES) {
    buf[filled] = val; sum += val;
    return (int)(sum / (filled + 1)); // will increment filled after all sensors seeded
  } else {
    sum -= buf[idx];
    buf[idx] = val;
    sum += val;
    idx++; if (idx >= AVG_SAMPLES) idx = 0;
    return (int)(sum / AVG_SAMPLES);
  }
}

bool bigImbalance(int diff, int margin){
  // During early startup, demand a larger imbalance to move
  if (millis() < SAFE_START_MS) return abs(diff) > (margin * 2);
  return abs(diff) > margin;
}

/* Pan control: move slowly until balanced, then stop (with soft-start ramp) */
void panControlStep(float dt_s, int diffLR, int maxLight) {
  // no movement until we've actually attached (after ARM_DELAY_MS)
  if (!servoAttached) return;

  // ramp the speed cap up to its max
  panSpeedCap += PAN_SPEED_RAMP * dt_s;
  if (panSpeedCap > PAN_DEG_PER_SEC_MAX) panSpeedCap = PAN_DEG_PER_SEC_MAX;

  if (maxLight <= LOW_LIGHT_THR) return; // too dark → hold

  int margin = DEAD_BAND + HYSTERESIS;
  if (!bigImbalance(diffLR, margin)) return; // conservative at startup

  float step = PAN_STEP_GAIN * abs(diffLR);
  float maxStep = panSpeedCap * dt_s;        // <= 2°/s once fully ramped
  if (step > maxStep) step = maxStep;

  float newDeg = panAngle + (diffLR > 0 ? +step : -step);
  servoWriteSafe(newDeg);
}

/* Smooth actuator move toward actTargetPct (unchanged behavior, just safer start) */
void actDriveSmooth(float dt_s) {
  // hard clamp target and slew it
  actTargetPct = constrain(actTargetPct, (float)MIN_TILT_PCT, (float)MAX_TILT_PCT);
  float maxStep = TILT_SLEW_PCT_PER_S * dt_s;
  float d = actTargetPct - actSlewPct;
  if (d >  maxStep) d =  maxStep;
  if (d < -maxStep) d = -maxStep;
  actSlewPct += d;

  // read & filter feedback
  int fbRaw = analogRead(PIN_ACT_FB);
  float fbPct = (float)actFbToPercent(fbRaw);
  fbPctFilt = (1.0f - FB_LP_ALPHA) * fbPctFilt + FB_LP_ALPHA * fbPct;

  // control error
  float err = actSlewPct - fbPctFilt;

  // settle window (prevents chattering)
  if (fabs(err) <= (float)ACT_TOLERANCE) {
    if (settleSince == 0) settleSince = millis();
    if (millis() - settleSince >= ACT_SETTLE_MS) { actStop(); return; }
  } else {
    settleSince = 0;
  }

  // gentle PWM shaping by error
  int pwm = (int)(ACT_SPEED_MIN + min(fabs(err)*3.5f, 1.0f) * (ACT_SPEED_MAX - ACT_SPEED_MIN));
  pwm = constrain(pwm, ACT_SPEED_MIN, ACT_SPEED_MAX);

  if (err > 0) {         // need to extend (increase %)
    analogWrite(PIN_RPWM, pwm);
    analogWrite(PIN_LPWM, 0);
  } else {               // need to retract (decrease %)
    analogWrite(PIN_RPWM, 0);
    analogWrite(PIN_LPWM, pwm);
  }
}

/* ====== Setup ====== */
void setup() {
  Serial.begin(115200);

  // DO NOT attach servo yet (prevents any startup twitch)
  servoAttached = false;

  pinMode(PIN_RPWM, OUTPUT);
  pinMode(PIN_LPWM, OUTPUT);
  actStop();

  delay(150); // tiny settle for ADC

  // seed moving averages from first raw reads
  int up0 = analogRead(PIN_UP);
  int dn0 = analogRead(PIN_DOWN);
  int lf0 = analogRead(PIN_LEFT);
  int rt0 = analogRead(PIN_RIGHT);
  for (uint8_t i=0;i<AVG_SAMPLES;i++){ upBuf[i]=up0; dnBuf[i]=dn0; lfBuf[i]=lf0; rtBuf[i]=rt0; }
  upSum = (long)up0*AVG_SAMPLES; dnSum = (long)dn0*AVG_SAMPLES;
  lfSum = (long)lf0*AVG_SAMPLES; rtSum = (long)rt0*AVG_SAMPLES;
  filled = AVG_SAMPLES;

  // initialize actuator to current position (and clamp)
  if (HAS_FEEDBACK) {
    int fb = analogRead(PIN_ACT_FB);
    int pct = actFbToPercent(fb);
    actTargetPct = constrain((float)pct, (float)MIN_TILT_PCT, (float)MAX_TILT_PCT);
    actSlewPct   = actTargetPct;
    fbPctFilt    = actTargetPct;

    Serial.print("Boot actuator raw="); Serial.print(fb);
    Serial.print(" -> "); Serial.print(pct); Serial.println("% (clamped to 0..70%)");
  }

  // Load last saved pan angle; keep servo DETACHED for now
  panAngle = loadAngle();
  Serial.print("Pan last angle = "); Serial.println(panAngle, 1);

  tPrev = millis();
  tPrint = millis();
  Serial.println("Tracker ready (soft-start enabled, pan speed = 2°/s).");
}

/* ====== Loop ====== */
void loop() {
  unsigned long tNow = millis();
  float dt_s = (tNow - tPrev) / 1000.0f;
  if (dt_s <= 0) dt_s = 0.001f; // guard
  tPrev = tNow;

  // Attach servo only after delay to prevent startup twitch
  servoAttachIfNeeded();

  // moving averages (5-sample window) for each sensor
  int upAvg = pushAvg(analogRead(PIN_UP),   upBuf, upSum, upIdx);
  int dnAvg = pushAvg(analogRead(PIN_DOWN), dnBuf, dnSum, dnIdx);
  int lfAvg = pushAvg(analogRead(PIN_LEFT), lfBuf, lfSum, lfIdx);
  int rtAvg = pushAvg(analogRead(PIN_RIGHT),rtBuf, rtSum, rtIdx);

  // compute diffs (direction cues)
  int diffLR = rtAvg - lfAvg;  // + → brighter RIGHT
  int diffUD = upAvg - dnAvg;  // + → brighter UP
  int maxLight = max(max(upAvg, dnAvg), max(lfAvg, rtAvg));
  int margin = DEAD_BAND + HYSTERESIS;

  // ---------- SERVO PAN: move toward left/right brightest, stop when balanced
  panControlStep(dt_s, diffLR, maxLight);

  // ---------- ACTUATOR TILT: move toward up/down brightest, stop when balanced
  if (maxLight <= LOW_LIGHT_THR) {
    // too dark -> hold & align target to current feedback so we don't jump later
    int fbRaw = analogRead(PIN_ACT_FB);
    float currPct = (float)actFbToPercent(fbRaw);
    actTargetPct = constrain(currPct, (float)MIN_TILT_PCT, (float)MAX_TILT_PCT);
  } else {
    if (bigImbalance(diffUD, margin)) {
      // nudge target toward the brighter side
      float k = 0.012f; // small = smoother
      actTargetPct += (diffUD > 0 ? +1.0f : -1.0f) * k * (abs(diffUD));
    }
    // always clamp to 0..70%
    actTargetPct = constrain(actTargetPct, (float)MIN_TILT_PCT, (float)MAX_TILT_PCT);
  }

  // drive actuator smoothly toward target (with settle stop)
  actDriveSmooth(dt_s);

  // ----- Debug every 600 ms -----
  if (millis() - tPrint > 600) {
    tPrint = millis();
    int fb = analogRead(PIN_ACT_FB);
    int pct = actFbToPercent(fb);
    Serial.print("U="); Serial.print(upAvg);
    Serial.print(" D="); Serial.print(dnAvg);
    Serial.print(" L="); Serial.print(lfAvg);
    Serial.print(" R="); Serial.print(rtAvg);
    Serial.print(" | pan="); Serial.print((int)panAngle);
    Serial.print(" | tilt tgt="); Serial.print((int)actTargetPct);
    Serial.print("% slew="); Serial.print((int)actSlewPct);
    Serial.print("% fb="); Serial.print(pct);
    Serial.print("% (raw="); Serial.print(fb); Serial.println(")");
  }

  // persist angle occasionally so next boot resumes here
  static unsigned long tEE = 0;
  if (millis() - tEE > 1500) { saveAngle(panAngle); tEE = millis(); }

  delay(LOOP_DELAY_MS);
}