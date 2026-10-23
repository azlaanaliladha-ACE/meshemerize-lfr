#include <Wire.h>
#include <math.h>

// =====================================================
// ESP32 + MAKEBLOCK RGB LINE FOLLOWER + L298N
// Accuracy-first state-machine controller
// =====================================================

// ---------- I2C SENSOR ----------
#define SENSOR_ADDR  0x20
#define STATE_REG    0x07

#define I2C_SDA_PIN  21
#define I2C_SCL_PIN  22

// Confirmed raw readings:
// All white          = 1111
// R2 and R3 on black = 1001
// All black          = 0000

const uint8_t WHITE_BACKGROUND_STATE = 0x0F;

// ---------- L298N MOTOR PINS ----------
#define ENA 25       // Right motor PWM
#define ENB 13       // Left motor PWM

#define IN1 26       // Right motor direction
#define IN2 27

#define IN3 33       // Left motor direction
#define IN4 32

#define STATUS_LED 2

// Keep the motor directions that work on your rover.
const bool RIGHT_MOTOR_INVERTED = true;
const bool LEFT_MOTOR_INVERTED  = true;

// ---------- ESP32 PWM ----------
const int PWM_FREQ = 1000;
const int PWM_RESOLUTION = 8;
const int MAX_PWM = 255;

// =====================================================
// SPEED SETTINGS
// =====================================================

const int BASE_SPEED = 105;       // was 130 (slower)
const int MIN_BASE_SPEED = 65;

const int INTERSECTION_SPEED = 60;
const int EDGE_CREEP_SPEED = 60;

const float SPEED_REDUCTION_PER_ERROR = 10.0f;

// =====================================================
// PD STEERING SETTINGS
// =====================================================

// KI is intentionally omitted. Start with PD steering
// to reduce overshoot while tuning the mechanical setup.

const float KP = 25.0f;
const float KD = 3.0f;

const float MAX_CORRECTION = 95.0f;

float previousError = 0.0f;

// =====================================================
// TURN SETTINGS
// =====================================================

// An outer-only pattern must persist before pivoting.
const uint32_t EDGE_CONFIRM_MS = 20;

// Pivot until BOTH center sensors find the line.
const int SHARP_TURN_SPEED = 75;   // was 85
const uint32_t TURN_TIMEOUT_MS = 1200;

// Extra bounded search if the first pivot times out.
const uint32_t TURN_EXTRA_SEARCH_MS = 400;

// =====================================================
// LINE-LOSS RECOVERY
// =====================================================

const int SEARCH_SPEED = 60;        // was 65
const uint32_t SEARCH_TIME_MS = 500;

// =====================================================
// OVERSHOOT CONTROL  (ideas from avro1199 LFR code)
// =====================================================

// Speed ramp: build up speed only while the line stays centred,
// drop straight back to MIN_BASE_SPEED on a big deviation.
const float RAMP_PER_TICK = 0.12f;       // PWM added per 5 ms tick while |error| <= 1
float cruiseSpeed = MIN_BASE_SPEED;

// Short reverse pulse to kill momentum before a pivot / stop.
const int BRAKE_SPEED = 120;
const uint32_t BRAKE_MS = 30;

// Short opposite pivot after a turn finishes, to cancel spin momentum.
const uint32_t COUNTER_PIVOT_MS = 25;

// =====================================================
// LINE-LOSS MEMORY  (ideas from amjed-ali-k ESP-32-Line-Follower)
// =====================================================

// Remember the last 10 errors; when the line vanishes, the weighted
// history says which side it left from.
const uint8_t HISTORY_SIZE = 10;
float errorHistory[HISTORY_SIZE] = {0};
uint8_t errorHistoryIndex = 0;

// When the line vanishes, keep rolling straight this long before sweeping
// (bridges small gaps / noise).
const uint32_t NO_LINE_GRACE_MS = 80;
bool lostTracking = false;
uint32_t lostSince = 0;
bool sweepLeftFirst = true;

// =====================================================
// ALL-BLACK STOP / ALL-WHITE SWEEP
// =====================================================

// All four sensors black for this long = stop for good.
// (Long enough to ignore a normal crossing bar, short enough to stop inside the box.)
const uint32_t ALL_BLACK_STOP_MS = 400;   // must be longer than a crossing bar takes to pass

// All white: pivot LEFT, then RIGHT, then LEFT ... slowly, each swing longer than the last,
// until the line is found. Breaks the robot out of loops / gaps.
const uint32_t SWEEP_STEP_MS  = 300;    // swing k lasts STEP*(k+1)
const uint32_t SWEEP_MAX_SWING_MS = 1500;
const uint32_t SWEEP_TOTAL_MS = 10000;  // give up and stop after this

// =====================================================
// CONTROL LOOP AND FILTERING
// =====================================================

const uint32_t CONTROL_PERIOD_US = 5000; // 5 ms, 200 Hz

const uint8_t CENTER_MASK = 0b0110;
const uint8_t LEFT_EDGE_MASK = 0b0001;
const uint8_t RIGHT_EDGE_MASK = 0b1000;
const uint8_t OUTER_PAIR_MASK = 0b1001;
const uint8_t ALL_SENSORS_MASK = 0b1111;

uint8_t maskHistory[3] = {0, 0, 0};
bool filterInitialized = false;

// =====================================================
// CONTROLLER STATE
// =====================================================

enum RunState {
  FOLLOWING,
  TURNING_LEFT,
  TURNING_RIGHT,
  SEARCHING_LEFT,
  SEARCHING_RIGHT,
  LOST_STOPPED,
  SWEEPING,
  FINISHED,
  SENSOR_FAULT
};

RunState runState = FOLLOWING;

uint32_t stateStartedAt = 0;
uint32_t lastControlUs = 0;

uint8_t centerStreak = 0;
uint8_t reacquireStreak = 0;

bool turnArmed = true;
bool searchRequiresCenter = false;

uint32_t currentSearchTimeout = 0;

// +1 = last line deviation was left
// -1 = last line deviation was right
int lastTurnDirection = 0;

// All-black stop tracking
bool allBlackTracking = false;
uint32_t allBlackSince = 0;

// All-white sweep
uint8_t sweepSwing = 0;
uint32_t sweepSwingStart = 0;

// Candidate outer sensor pattern
uint8_t edgeCandidate = 0;
uint32_t edgeCandidateSince = 0;

// =====================================================
// SENSOR READING
// =====================================================

bool readSensorState(uint8_t &state) {
  Wire.beginTransmission(SENSOR_ADDR);
  Wire.write(STATE_REG);

  if (Wire.endTransmission() != 0) {
    return false;
  }

  if (Wire.requestFrom(
        (uint8_t)SENSOR_ADDR,
        (uint8_t)1) != 1) {
    return false;
  }

  state = Wire.read() & 0x0F;
  return true;
}

// Converts active-low raw sensor states into a mask
// in which 1 means the sensor detects black.
//
// R1: bit 0, leftmost
// R2: bit 1
// R3: bit 2
// R4: bit 3, rightmost

uint8_t getLineMask(uint8_t rawState) {
  return (WHITE_BACKGROUND_STATE ^ rawState) & 0x0F;
}

// Three-sample majority filter, independently per bit.
uint8_t filterLineMask(uint8_t newMask) {
  if (!filterInitialized) {
    maskHistory[0] = newMask;
    maskHistory[1] = newMask;
    maskHistory[2] = newMask;

    filterInitialized = true;
    return newMask;
  }

  maskHistory[0] = maskHistory[1];
  maskHistory[1] = maskHistory[2];
  maskHistory[2] = newMask;

  return (
      (maskHistory[0] & maskHistory[1])
    | (maskHistory[0] & maskHistory[2])
    | (maskHistory[1] & maskHistory[2])
  ) & 0x0F;
}

bool centerPairDetected(uint8_t mask) {
  return (mask & CENTER_MASK) == CENTER_MASK;
}

// =====================================================
// MOTOR CONTROL
// =====================================================

void setMotor(
  int pwmPin,
  int inA,
  int inB,
  int speed,
  bool inverted
) {
  speed = constrain(speed, -MAX_PWM, MAX_PWM);

  if (speed == 0) {
    ledcWrite(pwmPin, 0);
    digitalWrite(inA, LOW);
    digitalWrite(inB, LOW);
    return;
  }

  bool forward = (speed > 0);

  if (inverted) {
    forward = !forward;
  }

  if (forward) {
    digitalWrite(inA, HIGH);
    digitalWrite(inB, LOW);
  } else {
    digitalWrite(inA, LOW);
    digitalWrite(inB, HIGH);
  }

  ledcWrite(pwmPin, abs(speed));
}

// Arguments are left speed, then right speed.
void driveMotors(int leftSpeed, int rightSpeed) {
  setMotor(
    ENA, IN1, IN2,
    rightSpeed, RIGHT_MOTOR_INVERTED
  );

  setMotor(
    ENB, IN3, IN4,
    leftSpeed, LEFT_MOTOR_INVERTED
  );
}

void stopMotors() {
  ledcWrite(ENA, 0);
  ledcWrite(ENB, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void pivotLeft(int speed) {
  driveMotors(-speed, speed);
}

void pivotRight(int speed) {
  driveMotors(speed, -speed);
}

// =====================================================
// LINE POSITION CALCULATION
// =====================================================

// Positive error: line lies to the left.
// Negative error: line lies to the right.
//
// R1 only = +3
// R2 only = +1
// R3 only = -1
// R4 only = -3
// R2 + R3 = 0

float calculateLineError(uint8_t lineMask) {
  const int positions[4] = {3, 1, -1, -3};

  int weightedSum = 0;
  int activeCount = 0;

  for (int i = 0; i < 4; i++) {
    if (lineMask & (1 << i)) {
      weightedSum += positions[i];
      activeCount++;
    }
  }

  if (activeCount == 0) {
    return 999.0f;
  }

  return (float)weightedSum / activeCount;
}

// =====================================================
// HELPERS: BRAKE, HISTORY
// =====================================================

void brakePulse() {
  driveMotors(-BRAKE_SPEED, -BRAKE_SPEED);
  delay(BRAKE_MS);
  stopMotors();
}

void pushErrorHistory(float e) {
  errorHistory[errorHistoryIndex] = e;
  errorHistoryIndex = (errorHistoryIndex + 1) % HISTORY_SIZE;
}

// +1 = line was last on the left, -1 = right, 0 = unknown.
int historyDirection() {
  float sum = 0.0f;
  for (uint8_t k = 0; k < HISTORY_SIZE; k++) {
    // k = 0 is the newest sample; newer samples weigh more.
    uint8_t idx = (errorHistoryIndex + HISTORY_SIZE - 1 - k) % HISTORY_SIZE;
    sum += errorHistory[idx] * (float)(HISTORY_SIZE - k);
  }
  if (sum > 3.0f) return 1;
  if (sum < -3.0f) return -1;
  return lastTurnDirection;
}

// =====================================================
// STATE TRANSITIONS
// =====================================================

void enterState(RunState nextState, uint32_t now) {
  runState = nextState;
  stateStartedAt = now;

  centerStreak = 0;
  reacquireStreak = 0;
}

void beginTurn(bool turnLeft, uint32_t now) {
  lastTurnDirection = turnLeft ? 1 : -1;

  turnArmed = false;
  edgeCandidate = 0;

  if (turnLeft) {
    enterState(TURNING_LEFT, now);
  } else {
    enterState(TURNING_RIGHT, now);
  }
}

void beginSearch(
  bool searchLeft,
  bool requireCenter,
  uint32_t timeoutMs,
  uint32_t now
) {
  lastTurnDirection = searchLeft ? 1 : -1;

  turnArmed = false;
  edgeCandidate = 0;

  searchRequiresCenter = requireCenter;
  currentSearchTimeout = timeoutMs;

  if (searchLeft) {
    enterState(SEARCHING_LEFT, now);
  } else {
    enterState(SEARCHING_RIGHT, now);
  }
}

void beginSweep(uint32_t now) {
  turnArmed = false;
  edgeCandidate = 0;
  sweepSwing = 0;
  sweepSwingStart = now;
  sweepLeftFirst = (historyDirection() >= 0);   // first swing goes where the line was last seen
  enterState(SWEEPING, now);
}

void resumeFollowing(uint8_t mask, uint32_t now) {
  enterState(FOLLOWING, now);

  previousError = calculateLineError(mask);
  if (previousError == 999.0f) {
    previousError = 0.0f;
  }

  turnArmed = centerPairDetected(mask);
  edgeCandidate = 0;
  cruiseSpeed = MIN_BASE_SPEED;
  lostTracking = false;

  stopMotors();
}

// =====================================================
// NORMAL PD LINE FOLLOWING
// =====================================================

void followNormally(uint8_t mask, uint32_t now) {
  // No sensor sees black: start a bounded search.
  if (mask == 0) {
    allBlackTracking = false;
    if (!lostTracking) {
      lostTracking = true;
      lostSince = now;
    }
    if ((uint32_t)(now - lostSince) < NO_LINE_GRACE_MS) {
      driveMotors(INTERSECTION_SPEED, INTERSECTION_SPEED);   // bridge a tiny gap
      return;
    }
    lostTracking = false;
    beginSweep(now);
    return;
  }
  lostTracking = false;

  // All black held long enough: stop and stay stopped.
  if (mask == ALL_SENSORS_MASK) {
    if (!allBlackTracking) {
      allBlackTracking = true;
      allBlackSince = now;
    } else if ((uint32_t)(now - allBlackSince) >= ALL_BLACK_STOP_MS) {
      brakePulse();
      enterState(FINISHED, now);
      return;
    }
    driveMotors(INTERSECTION_SPEED, INTERSECTION_SPEED);  // creep while confirming
    return;
  }
  allBlackTracking = false;

  // The line is directly under both center sensors:
  // allow turn detection to be armed again.
  if (centerPairDetected(mask)) {
    turnArmed = true;
    edgeCandidate = 0;
  }

  // Intersections and ambiguous broad markings.
  // This controller's default policy is to proceed slowly.
  if (mask == ALL_SENSORS_MASK ||
      mask == OUTER_PAIR_MASK) {

    edgeCandidate = 0;
    previousError = 0.0f;

    if (mask == ALL_SENSORS_MASK) {
      turnArmed = true;
    }

    driveMotors(
      INTERSECTION_SPEED,
      INTERSECTION_SPEED
    );

    return;
  }

  // ---------- CONFIRMED SHARP TURN DETECTION ----------
  if (turnArmed &&
      (mask == LEFT_EDGE_MASK ||
       mask == RIGHT_EDGE_MASK)) {

    if (edgeCandidate != mask) {
      edgeCandidate = mask;
      edgeCandidateSince = now;

      lastTurnDirection =
          (mask == LEFT_EDGE_MASK) ? 1 : -1;

    } else if (
      (uint32_t)(now - edgeCandidateSince)
      >= EDGE_CONFIRM_MS
    ) {
      brakePulse();

      beginTurn(
        mask == LEFT_EDGE_MASK,
        now
      );

      // Begin the pivot immediately.
      if (mask == LEFT_EDGE_MASK) {
        pivotLeft(SHARP_TURN_SPEED);
      } else {
        pivotRight(SHARP_TURN_SPEED);
      }

      return;
    }

  } else {
    edgeCandidate = 0;
  }

  // ---------- PD STEERING ----------
  float error = calculateLineError(mask);

  if (error == 999.0f) {
    stopMotors();
    return;
  }

  if (error > 0.1f) {
    lastTurnDirection = 1;
  } else if (error < -0.1f) {
    lastTurnDirection = -1;
  }

  pushErrorHistory(error);

  float derivative = error - previousError;

  float correction =
      KP * error
    + KD * derivative;

  correction = constrain(
    correction,
    -MAX_CORRECTION,
    MAX_CORRECTION
  );

  previousError = error;

  // Reduce speed as the line moves farther from center.
  // Speed ramp: accelerate only while the line is centred.
  if (fabsf(error) <= 1.0f) {
    cruiseSpeed += RAMP_PER_TICK;
    if (cruiseSpeed > BASE_SPEED) cruiseSpeed = BASE_SPEED;
  } else if (fabsf(error) >= 3.0f) {
    cruiseSpeed = MIN_BASE_SPEED;
  }

  int baseSpeed = (int)cruiseSpeed -
      (int)(fabsf(error) * SPEED_REDUCTION_PER_ERROR);

  baseSpeed = constrain(
    baseSpeed,
    MIN_BASE_SPEED,
    BASE_SPEED
  );

  // Positive error = turn left.
  int leftSpeed = (int)(baseSpeed - correction);
  int rightSpeed = (int)(baseSpeed + correction);

  leftSpeed = constrain(leftSpeed, -MAX_PWM, MAX_PWM);
  rightSpeed = constrain(rightSpeed, -MAX_PWM, MAX_PWM);

  driveMotors(leftSpeed, rightSpeed);
}

// =====================================================
// STATE-MACHINE CONTROLLER
// =====================================================

void processLineMask(uint8_t mask, uint32_t now) {

  switch (runState) {

    // -------------------------------------------------
    // NORMAL TRACKING
    // -------------------------------------------------

    case FOLLOWING:
      followNormally(mask, now);
      break;

    // -------------------------------------------------
    // SHARP LEFT TURN
    // -------------------------------------------------

    case TURNING_LEFT:

      if (centerPairDetected(mask)) {
        if (centerStreak < 255) {
          centerStreak++;
        }
      } else {
        centerStreak = 0;
      }

      // Require three consecutive filtered readings
      // with both center sensors detecting black.
      if (centerStreak >= 3) {
        pivotRight(SHARP_TURN_SPEED);          // cancel spin momentum
        delay(COUNTER_PIVOT_MS);
        resumeFollowing(mask, now);
        turnArmed = true;
        return;
      }

      if ((uint32_t)(now - stateStartedAt)
          >= TURN_TIMEOUT_MS) {

        // Continue searching left for a bounded time.
        beginSearch(
          true,
          true,
          TURN_EXTRA_SEARCH_MS,
          now
        );

        return;
      }

      pivotLeft(SHARP_TURN_SPEED);
      break;

    // -------------------------------------------------
    // SHARP RIGHT TURN
    // -------------------------------------------------

    case TURNING_RIGHT:

      if (centerPairDetected(mask)) {
        if (centerStreak < 255) {
          centerStreak++;
        }
      } else {
        centerStreak = 0;
      }

      if (centerStreak >= 3) {
        pivotLeft(SHARP_TURN_SPEED);           // cancel spin momentum
        delay(COUNTER_PIVOT_MS);
        resumeFollowing(mask, now);
        turnArmed = true;
        return;
      }

      if ((uint32_t)(now - stateStartedAt)
          >= TURN_TIMEOUT_MS) {

        beginSearch(
          false,
          true,
          TURN_EXTRA_SEARCH_MS,
          now
        );

        return;
      }

      pivotRight(SHARP_TURN_SPEED);
      break;

    // -------------------------------------------------
    // SEARCHING AFTER LINE LOSS OR TURN FAILURE
    // -------------------------------------------------

    case SEARCHING_LEFT:
    case SEARCHING_RIGHT: {
      bool searchLeft = (runState == SEARCHING_LEFT);

      if (searchRequiresCenter) {
        // After a sharp turn, require both center
        // sensors to detect black consistently.
        if (centerPairDetected(mask)) {
          if (centerStreak < 255) {
            centerStreak++;
          }
        } else {
          centerStreak = 0;
        }

        if (centerStreak >= 3) {
          resumeFollowing(mask, now);
          turnArmed = true;
          return;
        }

      } else {
        // During ordinary line-loss recovery, any
        // stable nonzero line mask permits reacquisition.
        if (mask != 0) {
          if (reacquireStreak < 255) {
            reacquireStreak++;
          }
        } else {
          reacquireStreak = 0;
        }

        if (reacquireStreak >= 3) {
          resumeFollowing(mask, now);

          // If only an outer sensor is detected, do not
          // instantly retrigger the same sharp turn.
          turnArmed = centerPairDetected(mask);
          return;
        }
      }

      // Give up after the configured search window.
      if ((uint32_t)(now - stateStartedAt)
          >= currentSearchTimeout) {

        stopMotors();

        enterState(LOST_STOPPED, now);
        turnArmed = false;

        return;
      }

      // Continue pivoting in the appropriate direction.
      if (searchLeft) {
        int speed = searchRequiresCenter
            ? SHARP_TURN_SPEED
            : SEARCH_SPEED;

        pivotLeft(speed);

      } else {
        int speed = searchRequiresCenter
            ? SHARP_TURN_SPEED
            : SEARCH_SPEED;

        pivotRight(speed);
      }

      break;
    }

    // -------------------------------------------------
    // ALL WHITE: SLOW ALTERNATING LEFT / RIGHT SWEEP
    // -------------------------------------------------

    case SWEEPING: {
      if (mask != 0) {
        if (reacquireStreak < 255) {
          reacquireStreak++;
        }
      } else {
        reacquireStreak = 0;
      }

      if (reacquireStreak >= 3) {
        resumeFollowing(mask, now);
        turnArmed = centerPairDetected(mask);
        return;
      }

      if ((uint32_t)(now - stateStartedAt) >= SWEEP_TOTAL_MS) {
        stopMotors();
        enterState(LOST_STOPPED, now);
        turnArmed = false;
        return;
      }

      uint32_t swingMs = SWEEP_STEP_MS * (uint32_t)(sweepSwing + 1);
      if (swingMs > SWEEP_MAX_SWING_MS) swingMs = SWEEP_MAX_SWING_MS;

      if ((uint32_t)(now - sweepSwingStart) >= swingMs) {
        sweepSwing++;
        sweepSwingStart = now;
      }

      if (((sweepSwing % 2) == 0) == sweepLeftFirst) {
        pivotLeft(SEARCH_SPEED);
      } else {
        pivotRight(SEARCH_SPEED);
      }

      break;
    }

    // -------------------------------------------------
    // ALL BLACK: STOPPED FOR GOOD
    // -------------------------------------------------

    case FINISHED:
      stopMotors();
      digitalWrite(STATUS_LED, HIGH);
      break;

    // -------------------------------------------------
    // SAFE STOP AFTER A FAILED SEARCH
    // -------------------------------------------------

    case LOST_STOPPED:
      stopMotors();

      // Wait for stable sensor detection before moving.
      if (mask != 0) {
        if (reacquireStreak < 255) {
          reacquireStreak++;
        }
      } else {
        reacquireStreak = 0;
      }

      if (reacquireStreak >= 3) {
        resumeFollowing(mask, now);
      }

      break;

    // -------------------------------------------------
    // SENSOR COMMUNICATION FAILURE
    // -------------------------------------------------

    case SENSOR_FAULT:
      stopMotors();
      break;
  }
}

// =====================================================
// SETUP
// =====================================================

void setup() {
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);

  bool pwmAReady = ledcAttach(
    ENA, PWM_FREQ, PWM_RESOLUTION
  );

  bool pwmBReady = ledcAttach(
    ENB, PWM_FREQ, PWM_RESOLUTION
  );

  if (!pwmAReady || !pwmBReady) {
    digitalWrite(ENA, LOW);
    digitalWrite(ENB, LOW);

    while (true) {
      digitalWrite(
        STATUS_LED,
        !digitalRead(STATUS_LED)
      );

      delay(200);
    }
  }

  stopMotors();

  // ESP32 Wire.begin order: SDA, then SCL.
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);

  delay(300);
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {
  uint32_t nowUs = micros();

  if ((uint32_t)(nowUs - lastControlUs)
      < CONTROL_PERIOD_US) {
    return;
  }

  lastControlUs = nowUs;

  uint32_t nowMs = millis();

  uint8_t rawState;

  if (!readSensorState(rawState)) {
    stopMotors();

    runState = SENSOR_FAULT;
    filterInitialized = false;

    edgeCandidate = 0;
    centerStreak = 0;
    reacquireStreak = 0;

    // Blink LED while I2C communication is failing.
    digitalWrite(
      STATUS_LED,
      ((nowMs / 150) % 2) ? HIGH : LOW
    );

    return;
  }

  digitalWrite(STATUS_LED, LOW);

  // After sensor communication returns, stop first
  // and reacquire the line before resuming motion.
  if (runState == SENSOR_FAULT) {
    enterState(LOST_STOPPED, nowMs);

    turnArmed = false;
    previousError = 0.0f;
    edgeCandidate = 0;

    filterInitialized = false;
  }

  uint8_t lineMask = getLineMask(rawState);
  lineMask = filterLineMask(lineMask);

  processLineMask(lineMask, nowMs);
}
