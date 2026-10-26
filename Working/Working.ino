/*
  MAZE ROBOT - LINE FOLLOWER WITH U-TURN RECOVERY

  Hardware:
    Arduino Nano
    TB6612FNG motor driver
    N20 motors
    7-channel analog IR sensor array

  BUTTONS (connect each button between pin and GND):
    D7  = Calibrate when idle / Kd+ while running
    D11 = Dry run when idle / Kd- while running
    D12 = Final run when idle / STOP while running

  IMPORTANT:
    D12 is also connected to TB6612FNG STBY.
    D12 must ALWAYS remain INPUT_PULLUP.
    Never configure D12 as OUTPUT.

  SENSOR ORDER ASSUMPTION:
    A0 = leftmost sensor
    A6 = rightmost sensor

  SENSOR POLARITY:
    isBlackLine = false -> higher raw readings indicate the line
    isBlackLine = true  -> lower raw readings indicate the line

  CONTROL:
    Kp = 0.18
    Kd = 0.040
    Ki = 0.0

  LINE-LOSS RECOVERY:
    1. Wait briefly after line loss.
    2. Search in the last known line direction.
    3. Search in the opposite direction.
    4. If searching fails, attempt an approximate 180-degree turn.
    5. If the line is still not found, stop safely.

  LIMITATION:
    This is line-following with U-turn recovery.
    It does not yet map maze junctions or calculate a shortest path.
    Without encoders or distinctive markers, a timed U-turn is approximate.
*/

#include <EEPROM.h>

// ============================================================
// PIN DEFINITIONS
// ============================================================

// TB6612FNG motor A
#define AIN1 3
#define AIN2 2
#define PWMA 9

// TB6612FNG motor B
#define BIN1 5
#define BIN2 6
#define PWMB 10

// Buttons
#define BTN_CALIBRATE 7
#define BTN_DRYRUN    11
#define BTN_MAIN      12

#define LED_PIN 13

// ============================================================
// SENSOR SETTINGS
// ============================================================

#define NUM_SENSORS 7

const uint8_t sensorPin[NUM_SENSORS] = {
  A0, A1, A2, A3, A4, A5, A6
};

// Assumes A0 is leftmost and A6 is rightmost.
const float sensorWeight[NUM_SENSORS] = {
  3, 2, 1, 0, -1, -2, -3
};

// false: higher raw value means the desired line.
// true:  lower raw value means the desired line.
bool isBlackLine = false;

// Change to -1 only if the robot consistently steers
// away from the line during the initial steering test.
int steerDir = 1;

// Motor balance adjustments.
float leftScale = 1.00;
float rightScale = 1.00;

int minValues[NUM_SENSORS];
int maxValues[NUM_SENSORS];
int sensorValue[NUM_SENSORS];
bool sensorArray[NUM_SENSORS];

bool onLine = false;
bool calibrated = false;

// ============================================================
// PID SETTINGS
// ============================================================

// Original gains retained as requested.
float Kp = 0.18;
float Kd = 0.040;
float Ki = 0.0;

const float KD_STEP = 0.02;
const float KD_MIN = 0.0;
const float KD_MAX = 30.0;

bool kdChanged = false;

float error = 0;
float previousError = 0;
float I = 0;
float lastLineError = 0;

int PIDvalue = 0;
int lsp = 0;
int rsp = 0;

// ============================================================
// SPEED SETTINGS
// ============================================================

int lfSpeed = 220;
int dryRunSpeed = 170;
int currentSpeed = 30;

// Nominal line-following control-loop period.
const unsigned long CONTROL_PERIOD_US = 5000UL;

// ============================================================
// LINE-LOSS RECOVERY SETTINGS
// ============================================================

const unsigned long LOST_BEFORE_SEARCH_MS = 80;

const int SEARCH_SPEED = 90;
const unsigned long SEARCH_PHASE_MS = 350;

// ============================================================
// U-TURN SETTINGS
// ============================================================

// Approximate rotation speed.
const int UTURN_SPEED = 150;

// Approximate time to rotate before accepting a centre-sensor
// detection. Must be tuned for your robot.
const unsigned long UTURN_MIN_ROTATE_MS = 450;

// Maximum time allowed for the U-turn and line reacquisition.
const unsigned long UTURN_TIMEOUT_MS = 2500;

// 1 or -1 selects the direction of rotation.
// Change to -1 if the robot turns the wrong way.
const int UTURN_DIR = 1;

// ============================================================
// EEPROM SETTINGS
// ============================================================

const int EE_MAGIC_ADDR = 0;
const int EE_KD_ADDR = 2;
const uint16_t EE_MAGIC = 0xA55E;

// ============================================================
// FUNCTION DECLARATIONS
// ============================================================

void loadKd();
void saveKd();

void motor1run(int motorSpeed);
void motor2run(int motorSpeed);

void waitRelease(int pin);
void errorBlink();
int waitForMode();

bool handleKdButtons();
bool calibrate();
bool readLine();

void runLineFollower(int targetSpeed);
void linefollow();
bool searchForLine();
bool uTurn();

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  pinMode(BTN_CALIBRATE, INPUT_PULLUP);
  pinMode(BTN_DRYRUN, INPUT_PULLUP);

  // D12 is shared with TB6612FNG STBY.
  // Keep it as INPUT_PULLUP permanently.
  pinMode(BTN_MAIN, INPUT_PULLUP);

  pinMode(LED_PIN, OUTPUT);

  motor1run(0);
  motor2run(0);

  loadKd();

  Serial.println(F("\nMaze Robot Ready"));
  Serial.print(F("Kp = "));
  Serial.println(Kp, 3);

  Serial.print(F("Kd = "));
  Serial.println(Kd, 3);

  Serial.println(F("D7: Calibrate / Kd+"));
  Serial.println(F("D11: Dry run / Kd-"));
  Serial.println(F("D12: Final run / Stop"));
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop() {
  int mode = waitForMode();

  if (mode == 1) {
    digitalWrite(LED_PIN, HIGH);

    calibrated = calibrate();

    digitalWrite(LED_PIN, LOW);

    if (calibrated) {
      Serial.println(F("Calibration successful."));
    } else {
      Serial.println(F("Calibration failed."));
      errorBlink();
    }

    return;
  }

  if (!calibrated) {
    Serial.println(F("Calibrate before running."));
    errorBlink();
    return;
  }

  if (mode == 2) {
    Serial.println(F("DRY RUN started."));
    runLineFollower(dryRunSpeed);
  } else if (mode == 3) {
    Serial.println(F("FINAL RUN started."));
    runLineFollower(lfSpeed);
  }
}

// ============================================================
// EEPROM: LOAD AND SAVE KD
// ============================================================

void loadKd() {
  uint16_t storedMagic;
  EEPROM.get(EE_MAGIC_ADDR, storedMagic);

  if (storedMagic == EE_MAGIC) {
    float storedKd;
    EEPROM.get(EE_KD_ADDR, storedKd);

    if (storedKd >= KD_MIN && storedKd <= KD_MAX) {
      Kd = storedKd;
    }
  }
}

void saveKd() {
  EEPROM.put(EE_MAGIC_ADDR, EE_MAGIC);
  EEPROM.put(EE_KD_ADDR, Kd);
}

// ============================================================
// BUTTON HELPERS
// ============================================================

void waitRelease(int pin) {
  while (digitalRead(pin) == LOW) {
    // Wait for release.
  }

  delay(50);
}

int waitForMode() {
  motor1run(0);
  motor2run(0);
  digitalWrite(LED_PIN, LOW);

  while (true) {
    if (digitalRead(BTN_CALIBRATE) == LOW) {
      delay(30);

      if (digitalRead(BTN_CALIBRATE) == LOW) {
        waitRelease(BTN_CALIBRATE);
        return 1;
      }
    }

    if (digitalRead(BTN_DRYRUN) == LOW) {
      delay(30);

      if (digitalRead(BTN_DRYRUN) == LOW) {
        waitRelease(BTN_DRYRUN);
        return 2;
      }
    }

    if (digitalRead(BTN_MAIN) == LOW) {
      delay(30);

      if (digitalRead(BTN_MAIN) == LOW) {
        waitRelease(BTN_MAIN);
        return 3;
      }
    }
  }
}

void errorBlink() {
  for (int i = 0; i < 6; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(70);

    digitalWrite(LED_PIN, LOW);
    delay(70);
  }
}

// ============================================================
// KD TUNING WHILE RUNNING
// ============================================================

bool handleKdButtons() {
  bool up = digitalRead(BTN_CALIBRATE) == LOW;
  bool down = digitalRead(BTN_DRYRUN) == LOW;

  // Ignore if both buttons are pressed or neither is pressed.
  if (up == down) {
    return false;
  }

  delay(30);

  up = digitalRead(BTN_CALIBRATE) == LOW;
  down = digitalRead(BTN_DRYRUN) == LOW;

  if (up == down) {
    return false;
  }

  motor1run(0);
  motor2run(0);

  if (up) {
    Kd += KD_STEP;
  } else {
    Kd -= KD_STEP;
  }

  Kd = constrain(Kd, KD_MIN, KD_MAX);
  kdChanged = true;

  Serial.print(F("Kd = "));
  Serial.println(Kd, 3);

  waitRelease(up ? BTN_CALIBRATE : BTN_DRYRUN);

  return true;
}

// ============================================================
// SENSOR CALIBRATION
// ============================================================

bool calibrate() {
  Serial.println(F("Calibrating for 3 seconds."));
  Serial.println(F("Ensure every sensor sees both line and background."));

  motor1run(0);
  motor2run(0);

  for (int i = 0; i < NUM_SENSORS; i++) {
    int v = analogRead(sensorPin[i]);

    minValues[i] = v;
    maxValues[i] = v;
  }

  // Slowly rotate to expose the sensors to both surfaces.
  motor1run(70);
  motor2run(-70);

  unsigned long startTime = millis();

  while (millis() - startTime < 3000UL) {
    if (digitalRead(BTN_MAIN) == LOW) {
      motor1run(0);
      motor2run(0);

      waitRelease(BTN_MAIN);

      Serial.println(F("Calibration cancelled."));
      return false;
    }

    for (int i = 0; i < NUM_SENSORS; i++) {
      int val = analogRead(sensorPin[i]);

      if (val < minValues[i]) {
        minValues[i] = val;
      }

      if (val > maxValues[i]) {
        maxValues[i] = val;
      }
    }
  }

  motor1run(0);
  motor2run(0);

  bool valid = true;

  Serial.println(F("Sensor calibration ranges:"));

  for (int i = 0; i < NUM_SENSORS; i++) {
    int range = maxValues[i] - minValues[i];

    Serial.print(F("S"));
    Serial.print(i);
    Serial.print(F(": "));
    Serial.print(minValues[i]);
    Serial.print(F("-"));
    Serial.print(maxValues[i]);
    Serial.print(F(" range="));
    Serial.println(range);

    if (range < 20) {
      valid = false;
    }
  }

  if (!valid) {
    Serial.println(F("Calibration range too small on at least one sensor."));
    Serial.println(F("Check sensor height, wiring, and calibration movement."));
    return false;
  }

  I = 0;
  error = 0;
  previousError = 0;
  lastLineError = 0;

  return true;
}

// ============================================================
// READ AND NORMALIZE SENSORS
// ============================================================

bool readLine() {
  onLine = false;

  for (int i = 0; i < NUM_SENSORS; i++) {
    int raw = analogRead(sensorPin[i]);
    int range = maxValues[i] - minValues[i];

    if (range < 20) {
      sensorValue[i] = 0;
    } else if (isBlackLine) {
      // Lower raw readings represent the desired line.
      sensorValue[i] =
        map(raw, minValues[i], maxValues[i], 1000, 0);
    } else {
      // Higher raw readings represent the desired line.
      sensorValue[i] =
        map(raw, minValues[i], maxValues[i], 0, 1000);
    }

    sensorValue[i] = constrain(sensorValue[i], 0, 1000);

    sensorArray[i] = sensorValue[i] > 500;

    if (sensorArray[i]) {
      onLine = true;
    }
  }

  return onLine;
}

// ============================================================
// LINE-FOLLOWING RUN
// ============================================================

void runLineFollower(int targetSpeed) {
  I = 0;
  previousError = 0;
  error = 0;
  lastLineError = 0;

  currentSpeed = 30;
  kdChanged = false;

  unsigned long lostSince = 0;

  while (true) {
    unsigned long loopStartUs = micros();

    // STOP button. D12 also pulls TB6612FNG STBY LOW.
    if (digitalRead(BTN_MAIN) == LOW) {
      motor1run(0);
      motor2run(0);
      digitalWrite(LED_PIN, LOW);

      if (kdChanged) {
        saveKd();
      }

      waitRelease(BTN_MAIN);
      return;
    }

    // Tune Kd while running.
    if (handleKdButtons()) {
      motor1run(-100);
      motor2run(-100);
      delay(250);

      motor1run(0);
      motor2run(0);
      delay(200);

      I = 0;
      previousError = 0;
      error = 0;
      lostSince = 0;
      currentSpeed = 30;

      continue;
    }

    // Gradual acceleration with a nominal 5 ms control period.
    if (currentSpeed < targetSpeed) {
      currentSpeed++;
    }

    readLine();

    if (onLine) {
      lostSince = 0;

      linefollow();

      digitalWrite(LED_PIN, HIGH);
    } else {
      digitalWrite(LED_PIN, LOW);

      if (lostSince == 0) {
        lostSince = millis();
      }

      if (millis() - lostSince >= LOST_BEFORE_SEARCH_MS) {
        motor1run(0);
        motor2run(0);

        Serial.println(F("Line lost. Starting search."));

        bool found = searchForLine();

        // If the normal search fails, attempt a U-turn.
        if (!found) {
          // If STOP is currently pressed, do not start turning.
          if (digitalRead(BTN_MAIN) == LOW) {
            motor1run(0);
            motor2run(0);

            if (kdChanged) {
              saveKd();
            }

            return;
          }

          Serial.println(F("Search failed. Attempting U-turn."));
          found = uTurn();
        }

        if (found) {
          I = 0;
          previousError = 0;
          error = 0;
          lostSince = 0;
          currentSpeed = 60;

          Serial.println(F("Line reacquired. Resuming."));
        } else {
          motor1run(0);
          motor2run(0);

          if (kdChanged) {
            saveKd();
          }

          Serial.println(F("Recovery failed. Robot stopped."));
          errorBlink();
          return;
        }
      } else {
        motor1run(0);
        motor2run(0);
      }
    }

    // Keep the normal control loop near 5 ms.
    while ((unsigned long)(micros() - loopStartUs)
           < CONTROL_PERIOD_US) {
      // Short timing wait.
    }
  }
}

// ============================================================
// SEARCH FOR THE LINE
// ============================================================

bool searchForLine() {
  // Search first toward the last known line direction.
  int firstDirection = (lastLineError >= 0) ? 1 : -1;

  for (int phase = 0; phase < 2; phase++) {
    int direction = (phase == 0)
                    ? firstDirection
                    : -firstDirection;

    unsigned long startTime = millis();

    while (millis() - startTime < SEARCH_PHASE_MS) {
      if (digitalRead(BTN_MAIN) == LOW) {
        motor1run(0);
        motor2run(0);
        return false;
      }

      // Rotate in place.
      motor1run(-direction * SEARCH_SPEED);
      motor2run(direction * SEARCH_SPEED);

      readLine();

      if (onLine) {
        motor1run(0);
        motor2run(0);
        delay(50);

        return true;
      }
    }

    motor1run(0);
    motor2run(0);
    delay(50);
  }

  return false;
}

// ============================================================
// APPROXIMATE 180-DEGREE U-TURN
// ============================================================

bool uTurn() {
  motor1run(0);
  motor2run(0);
  delay(100);

  unsigned long startTime = millis();

  while (millis() - startTime < UTURN_TIMEOUT_MS) {
    // STOP must remain responsive during the turn.
    if (digitalRead(BTN_MAIN) == LOW) {
      motor1run(0);
      motor2run(0);
      return false;
    }

    unsigned long elapsed = millis() - startTime;

    // Rotate in place.
    if (UTURN_DIR > 0) {
      motor1run(UTURN_SPEED);
      motor2run(-UTURN_SPEED);
    } else {
      motor1run(-UTURN_SPEED);
      motor2run(UTURN_SPEED);
    }

    readLine();

    // Do not accept the old line immediately.
    // Wait for the minimum rotation interval first.
    if (elapsed >= UTURN_MIN_ROTATE_MS && sensorArray[3]) {
      motor1run(0);
      motor2run(0);
      delay(100);

      readLine();

      if (sensorArray[3]) {
        Serial.println(F("U-turn complete: centre sensor sees line."));
        return true;
      }
    }
  }

  motor1run(0);
  motor2run(0);

  Serial.println(F("U-turn timed out: line not found."));
  return false;
}

// ============================================================
// WEIGHTED-CENTROID PD CONTROL
// ============================================================

void linefollow() {
  float weightedSum = 0;
  float totalStrength = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorValue[i] > 100) {
      weightedSum += sensorWeight[i] * sensorValue[i];
      totalStrength += sensorValue[i];
    }
  }

  if (totalStrength <= 0) {
    motor1run(0);
    motor2run(0);
    return;
  }

  // Centroid is approximately between -3 and +3.
  // Scale it to approximately -1000 to +1000.
  error = (weightedSum / totalStrength) * (1000.0 / 3.0);
  error *= steerDir;

  lastLineError = error;

  float P = error;
  float D = error - previousError;

  I += error;

  PIDvalue = (int)((Kp * P) + (Ki * I) + (Kd * D));

  previousError = error;

  lsp = currentSpeed - PIDvalue;
  rsp = currentSpeed + PIDvalue;

  // Preserve the original reverse-correction capability.
  lsp = constrain(lsp, -100, 255);
  rsp = constrain(rsp, -100, 255);

  motor1run(lsp);
  motor2run(rsp);
}

// ============================================================
// MOTOR A CONTROL
// ============================================================

void motor1run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);

  motorSpeed = (int)(motorSpeed * leftScale);
  motorSpeed = constrain(motorSpeed, -255, 255);

  if (motorSpeed > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, -motorSpeed);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, 0);
  }
}

// ============================================================
// MOTOR B CONTROL
// ============================================================

void motor2run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);

  motorSpeed = (int)(motorSpeed * rightScale);
  motorSpeed = constrain(motorSpeed, -255, 255);

  if (motorSpeed > 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, -motorSpeed);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, 0);
  }
}
