/*
  Line Follower LF-2 : Arduino Nano + TB6612FNG + N20 motors + 7-channel analog IR array
  Sensor logic : HIGH (>900) on black floor, LOW on white line  (isBlackLine = 1 for this array)

  Buttons when the robot is idle
    D7  (B1) : CALIBRATE (spins ~3 s), then returns to idle (no auto run)
    D11 (B2) : DRY RUN   (needs calibration first)
    D12 (B3) : FINAL RUN (needs calibration first)

  Button while a run is going
    D12 (B3) : STOP (back to idle)

  U-TURN / END OF LINE:
    When the line is lost, the robot stops at once, waits LOST_BEFORE_UTURN_MS, then spins
    on its own axis (one motor forward, one backward) until the centre sensor finds the line.
    If the line is not found within UTURN_TIMEOUT_MS, the run stops and returns to idle.
    After MAX_UTURNS U-turns in one run, the next line loss ends the run (no endless shuttling).
    Set MAX_UTURNS = 0 for unlimited U-turns (e.g. closed-loop tracks).

  IMPORTANT (D12 shared by button 3 and STBY):
    - D12 is ONLY ever INPUT_PULLUP. The internal pull-up keeps STBY HIGH (motors enabled).
    - Button 3 must connect D12 to GND when pressed. Pressing it pulls STBY LOW,
      so motors are disabled while it is held (this is also the hardware "stop").
    - Never set D12 as OUTPUT.

  NOTE: the error is on the "big" scale (weight x 0..1000, averaged), so Kp and Kd are small.
*/

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// ============================================================
// PINS
// ============================================================
#define AIN1 3
#define AIN2 2
#define PWMA 9

#define BIN1 5
#define BIN2 6
#define PWMB 10

#define BTN_CALIBRATE 7    // B1
#define BTN_DRYRUN    11   // B2
#define BTN_MAIN      12   // B3 + STBY
#define LED_PIN       13

// ============================================================
// SENSORS
// ============================================================
#define NUM_SENSORS 7
const uint8_t sensorPin[NUM_SENSORS] = { A0, A1, A2, A3, A4, A5, A6 };

// Symmetric weights: centred line -> error 0
const float sensorWeight[NUM_SENSORS] = { 3, 2, 1, 0, -1, -2, -3 };

// Your array reads HIGH (>900) on the black floor and LOW on the white line.
// isBlackLine = 1 flips the mapping so the LOW (white line) reading becomes "active".
bool isBlackLine = 1;

int minValues[NUM_SENSORS], maxValues[NUM_SENSORS];
int sensorValue[NUM_SENSORS];
bool sensorArray[NUM_SENSORS];
bool onLine = true;
bool calibrated = false;

// ============================================================
// SIDE-PULL FIXES
// ============================================================
int steerDir = 1;           // set to -1 if the robot steers away from the line
float leftScale  = 1.00;    // lower if the LEFT motor is stronger
float rightScale = 1.00;    // lower if the RIGHT motor is stronger

// ============================================================
// PID  (edit here and re-upload to tune)
// Tune Kp first: raise slowly until it wobbles, then back off a bit.
// Then raise Kd until the wobble calms down.
// ============================================================
float Kp = 0.15;
float Kd = 0.040;
float Ki = 0;
const bool DEBUG_RUN = true;   // prints sensors/error/motors 10x per second during a run

float error = 0, previousError = 0, I = 0;
int PIDvalue;
int lsp, rsp;
int lfSpeed = 220;        // final run speed
int dryRunSpeed = 120;    // dry run speed (raise once it tracks cleanly)
int currentSpeed = 80;

// ============================================================
// U-TURN (line lost -> stop, then spin on own axis)
// ============================================================
const int UTURN_SPEED = 150;                     // raise to 180 if one wheel does not turn
const int UTURN_DIR = 1;                         // 1 = spin right, -1 = spin left
const unsigned long LOST_BEFORE_UTURN_MS = 80;   // line missing this long -> U-turn
const unsigned long UTURN_TIMEOUT_MS = 3000;     // give up if the line is not found
const int MAX_UTURNS = 1;                        // U-turns allowed per run (0 = unlimited)

// ============================================================
// SETUP
// ============================================================
void setup() {
  // faster ADC (prescaler 16)
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);

  Serial.begin(115200);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  pinMode(BTN_MAIN, INPUT_PULLUP);
  pinMode(BTN_CALIBRATE, INPUT_PULLUP);
  pinMode(BTN_DRYRUN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  motor1run(0);
  motor2run(0);

  Serial.print("Kp = ");
  Serial.print(Kp, 3);
  Serial.print("  Kd = ");
  Serial.println(Kd, 3);
}

// ============================================================
// BUTTON HELPERS
// ============================================================
void waitRelease(int pin) {
  while (digitalRead(pin) == LOW) {}
  delay(50);
}

void errorBlink() {
  for (int i = 0; i < 6; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(70);
    digitalWrite(LED_PIN, LOW);
    delay(70);
  }
}

// Idle: returns 1 = calibrate, 2 = dry run, 3 = final run.
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

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {
  int mode = waitForMode();

  if (mode == 1) {
    digitalWrite(LED_PIN, HIGH);
    calibrate();                          // ~3 s spin
    digitalWrite(LED_PIN, LOW);
    calibrated = true;
    return;                               // back to idle
  }

  if (!calibrated) {
    errorBlink();
    return;
  }

  if (mode == 2) runLineFollower(dryRunSpeed);
  else runLineFollower(lfSpeed);
}

// ============================================================
// LINE FOLLOW RUN
// ============================================================
void runLineFollower(int targetSpeed) {
  I = 0;
  previousError = 0;
  error = 0;
  currentSpeed = 30;
  unsigned long lostSince = 0;
  int uTurnCount = 0;

  while (1) {
    // B3 pressed -> stop and go back to idle
    if (digitalRead(BTN_MAIN) == LOW) {
      motor1run(0);
      motor2run(0);
      digitalWrite(LED_PIN, LOW);
      waitRelease(BTN_MAIN);
      return;
    }

    readLine();

    if (currentSpeed < targetSpeed) currentSpeed++;

    if (onLine) {
      lostSince = 0;
      linefollow();
      digitalWrite(LED_PIN, HIGH);
    } else {
      digitalWrite(LED_PIN, LOW);
      if (lostSince == 0) lostSince = millis();

      if (millis() - lostSince > LOST_BEFORE_UTURN_MS) {
        // U-turn limit reached -> end the run
        if (MAX_UTURNS > 0 && uTurnCount >= MAX_UTURNS) {
          motor1run(0);
          motor2run(0);
          digitalWrite(LED_PIN, LOW);
          return;
        }

        // line is over -> U-turn on own axis
        if (uTurn()) {
          uTurnCount++;
          I = 0;
          previousError = 0;
          error = 0;
          currentSpeed = 60;
          lostSince = 0;
        } else {
          // line not found or B3 pressed -> stop, back to idle
          motor1run(0);
          motor2run(0);
          if (digitalRead(BTN_MAIN) == LOW) waitRelease(BTN_MAIN);
          return;
        }
      } else {
        motor1run(0);      // line gone: stop at once, do not drive in an arc
        motor2run(0);
      }
    }
  }
}

// ============================================================
// U-TURN: spin in place until the centre sensor sees the line
// Returns true if the line was found, false on timeout or B3 stop.
// ============================================================
bool uTurn() {
  motor1run(0);
  motor2run(0);
  delay(100);                                   // short pause before spinning

  if (UTURN_DIR > 0) {
    motor1run(UTURN_SPEED);                     // left forward
    motor2run(-UTURN_SPEED);                    // right backward
  } else {
    motor1run(-UTURN_SPEED);
    motor2run(UTURN_SPEED);
  }

  unsigned long t0 = millis();
  bool found = false;

  while (millis() - t0 < UTURN_TIMEOUT_MS) {
    if (digitalRead(BTN_MAIN) == LOW) break;    // B3 = stop

    readLine();
    // spin at least 100 ms, then stop as soon as the centre sensor sees the line
    if (millis() - t0 > 100 && sensorArray[3]) {
      found = true;
      break;
    }
  }

  motor1run(0);
  motor2run(0);
  delay(100);
  return found;
}

// ============================================================
// PID LOGIC
// ============================================================
void linefollow() {
  error = 0;
  int activeSensors = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorArray[i]) {
      error += (float)sensorWeight[i] * sensorValue[i];
      activeSensors++;
    }
  }
  if (activeSensors > 0) error /= activeSensors;
  error *= steerDir;

  float P = error;
  I += error;
  float D = error - previousError;

  PIDvalue = (int)((Kp * P) + (Ki * I) + (Kd * D));
  previousError = error;

  lsp = currentSpeed - PIDvalue;
  rsp = currentSpeed + PIDvalue;

  lsp = constrain(lsp, -100, 255);
  rsp = constrain(rsp, -100, 255);

  motor1run(lsp);
  motor2run(rsp);

  if (DEBUG_RUN) {
    static unsigned long lastDbg = 0;
    if (millis() - lastDbg > 100) {
      lastDbg = millis();
      for (int i = 0; i < NUM_SENSORS; i++) Serial.print(sensorArray[i] ? '1' : '0');
      Serial.print("  err=");
      Serial.print(error, 0);
      Serial.print("  pid=");
      Serial.print(PIDvalue);
      Serial.print("  L=");
      Serial.print(lsp);
      Serial.print("  R=");
      Serial.println(rsp);
    }
  }
}

// ============================================================
// CALIBRATION (spins in place for ~3 seconds)
// ============================================================
void calibrate() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    int v = analogRead(sensorPin[i]);
    minValues[i] = v;
    maxValues[i] = v;
  }

  motor1run(70);
  motor2run(-70);

  unsigned long t0 = millis();
  while (millis() - t0 < 3000) {
    for (int j = 0; j < NUM_SENSORS; j++) {
      int val = analogRead(sensorPin[j]);
      if (val < minValues[j]) minValues[j] = val;
      if (val > maxValues[j]) maxValues[j] = val;
    }
  }

  motor1run(0);
  motor2run(0);

  // Sensor that never crossed the line during the spin: its range is just noise.
  // Your log shows white is ~150-300 below black, so assume 220 instead of stretching noise.
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (maxValues[i] - minValues[i] < 120) minValues[i] = maxValues[i] - 220;
  }

  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print("S");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(minValues[i]);
    Serial.print("-");
    Serial.print(maxValues[i]);
    Serial.print("  ");
  }
  Serial.println();
}

// ============================================================
// SENSOR READING
// ============================================================
void readLine() {
  onLine = false;
  for (int i = 0; i < NUM_SENSORS; i++) {
    int raw = analogRead(sensorPin[i]);

    if (maxValues[i] - minValues[i] < 10) {   // not calibrated properly
      sensorValue[i] = 0;
    } else if (isBlackLine) {
      sensorValue[i] = map(raw, minValues[i], maxValues[i], 1000, 0);
    } else {
      sensorValue[i] = map(raw, minValues[i], maxValues[i], 0, 1000);
    }

    sensorValue[i] = constrain(sensorValue[i], 0, 1000);
    sensorArray[i] = sensorValue[i] > 500;
    if (sensorArray[i]) onLine = true;
  }
}

// ============================================================
// MOTOR DRIVERS (TB6612FNG)
// ============================================================
void motor1run(int motorSpeed) {   // LEFT (A)
  motorSpeed = constrain(motorSpeed, -255, 255);
  motorSpeed = (int)(motorSpeed * leftScale);
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

void motor2run(int motorSpeed) {   // RIGHT (B)
  motorSpeed = constrain(motorSpeed, -255, 255);
  motorSpeed = (int)(motorSpeed * rightScale);
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
