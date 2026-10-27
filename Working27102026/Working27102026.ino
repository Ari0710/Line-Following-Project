/*
  Line Follower LF-2 : Arduino Nano + TB6612FNG + N20 motors + 7-channel analog IR array
  Sensor logic : LOW on black, HIGH on white  (isBlackLine = 0 -> white line)

  Buttons when the robot is idle
    D7  (B1) : CALIBRATE (spins ~3 s), then returns to idle (no auto run)
    D11 (B2) : DRY RUN   (needs calibration first)
    D12 (B3) : FINAL RUN (needs calibration first)

  Buttons while a run is going
    D7  (B1) : Kd +  -> stops, reverses, restarts with the new Kd
    D11 (B2) : Kd -  -> stops, reverses, restarts with the new Kd
    D12 (B3) : STOP (back to idle)

  Kd is saved to EEPROM when you stop, so the tuned value survives power-off.

  U-TURN / END OF LINE:
    When the line is lost, the robot stops at once, waits LOST_BEFORE_UTURN_MS, then spins
    on its own axis toward the side where the line was last seen.
    If the line is not found within UTURN_TIMEOUT_MS, the run stops and returns to idle.

  IMPORTANT (D12 shared by button 3 and STBY):
    - D12 is ONLY ever INPUT_PULLUP. The internal pull-up keeps STBY HIGH (motors enabled).
    - Button 3 must connect D12 to GND when pressed. Pressing it pulls STBY LOW,
      so motors are disabled while it is held (this is also the hardware "stop").
    - Never set D12 as OUTPUT.

  NOTE: the error here is on the "big" scale (weight x 0..1000, averaged), so Kp and Kd are small.
*/

#include <EEPROM.h>

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

// Symmetric weights: positive = line is toward the LEFT sensors.
const float sensorWeight[NUM_SENSORS] = { 3, 2, 1, 0, -1, -2, -3 };

bool isBlackLine = 0;  // 1 = black line, 0 = white line

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
// PID
// ============================================================
float Kp = 0.15;
float Kd = 0.05;
float Ki = 0;

const float KD_STEP = 0.02;   // one press = one step
const float KD_MIN  = 0.0;
const float KD_MAX  = 30.0;
bool kdChanged = false;

float error = 0, previousError = 0, I = 0;
int PIDvalue;
int lsp, rsp;

int lfSpeed = 170;        // final run speed
int dryRunSpeed = 120;    // dry run speed
int currentSpeed = 80;

// ============================================================
// REVERSE-AND-RESTART after each Kd change
// ============================================================
const unsigned long BACK_TIME_MS = 600;
const int BACK_SPEED = 100;
const unsigned long RESTART_PAUSE_MS = 300;

// ============================================================
// U-TURN (line lost -> stop, then spin on own axis)
// ============================================================
const int UTURN_SPEED = 150;                     // raise to 180 if one wheel does not turn
const unsigned long LOST_BEFORE_UTURN_MS = 80;   // line missing this long -> U-turn
const unsigned long UTURN_TIMEOUT_MS = 3000;     // give up if the line is not found

//  1 = line was last seen toward the LEFT sensors -> spin right
// -1 = line was last seen toward the RIGHT sensors -> spin left
int lastTurnDirection = 1;

// ============================================================
// EEPROM
// ============================================================
const int EE_MAGIC_ADDR = 0;
const int EE_KD_ADDR = 2;
const uint16_t EE_MAGIC = 0xA55E;

void loadKd() {
  uint16_t m;
  EEPROM.get(EE_MAGIC_ADDR, m);

  if (m == EE_MAGIC) {
    float k;
    EEPROM.get(EE_KD_ADDR, k);

    if (k >= KD_MIN && k <= KD_MAX) {
      Kd = k;
    }
  }
}

void saveKd() {
  EEPROM.put(EE_MAGIC_ADDR, EE_MAGIC);
  EEPROM.put(EE_KD_ADDR, Kd);
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  // Faster ADC. If readings are noisy, use prescaler 32 or 64 instead.
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

  loadKd();

  Serial.print(F("Kp = "));
  Serial.print(Kp, 3);
  Serial.print(F("  Kd = "));
  Serial.println(Kd, 3);
}

// ============================================================
// BUTTON HELPERS
// ============================================================
void waitRelease(int pin) {
  while (digitalRead(pin) == LOW) {}
  delay(50);
}

// One step per press. Returns true if Kd was changed (motors are stopped).
bool handleKdButtons() {
  bool up   = (digitalRead(BTN_CALIBRATE) == LOW);   // B1 = Kd +
  bool down = (digitalRead(BTN_DRYRUN) == LOW);      // B2 = Kd -

  if (up == down) return false;

  delay(30);                                         // debounce

  up   = (digitalRead(BTN_CALIBRATE) == LOW);
  down = (digitalRead(BTN_DRYRUN) == LOW);

  if (up == down) return false;

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

  if (mode == 2) {
    runLineFollower(dryRunSpeed);
  } else {
    runLineFollower(lfSpeed);
  }
}

// ============================================================
// LINE FOLLOW RUN
// ============================================================
void runLineFollower(int targetSpeed) {
  I = 0;
  previousError = 0;
  error = 0;
  currentSpeed = 30;
  kdChanged = false;

  unsigned long lostSince = 0;

  while (1) {
    // B3 pressed -> stop and go back to idle.
    // Note: pressing B3 also pulls TB6612FNG STBY LOW, disabling the motors.
    if (digitalRead(BTN_MAIN) == LOW) {
      motor1run(0);
      motor2run(0);
      digitalWrite(LED_PIN, LOW);

      if (kdChanged) saveKd();

      waitRelease(BTN_MAIN);
      return;
    }

    // B1 / B2 pressed -> change Kd, reverse, then restart with the new Kd.
    if (handleKdButtons()) {
      motor1run(-BACK_SPEED);
      motor2run(-BACK_SPEED);

      unsigned long reverseStart = millis();

      while (millis() - reverseStart < BACK_TIME_MS) {
        if (digitalRead(BTN_MAIN) == LOW) {
          motor1run(0);
          motor2run(0);

          if (kdChanged) saveKd();

          waitRelease(BTN_MAIN);
          return;
        }
      }

      motor1run(0);
      motor2run(0);
      delay(RESTART_PAUSE_MS);

      I = 0;
      previousError = 0;
      error = 0;
      currentSpeed = 30;
      lostSince = 0;
      continue;
    }

    readLine();

    if (currentSpeed < targetSpeed) {
      currentSpeed++;
    }

    if (onLine) {
      lostSince = 0;
      linefollow();
      digitalWrite(LED_PIN, HIGH);
    } else {
      digitalWrite(LED_PIN, LOW);

      if (lostSince == 0) {
        lostSince = millis();
      }

      if (millis() - lostSince > LOST_BEFORE_UTURN_MS) {
        // Line is over -> U-turn on own axis.
        if (uTurn()) {
          I = 0;
          previousError = 0;
          error = 0;
          currentSpeed = 60;
          lostSince = 0;
        } else {
          // Line not found or B3 pressed -> stop, back to idle.
          motor1run(0);
          motor2run(0);

          if (kdChanged) saveKd();

          if (digitalRead(BTN_MAIN) == LOW) {
            waitRelease(BTN_MAIN);
          }

          return;
        }
      } else {
        // Line gone: stop at once, do not drive in an arc.
        motor1run(0);
        motor2run(0);
      }
    }
  }
}

// ============================================================
// U-TURN: spin in place toward the last known line side.
// Returns true if the line was found, false on timeout or B3 stop.
// ============================================================
bool uTurn() {
  motor1run(0);
  motor2run(0);
  delay(100);                                   // short pause before spinning

  if (lastTurnDirection > 0) {
    // Line was toward the LEFT sensors -> spin RIGHT.
    motor1run(UTURN_SPEED);                     // left forward
    motor2run(-UTURN_SPEED);                    // right backward
  } else {
    // Line was toward the RIGHT sensors -> spin LEFT.
    motor1run(-UTURN_SPEED);
    motor2run(UTURN_SPEED);
  }

  unsigned long t0 = millis();
  bool found = false;

  while (millis() - t0 < UTURN_TIMEOUT_MS) {
    if (digitalRead(BTN_MAIN) == LOW) {
      break;                                    // B3 = stop
    }

    readLine();

    // Spin for at least 100 ms, then stop once the centre sensor sees the line.
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

  if (activeSensors > 0) {
    error /= activeSensors;
  }

  error *= steerDir;

  // Remember which side the line was on, for a smarter U-turn.
  if (error > 100) {
    lastTurnDirection = 1;      // line toward left sensors
  } else if (error < -100) {
    lastTurnDirection = -1;     // line toward right sensors
  }

  float P = error;
  I += error;
  I = constrain(I, -1000, 1000);

  float D = error - previousError;

  PIDvalue = (int)((Kp * P) + (Ki * I) + (Kd * D));
  previousError = error;

  lsp = currentSpeed - PIDvalue;
  rsp = currentSpeed + PIDvalue;

  lsp = constrain(lsp, -100, 255);
  rsp = constrain(rsp, -100, 255);

  motor1run(lsp);
  motor2run(rsp);
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

  // Assumes LEFT motor forward + RIGHT motor backward = spin.
  // If this drives the robot forward/backward instead, swap one motor's wiring.
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

  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(F("S"));
    Serial.print(i);
    Serial.print(F(": "));
    Serial.print(minValues[i]);
    Serial.print(F("-"));
    Serial.print(maxValues[i]);
    Serial.print(F("  "));
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

    if (maxValues[i] - minValues[i] < 10) {
      // Sensor was not calibrated properly.
      sensorValue[i] = 0;
    } else if (isBlackLine) {
      sensorValue[i] = map(raw, minValues[i], maxValues[i], 1000, 0);
    } else {
      sensorValue[i] = map(raw, minValues[i], maxValues[i], 0, 1000);
    }

    sensorValue[i] = constrain(sensorValue[i], 0, 1000);
    sensorArray[i] = sensorValue[i] > 500;

    if (sensorArray[i]) {
      onLine = true;
    }
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