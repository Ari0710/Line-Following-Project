/*
  7-channel analog line follower
  Arduino Nano + TB6612FNG + 2x N20 motors

  Sensors: A0..A6, ordered LEFT to RIGHT.
  Buttons: D7 calibrate, D11 dry-run, D12 main/final run.
  LED: D13.

  IMPORTANT:
  - Sensor logic is configured for BLACK line on WHITE floor.
  - Calibration values are saved in EEPROM.
  - D12 is shared with TB6612FNG STBY and BTN_MAIN. Keep D12 as
    INPUT_PULLUP (do NOT configure it as OUTPUT). Wire BTN_MAIN from D12
    to GND. The internal pull-up normally holds STBY high; for the most
    reliable motor-driver enable, wire STBY to 5V separately and move
    BTN_MAIN to a different free pin.
  - Test with wheels lifted first.
*/

#include <EEPROM.h>

// ---------------- MOTOR PINS ----------------
#define AIN1 3
#define AIN2 2
#define PWMA 9

#define BIN1 5
#define BIN2 6
#define PWMB 10

#define BTN_CALIBRATE 7
#define BTN_DRYRUN    11
#define BTN_MAIN      12
#define LED_PIN       13

// ---------------- SENSORS ----------------
#define NUM_SENSORS 7
const uint8_t sensorPin[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5, A6};

// Set true because your latest readings showed higher ADC values on black.
const bool BLACK_IS_HIGH = true;

// Position weights, ordered LEFT to RIGHT.
// Positive means the line is to the left; negative means to the right.
const float sensorWeight[NUM_SENSORS] = { 5.0, 3.0, 1.0, 0.0, -1.0, -3.0, -5.0 };

// ---------------- TUNING ----------------
int baseSpeed = 80;
int dryRunSpeed = 120;
int turnSpeed = 100;
int calibrationSpeed = 65; // Slower rotation during calibration only
float Kp = 120.0;
float Kd = 80.0;
const int MAX_PWM = 180;
const int MIN_PWM = 0;
const unsigned long CALIBRATION_TIME_MS = 3500;
const unsigned long DEBOUNCE_MS = 35;
const unsigned long BLACK_BOX_CONFIRM_MS = 250;
const unsigned long LINE_LOST_TIMEOUT_MS = 1500;

// ---------------- STATE ----------------
int sensorRaw[NUM_SENSORS];
int sensorOnLine[NUM_SENSORS];
int sensorMin[NUM_SENSORS];
int sensorMax[NUM_SENSORS];
int sensorThreshold[NUM_SENSORS];

float errorValue = 0;
float previousError = 0;
int lastSeenSide = 0; // -1 = line last seen left, +1 = right
bool calibrated = false;

const int EEPROM_MAGIC_ADDR = 100;
const byte EEPROM_MAGIC = 0x7B;

void setup() {
  Serial.begin(9600);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  pinMode(BTN_CALIBRATE, INPUT_PULLUP);
  pinMode(BTN_DRYRUN, INPUT_PULLUP);

  // D12 is shared with BTN_MAIN/STBY; must remain INPUT_PULLUP.
  pinMode(BTN_MAIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  stopMotors();
  digitalWrite(LED_PIN, LOW);

  calibrated = loadCalibration();
  Serial.println(F("\n7-channel line follower ready."));
  Serial.println(F("Press D7=calibrate, D11=dry run, D12=main run."));
  Serial.println(F("If bits are all 0 or all 1 on both surfaces, check polarity and recalibrate."));
   Serial.println(F("Current polarity: BLACK_IS_HIGH = true"));
}

void loop() {
  if (buttonPressed(BTN_CALIBRATE)) {
    calibrateSensors();
  }

  if (buttonPressed(BTN_DRYRUN)) {
    if (!calibrated) {
      Serial.println(F("Calibrate first (D7)."));
      blinkLED(3);
    } else {
      runLineFollower(true);
    }
  }

  if (buttonPressed(BTN_MAIN)) {
    if (!calibrated) {
      Serial.println(F("Calibrate first (D7)."));
      blinkLED(3);
    } else {
      runLineFollower(false);
    }
  }

  readSensors();
  printSensorValues();
  delay(100);
}

// ---------------- BUTTONS ----------------
bool buttonPressed(uint8_t pin) {
  if (digitalRead(pin) != LOW) return false;
  delay(DEBOUNCE_MS);
  if (digitalRead(pin) != LOW) return false;
  while (digitalRead(pin) == LOW) delay(5);
  delay(DEBOUNCE_MS);
  return true;
}

bool anyRunButtonPressed() {
  return digitalRead(BTN_DRYRUN) == LOW || digitalRead(BTN_MAIN) == LOW;
}

// ---------------- SENSOR READING ----------------
void readSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorRaw[i] = analogRead(sensorPin[i]);
    bool onBlack = BLACK_IS_HIGH
      ? (sensorRaw[i] >= sensorThreshold[i])
      : (sensorRaw[i] <= sensorThreshold[i]);
    sensorOnLine[i] = onBlack ? 1 : 0;
  }
}

bool lineDetected() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorOnLine[i]) return true;
  }
  return false;
}

bool allSensorsOnBlack() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (!sensorOnLine[i]) return false;
  }
  return true;
}

// Weighted line error: approximately +3 (far left) to -3 (far right).
// Returns false if no sensor detects the line.
bool getLineError(float &lineError) {
  float weighted = 0.0;
  int total = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorOnLine[i]) {
      weighted += sensorWeight[i];
      total++;
    }
  }

  if (total == 0) return false;
  lineError = weighted / total;
  return true;
}

// ---------------- CALIBRATION ----------------
void calibrateSensors() {
  Serial.println(F("Calibration: move sensors over both black and white."));
  Serial.println(F("Robot rotates left, then right. Keep it on the track."));
  digitalWrite(LED_PIN, HIGH);

  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorMin[i] = 1023;
    sensorMax[i] = 0;
  }

  unsigned long start = millis();
  while (millis() - start < CALIBRATION_TIME_MS) {
    // Slow left/right sweep in alternating directions.
    unsigned long elapsed = millis() - start;
    if (elapsed < CALIBRATION_TIME_MS / 2) driveMotors(-calibrationSpeed, calibrationSpeed);
    else driveMotors(calibrationSpeed, -calibrationSpeed);

    sampleCalibration();
    delay(3);
  }
  stopMotors();

  // Threshold is midpoint between observed min and max.
  bool valid = true;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorMax[i] - sensorMin[i] < 80) valid = false;
    sensorThreshold[i] = (sensorMin[i] + sensorMax[i]) / 2;
  }

  if (!valid) {
    Serial.println(F("Calibration FAILED: sensor range too small."));
    Serial.println(F("Check sensor height, wiring, and black/white surface."));
    digitalWrite(LED_PIN, LOW);
    blinkLED(5);
    calibrated = false;
    return;
  }

  saveCalibration();
  calibrated = true;
  Serial.println(F("Calibration saved to EEPROM."));
  printCalibration();
  digitalWrite(LED_PIN, LOW);
  blinkLED(2);
}

void sampleCalibration() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    int v = analogRead(sensorPin[i]);
    if (v < sensorMin[i]) sensorMin[i] = v;
    if (v > sensorMax[i]) sensorMax[i] = v;
  }
}

void saveCalibration() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    EEPROM.put(i * sizeof(int), sensorMin[i]);
    EEPROM.put((NUM_SENSORS + i) * sizeof(int), sensorMax[i]);
    EEPROM.put((NUM_SENSORS * 2 + i) * sizeof(int), sensorThreshold[i]);
  }
  EEPROM.update(EEPROM_MAGIC_ADDR, EEPROM_MAGIC);
}

bool loadCalibration() {
  if (EEPROM.read(EEPROM_MAGIC_ADDR) != EEPROM_MAGIC) return false;

  bool valid = true;
  for (int i = 0; i < NUM_SENSORS; i++) {
    EEPROM.get(i * sizeof(int), sensorMin[i]);
    EEPROM.get((NUM_SENSORS + i) * sizeof(int), sensorMax[i]);
    EEPROM.get((NUM_SENSORS * 2 + i) * sizeof(int), sensorThreshold[i]);
    if (sensorMin[i] < 0 || sensorMax[i] > 1023 ||
        sensorMax[i] - sensorMin[i] < 40 ||
        sensorThreshold[i] < 0 || sensorThreshold[i] > 1023) {
      valid = false;
    }
  }
  if (valid) {
    Serial.println(F("Valid calibration loaded."));
    printCalibration();
  }
  return valid;
}

void printCalibration() {
  Serial.println(F("Index:   0    1    2    3    4    5    6"));
  Serial.print(F("Min:   "));
  for (int i = 0; i < NUM_SENSORS; i++) { Serial.print(sensorMin[i]); Serial.print(' '); }
  Serial.println();
  Serial.print(F("Max:   "));
  for (int i = 0; i < NUM_SENSORS; i++) { Serial.print(sensorMax[i]); Serial.print(' '); }
  Serial.println();
  Serial.print(F("Thres: "));
  for (int i = 0; i < NUM_SENSORS; i++) { Serial.print(sensorThreshold[i]); Serial.print(' '); }
  Serial.println();
}

// ---------------- PID FOLLOWING ----------------
void runLineFollower(bool dryRun) {
  Serial.println(dryRun ? F("DRY RUN started. Press any run button to stop.")
                        : F("MAIN RUN started. Press any run button to stop."));
  int speed = dryRun ? dryRunSpeed : baseSpeed;
  previousError = 0;
  digitalWrite(LED_PIN, HIGH);

  unsigned long allBlackSince = 0;
  unsigned long lineLostSince = 0;

  while (true) {
    if (digitalRead(BTN_DRYRUN) == LOW || digitalRead(BTN_MAIN) == LOW) {
      delay(DEBOUNCE_MS);
      if (digitalRead(BTN_DRYRUN) == LOW || digitalRead(BTN_MAIN) == LOW) {
        stopMotors();
        while (digitalRead(BTN_DRYRUN) == LOW || digitalRead(BTN_MAIN) == LOW) delay(5);
        break;
      }
    }

    readSensors();

    // Stop only when ALL seven sensors see black continuously.
    if (allSensorsOnBlack()) {
      if (allBlackSince == 0) allBlackSince = millis();
      if (millis() - allBlackSince >= BLACK_BOX_CONFIRM_MS) {
        stopMotors();
        Serial.println(F("BLACK END BOX DETECTED: motors stopped."));
        while (true) {
          digitalWrite(LED_PIN, HIGH);
          delay(250);
          digitalWrite(LED_PIN, LOW);
          delay(250);
        }
      }
    } else {
      allBlackSince = 0;
    }

    float weightedError = 0.0;
    if (!getLineError(weightedError)) {
      if (lineLostSince == 0) lineLostSince = millis();
      if (millis() - lineLostSince >= LINE_LOST_TIMEOUT_MS) {
        stopMotors();
        Serial.println(F("LINE LOST: recovery timed out, motors stopped."));
        break;
      }

      if (lastSeenSide < 0) driveMotors(-turnSpeed, turnSpeed);
      else if (lastSeenSide > 0) driveMotors(turnSpeed, -turnSpeed);
      else stopMotors();
      delay(2);
      continue;
    }

    lineLostSince = 0;
    errorValue = weightedError;
    if (errorValue > 0) lastSeenSide = -1;
    else if (errorValue < 0) lastSeenSide = 1;

    float derivative = errorValue - previousError;
    float correction = Kp * errorValue + Kd * derivative;
    previousError = errorValue;

    int left = constrain((int)(speed - correction), MIN_PWM, MAX_PWM);
    int right = constrain((int)(speed + correction), MIN_PWM, MAX_PWM);
    driveMotors(left, right);
  }

  stopMotors();
  digitalWrite(LED_PIN, LOW);
  Serial.println(F("Run stopped."));
}

// ---------------- MOTOR CONTROL ----------------
// Positive speed = forward. If a motor spins backward, swap its two wires
// or reverse the signs for that motor here.
void driveMotors(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  // Left motor uses BIN1/BIN2/PWMB.
  if (leftSpeed >= 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  }

  // Right motor uses AIN1/AIN2/PWMA.
  if (rightSpeed >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }

  analogWrite(PWMB, abs(leftSpeed));
  analogWrite(PWMA, abs(rightSpeed));
}

void stopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}

void blinkLED(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH); delay(120);
    digitalWrite(LED_PIN, LOW); delay(120);
  }
}

void printSensorValues() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(sensorRaw[i]);
    Serial.print('\t');
  }
  Serial.print(F(" | bits L->R: "));
  for (int i = 0; i < NUM_SENSORS; i++) Serial.print(sensorOnLine[i]);
  Serial.println();
}
