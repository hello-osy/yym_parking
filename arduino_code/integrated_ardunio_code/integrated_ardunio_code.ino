/*
  Integrated one-Arduino firmware for the parking vehicle (Arduino Mega 2560).

  This file is in its own Arduino sketch directory so the two legacy sketches
  in the parent directory are not compiled with it.

  Serial input:  "<steer> <speed>\n"  (for example: "-30 20")
  Serial output:
    "U,<r1>,<r2>,<r3>,<r4>,<r5>,<r6>\n" in metres.
    "R,<raw>,<voltage>,<percent>\n" for the A1 rotation sensor.

  Motor control has priority:
    - commands are serviced before and after every ultrasonic measurement;
    - the 500 ms watchdog is checked during the measurement cycle;
    - ultrasonic values are buffered and emitted as one complete line only
      after all six measurements have finished.
*/

#include <Arduino.h>

constexpr uint8_t STEER_IN1 = 7;
constexpr uint8_t STEER_IN2 = 6;
constexpr uint8_t MOTOR1_IN1 = 13;
constexpr uint8_t MOTOR1_IN2 = 12;
constexpr uint8_t MOTOR2_IN1 = 11;
constexpr uint8_t MOTOR2_IN2 = 10;

constexpr uint8_t SENSOR_COUNT = 6;
constexpr uint8_t TRIG_PINS[SENSOR_COUNT] = {22, 30, 26, 34, 38, 44};
constexpr uint8_t ECHO_PINS[SENSOR_COUNT] = {23, 31, 27, 35, 39, 45};
constexpr uint8_t ROTATION_SENSOR_PIN = A1;
constexpr uint8_t ROTATION_SAMPLE_COUNT = 8;

constexpr int MAX_STEER_PWM = 150;
constexpr int MAX_DRIVE_PWM = 230;
constexpr unsigned long COMMAND_TIMEOUT_MS = 500;
constexpr unsigned long SENSOR_PERIOD_MS = 180;
constexpr unsigned long ECHO_TIMEOUT_US = 20000;
constexpr unsigned long SENSOR_GUARD_US = 3000;
// 실차 측정 후 이 두 시간만 조정한다.
constexpr unsigned long LEFT_END_TIME_MS = 2000;
constexpr unsigned long LEFT_TO_CENTER_TIME_MS = 450;
constexpr unsigned long STEER_SETTLE_TIME_MS = 300;


char commandBuffer[32];
uint8_t commandLength = 0;
unsigned long lastCommandAt = 0;
unsigned long lastSensorAt = 0;
bool watchdogStopped = false;
bool discardCommandUntilNewline = false;

int clampValue(int value, int limit) {
  return constrain(value, -limit, limit);
}

void setSteer(int value) {
  const int pwm = abs(clampValue(value, MAX_STEER_PWM));
  if (value > 0) {
    analogWrite(STEER_IN1, pwm);
    digitalWrite(STEER_IN2, LOW);
  } else if (value < 0) {
    digitalWrite(STEER_IN1, LOW);
    analogWrite(STEER_IN2, pwm);
  } else {
    digitalWrite(STEER_IN1, LOW);
    digitalWrite(STEER_IN2, LOW);
  }
}

void setDrive(int value) {
  const int pwm = abs(clampValue(value, MAX_DRIVE_PWM));
  if (value > 0) {
    analogWrite(MOTOR1_IN1, pwm);
    digitalWrite(MOTOR1_IN2, LOW);
    analogWrite(MOTOR2_IN1, pwm);
    digitalWrite(MOTOR2_IN2, LOW);
  } else if (value < 0) {
    digitalWrite(MOTOR1_IN1, LOW);
    analogWrite(MOTOR1_IN2, pwm);
    digitalWrite(MOTOR2_IN1, LOW);
    analogWrite(MOTOR2_IN2, pwm);
  } else {
    digitalWrite(MOTOR1_IN1, LOW);
    digitalWrite(MOTOR1_IN2, LOW);
    digitalWrite(MOTOR2_IN1, LOW);
    digitalWrite(MOTOR2_IN2, LOW);
  }
}

void stopAll() {
  setSteer(0);
  setDrive(0);
}

// void homeSteering() {
//   setDrive(0);

//   // 현재 위치와 관계없이 왼쪽 끝까지 이동한다.
//   setSteer(-MAX_STEER_PWM);
//   delay(LEFT_END_TIME_MS);
//   setSteer(0);
//   delay(STEER_SETTLE_TIME_MS);

//   // 왼쪽 끝에서 중앙까지 복귀한다.
//   setSteer(MAX_STEER_PWM);
//   delay(LEFT_TO_CENTER_TIME_MS);
//   stopAll();
// }

void processCommand() {
  commandBuffer[commandLength] = '\0';
  int steer = 0;
  int speed = 0;
  if (sscanf(commandBuffer, "%d %d", &steer, &speed) == 2) {
    setSteer(steer);
    setDrive(speed);
    lastCommandAt = millis();
    watchdogStopped = false;
  }
  commandLength = 0;
}

void readCommands() {
  while (Serial.available() > 0) {
    const char value = static_cast<char>(Serial.read());
    if (value == '\n' || value == '\r') {
      if (!discardCommandUntilNewline && commandLength > 0) {
        processCommand();
      }
      discardCommandUntilNewline = false;
      commandLength = 0;
    } else if (discardCommandUntilNewline) {
      continue;
    } else if (commandLength < sizeof(commandBuffer) - 1) {
      commandBuffer[commandLength++] = value;
    } else {
      // Discard an overlong/corrupt command without refreshing the watchdog.
      commandLength = 0;
      discardCommandUntilNewline = true;
    }
  }
}

void enforceCommandWatchdog() {
  if (millis() - lastCommandAt > COMMAND_TIMEOUT_MS && !watchdogStopped) {
    stopAll();
    watchdogStopped = true;
  }
}

void serviceMotorControl() {
  readCommands();
  enforceCommandWatchdog();
}

float readDistance(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(3);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  const unsigned long duration = pulseIn(echoPin, HIGH, ECHO_TIMEOUT_US);
  if (duration == 0) {
    return NAN;
  }
  return static_cast<float>(duration) * 0.000343F / 2.0F;
}

void publishUltrasonics() {
  float distances[SENSOR_COUNT];

  for (uint8_t index = 0; index < SENSOR_COUNT; ++index) {
    serviceMotorControl();
    distances[index] = readDistance(TRIG_PINS[index], ECHO_PINS[index]);
    serviceMotorControl();
    if (index + 1 < SENSOR_COUNT) {
      delayMicroseconds(SENSOR_GUARD_US);
    }
  }

  // 초음파와 회전 센서는 서로 다른 접두사의 완전한 한 줄 프레임으로 보내
  // 수신 측에서 U/R 데이터를 안전하게 구분할 수 있게 한다.
  Serial.print(F("U"));
  for (uint8_t index = 0; index < SENSOR_COUNT; ++index) {
    Serial.print(',');
    if (isnan(distances[index])) {
      Serial.print(F("nan"));
    } else {
      Serial.print(distances[index], 3);
    }
  }
  Serial.println();
}

void publishRotationSensor() {
  unsigned long sum = 0;
  for (uint8_t index = 0; index < ROTATION_SAMPLE_COUNT; ++index) {
    sum += analogRead(ROTATION_SENSOR_PIN);
  }

  const int rawValue = sum / ROTATION_SAMPLE_COUNT;
  const float voltage = rawValue * (5.0F / 1023.0F);
  const int percent = map(rawValue, 0, 1023, 0, 100);

  Serial.print(F("R,"));
  Serial.print(rawValue);
  Serial.print(',');
  Serial.print(voltage, 2);
  Serial.print(',');
  Serial.println(percent);
}

void setup() {
  Serial.begin(115200);
  pinMode(STEER_IN1, OUTPUT);
  pinMode(STEER_IN2, OUTPUT);
  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);
  pinMode(MOTOR2_IN1, OUTPUT);
  pinMode(MOTOR2_IN2, OUTPUT);
  pinMode(ROTATION_SENSOR_PIN, INPUT);
  for (uint8_t index = 0; index < SENSOR_COUNT; ++index) {
    pinMode(TRIG_PINS[index], OUTPUT);
    pinMode(ECHO_PINS[index], INPUT);
    digitalWrite(TRIG_PINS[index], LOW);
  }

  stopAll();
  // homeSteering();
  lastCommandAt = millis();
  lastSensorAt = millis();
}

void loop() {
  serviceMotorControl();
  const unsigned long now = millis();
  if (now - lastSensorAt >= SENSOR_PERIOD_MS) {
    lastSensorAt = now;
    // 순서: 조향/주행 명령 처리 -> 초음파 측정/전송 -> A1 회전값 전송.
    serviceMotorControl();
    publishUltrasonics();
    publishRotationSensor();
  }
}
