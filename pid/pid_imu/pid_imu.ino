// PID IMU - holds a direction using ONLY the gyro (no line, no encoders).
// How it works, in simple words:
//   The gyro tells how fast we turn. Adding that up over time gives "heading" = how many degrees we have turned.
//   error = wanted heading - real heading. The PID turns the robot until the error is zero.
// Two ways to use it:
//   * SPEED = 100: the robot drives straight and keeps its direction.
//   * SPEED = 0 (default): the robot stays in place. Each button press turns it 90 degrees RIGHT.
//     This tests how good the 90 and 180 degree turns are.
// Keep the robot STILL for 1 second after power on (the gyro finds its zero).
#include "../../pins.h"                       // all the pin numbers live in this file
#if !PINS_FILLED
  #error "Open pins.h, fill in the pin numbers, then set PINS_FILLED to true"
#endif
#include <Wire.h>                             // library for the IMU's 2-wire connection

// ---------- numbers you can change (tune these) ----------
float KP = 3.0;           // P = how hard to turn back to the wanted direction
float KI = 0.0;           // I = helps when the robot stops a little short of the target
float KD = 0.5;           // D = stops the robot from overshooting
int SPEED = 0;            // forward power while holding the direction (0 = only turn on the spot)

// ---------- what the robot is doing right now (the "mode") ----------
const int GYRO_ZERO = 0;  // keeping still while the gyro finds its zero
const int HOLD = 1;       // holding the wanted direction
int mode = GYRO_ZERO;     // we start with the gyro zero

// ---------- variables the program uses ----------
byte imuAddress = 0;      // IMU address (found in setup)
float gyroBias = 0;       // the gyro's "zero" value when the robot is still
float gyroTotal = 0;      // adds up the gyro readings while still
int gyroCount = 0;        // how many readings we added
float heading = 0;        // degrees turned so far (+ = right)
float targetHeading = 0;  // the direction we want
float error = 0;          // wanted heading - real heading
float lastError = 0;      // the error in the previous loop
float integral = 0;       // the errors added up over time
unsigned long lastMicros = 0;      // time of the last loop (to measure how long a loop took)
unsigned long startTime = 0;       // when the gyro zero started
unsigned long lastPrintTime = 0;   // when we last printed
bool wasPressed = false;           // was the button pressed in the last loop?

void setup() {
  Serial.begin(115200);                       // start the Serial Monitor
  pinMode(LEFT_MOTOR_IN1, OUTPUT);            // motor direction wires
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);
  if (MOTOR_STANDBY_PIN >= 0) {               // if the driver's STBY wire goes to the ESP32
    pinMode(MOTOR_STANDBY_PIN, OUTPUT);
    digitalWrite(MOTOR_STANDBY_PIN, HIGH);    // HIGH = wake the driver up
  }
  ledcAttach(LEFT_MOTOR_PWM, 20000, 8);       // motor speed wires
  ledcAttach(RIGHT_MOTOR_PWM, 20000, 8);
  pinMode(BUTTON_PIN, INPUT_PULLUP);          // button uses the built-in pull-up
  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);       // start the IMU connection
  for (byte address = 0x6A; address <= 0x6B; address++) {   // look for the IMU at 0x6A or 0x6B
    Wire.beginTransmission(address); Wire.write(0x0F); Wire.endTransmission(false);
    Wire.requestFrom(address, (byte)1);
    if (Wire.read() == 0x6C) imuAddress = address;          // the LSM6DSO answers 0x6C
  }
  if (imuAddress == 0) { Serial.println("IMU NOT FOUND"); while (true) delay(1000); }
  Wire.beginTransmission(imuAddress); Wire.write(0x11); Wire.write(0x74); Wire.endTransmission();   // turn the gyro on
  Serial.println("Keep the robot still for 1 second...");
  startTime = millis();                       // remember when the stillness started
  lastMicros = micros();                      // start the loop timer
}

void loop() {
  // ===== 1. READ THE GYRO =====
  Wire.beginTransmission(imuAddress);         // talk to the IMU
  Wire.write(0x26);                           // gyro Z numbers start at 0x26
  Wire.endTransmission(false);
  Wire.requestFrom(imuAddress, (byte)2);      // read 2 bytes
  int low = Wire.read();                      // first byte (low part)
  int high = Wire.read();                     // second byte (high part)
  float gyroRaw = (int16_t)(high << 8 | low) * 0.0175;   // degrees per second (still includes the zero offset)
  float gyroSpeed = gyroRaw - gyroBias;       // real turning speed
  unsigned long now = micros();               // time now
  heading += IMU_DIRECTION * gyroSpeed * (now - lastMicros) * 0.000001;   // speed x time = angle turned
  lastMicros = now;                           // remember the time for the next loop

  // ===== 2. DECIDE WHAT TO DO =====
  int leftSpeed = 0;                          // power for the left motor (0 = stopped)
  int rightSpeed = 0;                         // power for the right motor

  if (mode == GYRO_ZERO) {                    // --- finding the gyro zero (robot still) ---
    gyroTotal += gyroRaw;                     // add up the readings
    gyroCount++;
    if (millis() - startTime > 1000) {        // 1 second is over
      gyroBias = gyroTotal / gyroCount;       // the average reading = the zero value
      heading = 0;                            // start counting the angle from zero
      Serial.println("Ready. Press the button to turn 90 degrees right.");
      mode = HOLD;
    }
  }
  else if (mode == HOLD) {                    // --- holding the wanted direction ---
    bool isPressed = (digitalRead(BUTTON_PIN) == BUTTON_PRESSED);   // is the button down?
    if (isPressed && !wasPressed) targetHeading += 90;              // a new press = turn 90 degrees more to the right
    wasPressed = isPressed;                   // remember for the next loop
    error = targetHeading - heading;          // + means we must turn RIGHT
    integral = constrain(integral + error, -500, 500);   // I part: errors added up (limited)
    float correction = KP * error + KI * integral + KD * (error - lastError);   // the PID formula
    lastError = error;                        // remember for the next loop
    leftSpeed = SPEED + correction;           // positive correction = left wheel faster = turn right
    rightSpeed = SPEED - correction;
    if (millis() - lastPrintTime > 100) {     // print 10 times per second
      lastPrintTime = millis();
      Serial.print("target "); Serial.print(targetHeading);
      Serial.print("  heading "); Serial.print(heading);
      Serial.print("  error "); Serial.println(error);
    }
  }

  // ===== 3. SEND THE POWER TO THE MOTORS =====
  leftSpeed = constrain(leftSpeed * LEFT_MOTOR_DIRECTION, -MAX_PWM, MAX_PWM);      // flip if wired backwards, never above the limit
  rightSpeed = constrain(rightSpeed * RIGHT_MOTOR_DIRECTION, -MAX_PWM, MAX_PWM);
  digitalWrite(LEFT_MOTOR_IN1, leftSpeed >= 0);    // forward: IN1 high, IN2 low
  digitalWrite(LEFT_MOTOR_IN2, leftSpeed <= 0);    // backward: IN1 low, IN2 high (both high = brake)
  ledcWrite(LEFT_MOTOR_PWM, abs(leftSpeed));       // speed of the left motor
  digitalWrite(RIGHT_MOTOR_IN1, rightSpeed >= 0);
  digitalWrite(RIGHT_MOTOR_IN2, rightSpeed <= 0);
  ledcWrite(RIGHT_MOTOR_PWM, abs(rightSpeed));     // speed of the right motor
  delay(5);                                        // tiny wait, then repeat
}
