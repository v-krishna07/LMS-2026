// DRIFT CHECK - drives both motors with the SAME power and shows if the robot curves.
// Put the robot on the floor with free space in front. It drives straight for 3 seconds.
// How to read the result:
//   * Ticks: the wheel with MORE ticks is the stronger / faster one.
//   * Heading: the angle the robot turned. + means it curved to the RIGHT, - means LEFT.
//   * A big difference = one motor is stronger. The PID programs will fix this for you.
#include "../../pins.h"                       // all the pin numbers live in this file
#if !PINS_FILLED
  #error "Open pins.h, fill in the pin numbers, then set PINS_FILLED to true"
#endif
#include <Wire.h>                             // library for the IMU's 2-wire connection

const int TEST_POWER = 120;                   // same power for both wheels (0-255)

volatile long leftTicks = 0;                  // encoder counts
volatile long rightTicks = 0;
byte imuAddress = 0;                          // IMU address (found in setup)
float gyroBias = 0;                           // the gyro's "zero" value when the robot is still
float heading = 0;                            // degrees turned so far (+ = right)

void IRAM_ATTR leftEncoderISR() {             // runs by itself on every left encoder pulse
  if (digitalRead(LEFT_ENCODER_B) == HIGH) leftTicks -= LEFT_ENCODER_DIRECTION;
  else leftTicks += LEFT_ENCODER_DIRECTION;
}
void IRAM_ATTR rightEncoderISR() {            // runs by itself on every right encoder pulse
  if (digitalRead(RIGHT_ENCODER_B) == HIGH) rightTicks -= RIGHT_ENCODER_DIRECTION;
  else rightTicks += RIGHT_ENCODER_DIRECTION;
}

void setup() {
  Serial.begin(115200);                       // start the Serial Monitor
  pinMode(LEFT_MOTOR_IN1, OUTPUT);  pinMode(LEFT_MOTOR_IN2, OUTPUT);     // motor direction wires
  pinMode(RIGHT_MOTOR_IN1, OUTPUT); pinMode(RIGHT_MOTOR_IN2, OUTPUT);
  if (MOTOR_STANDBY_PIN >= 0) { pinMode(MOTOR_STANDBY_PIN, OUTPUT); digitalWrite(MOTOR_STANDBY_PIN, HIGH); }   // wake the driver up
  ledcAttach(LEFT_MOTOR_PWM, 20000, 8);       // motor speed wires
  ledcAttach(RIGHT_MOTOR_PWM, 20000, 8);
  pinMode(LEFT_ENCODER_A, INPUT);  pinMode(LEFT_ENCODER_B, INPUT);       // encoder wires only listen
  pinMode(RIGHT_ENCODER_A, INPUT); pinMode(RIGHT_ENCODER_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), leftEncoderISR, RISING);     // call the functions on every pulse
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), rightEncoderISR, RISING);
  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);       // start the IMU connection
  for (byte address = 0x6A; address <= 0x6B; address++) {   // look for the IMU at 0x6A or 0x6B
    Wire.beginTransmission(address); Wire.write(0x0F); Wire.endTransmission(false);
    Wire.requestFrom(address, (byte)1);
    if (Wire.read() == 0x6C) imuAddress = address;          // the LSM6DSO answers 0x6C
  }
  if (imuAddress == 0) { Serial.println("IMU NOT FOUND"); while (true) delay(1000); }
  Wire.beginTransmission(imuAddress); Wire.write(0x11); Wire.write(0x74); Wire.endTransmission();   // turn the gyro on
  Serial.println("Keep the robot still for 1 second...");
  float total = 0;                            // add up 200 gyro readings while the robot is still
  for (int i = 0; i < 200; i++) {
    Wire.beginTransmission(imuAddress); Wire.write(0x26); Wire.endTransmission(false);   // gyro Z register
    Wire.requestFrom(imuAddress, (byte)2);
    int low = Wire.read();
    int high = Wire.read();
    total += (int16_t)(high << 8 | low) * 0.0175;     // turn the 2 bytes into degrees per second
    delay(5);
  }
  gyroBias = total / 200;                     // the average = the gyro's zero value
}

void loop() {
  Serial.println("Put the robot on the floor. Driving in 3 seconds...");
  delay(3000);                                // time to put the robot down
  leftTicks = 0; rightTicks = 0; heading = 0; // start counting from zero
  unsigned long lastTime = micros();          // used to measure the time between gyro readings
  digitalWrite(LEFT_MOTOR_IN1, LEFT_MOTOR_DIRECTION > 0);    // both wheels forward
  digitalWrite(LEFT_MOTOR_IN2, LEFT_MOTOR_DIRECTION < 0);
  digitalWrite(RIGHT_MOTOR_IN1, RIGHT_MOTOR_DIRECTION > 0);
  digitalWrite(RIGHT_MOTOR_IN2, RIGHT_MOTOR_DIRECTION < 0);
  ledcWrite(LEFT_MOTOR_PWM, TEST_POWER);      // same power on both motors
  ledcWrite(RIGHT_MOTOR_PWM, TEST_POWER);
  unsigned long startTime = millis();
  while (millis() - startTime < 3000) {       // drive for 3 seconds
    Wire.beginTransmission(imuAddress); Wire.write(0x26); Wire.endTransmission(false);   // read the gyro
    Wire.requestFrom(imuAddress, (byte)2);
    int low = Wire.read();
    int high = Wire.read();
    float gyroSpeed = (int16_t)(high << 8 | low) * 0.0175 - gyroBias;    // degrees per second
    unsigned long now = micros();
    heading += IMU_DIRECTION * gyroSpeed * (now - lastTime) * 0.000001;  // speed x time = angle turned
    lastTime = now;
  }
  ledcWrite(LEFT_MOTOR_PWM, 0);               // stop both motors
  ledcWrite(RIGHT_MOTOR_PWM, 0);
  Serial.print("Left ticks: ");  Serial.print(leftTicks);                // show the result
  Serial.print("   Right ticks: "); Serial.print(rightTicks);
  Serial.print("   Difference (left - right): "); Serial.print(leftTicks - rightTicks);
  Serial.print("   Heading (degrees): "); Serial.println(heading);
  delay(5000);                                // wait, then repeat
}
