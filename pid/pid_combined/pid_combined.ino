// PID COMBINED - IR array + encoders + gyro working together (steady line following).
//   * IR array + PID : steers the robot along the line.
//   * Encoders       : measure the real speed, so the robot keeps the SAME speed even when the battery gets weaker.
//   * Gyro           : if the line is lost for a moment (a gap), the robot keeps going straight in the same direction.
// How to start: keep the robot still for 1 second at power on (the gyro finds its zero), then the LED is on for
// 4 seconds = slide the array over the line and the black floor. Then put the robot on the line and press the button.
#include "../../pins.h"                       // all the pin numbers live in this file
#if !PINS_FILLED
  #error "Open pins.h, fill in the pin numbers, then set PINS_FILLED to true"
#endif
#include <Wire.h>                             // library for the IMU's 2-wire connection

// ---------- numbers you can change (tune these) ----------
float KP = 12.0;          // P = steer harder when the line is far from the middle
float KI = 0.0;           // I = fixes a small constant sideways drift (keep 0 at first)
float KD = 30.0;          // D = calms the wobble
int TARGET_TICKS = 20;    // wanted wheel speed = encoder ticks per loop (one loop is about 10 ms). Find a good number with encoder_check
float SPEED_GAIN = 0.5;   // how fast the power is corrected when the real speed is wrong
float HEADING_GAIN = 3.0; // how hard the gyro steers straight when the line is lost
int LOST_TIME = 400;      // when the line is lost, go straight for this many milliseconds, then stop

// ---------- what the robot is doing right now (the "mode") ----------
const int GYRO_ZERO = 0;  // keeping still while the gyro finds its zero
const int CALIBRATE = 1;  // learning what white and black look like
const int WAIT = 2;       // waiting for the button
const int FOLLOW = 3;     // following the line
int mode = GYRO_ZERO;     // we start with the gyro zero

// ---------- variables the program uses ----------
int sensorMin[16];        // smallest reading of each sensor (black floor)
int sensorMax[16];        // biggest reading of each sensor (white line)
int sensorValue[16];      // each sensor as a number 0-1000 (1000 = sees the line)
float error = 0;          // where the line is: 0 = middle, negative = left, positive = right
float lastError = 0;      // the error in the previous loop
float integral = 0;       // the errors added up over time
volatile long leftTicks = 0;      // wheel counts (changed by the interrupts)
volatile long rightTicks = 0;
long lastLeftTicks = 0;           // wheel counts at the end of the previous loop
long lastRightTicks = 0;
float basePower = 100;            // power given to both wheels (the encoders keep adjusting it)
byte imuAddress = 0;              // IMU address (found in setup)
float gyroBias = 0;               // the gyro's "zero" value when the robot is still
float gyroTotal = 0;              // adds up the gyro readings while still
int gyroCount = 0;                // how many readings we added
float heading = 0;                // degrees turned so far (+ = right)
float headingWhenLost = 0;        // the direction we were facing when the line was lost
unsigned long lostSince = 0;      // when the line was lost (0 = not lost)
unsigned long lastMicros = 0;     // time of the last loop
unsigned long startTime = 0;      // when the current start-up step began
unsigned long lastPrintTime = 0;  // when we last printed

void IRAM_ATTR leftEncoderISR() {             // runs by itself on every left encoder pulse
  if (digitalRead(LEFT_ENCODER_B) == HIGH) leftTicks -= LEFT_ENCODER_DIRECTION;   // wire B tells the direction
  else leftTicks += LEFT_ENCODER_DIRECTION;
}
void IRAM_ATTR rightEncoderISR() {            // runs by itself on every right encoder pulse
  if (digitalRead(RIGHT_ENCODER_B) == HIGH) rightTicks -= RIGHT_ENCODER_DIRECTION;
  else rightTicks += RIGHT_ENCODER_DIRECTION;
}

void setup() {
  Serial.begin(115200);                       // start the Serial Monitor
  pinMode(IR_S0_PIN, OUTPUT);                 // the 4 select wires send the sensor number to the array
  pinMode(IR_S1_PIN, OUTPUT);
  pinMode(IR_S2_PIN, OUTPUT);
  pinMode(IR_S3_PIN, OUTPUT);
  analogReadResolution(12);                   // analog readings go from 0 to 4095
  analogSetPinAttenuation(IR_SIGNAL_PIN, ADC_11db);   // read up to about 3.3 volts
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
  pinMode(LEFT_ENCODER_A, INPUT);             // encoder wires only listen
  pinMode(LEFT_ENCODER_B, INPUT);
  pinMode(RIGHT_ENCODER_A, INPUT);
  pinMode(RIGHT_ENCODER_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), leftEncoderISR, RISING);     // run the function on every pulse
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), rightEncoderISR, RISING);
  pinMode(LED_PIN, OUTPUT);                   // LED sends power out
  pinMode(BUTTON_PIN, INPUT_PULLUP);          // button uses the built-in pull-up
  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);       // start the IMU connection
  for (byte address = 0x6A; address <= 0x6B; address++) {   // look for the IMU at 0x6A or 0x6B
    Wire.beginTransmission(address); Wire.write(0x0F); Wire.endTransmission(false);
    Wire.requestFrom(address, (byte)1);
    if (Wire.read() == 0x6C) imuAddress = address;          // the LSM6DSO answers 0x6C
  }
  if (imuAddress == 0) { Serial.println("IMU NOT FOUND"); while (true) delay(1000); }
  Wire.beginTransmission(imuAddress); Wire.write(0x11); Wire.write(0x74); Wire.endTransmission();   // turn the gyro on
  for (int i = 0; i < 16; i++) {              // start values for the calibration
    sensorMin[i] = 4095;
    sensorMax[i] = 0;
  }
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

  // ===== 2. READ THE 16 SENSORS =====
  float weightedSum = 0;                      // adds up (sensor value x sensor position)
  float totalSum = 0;                         // adds up all the sensor values
  int sensorsOnLine = 0;                      // counts how many sensors see the line
  for (int i = 0; i < 16; i++) {              // i = sensor number, 0 = left-most
    int channel = IR_CHANNEL0_IS_RIGHT ? 15 - i : i;   // flip the order if sensor 0 is on the right
    digitalWrite(IR_S0_PIN, channel & 1);     // write the channel number on the 4 select wires
    digitalWrite(IR_S1_PIN, (channel >> 1) & 1);
    digitalWrite(IR_S2_PIN, (channel >> 2) & 1);
    digitalWrite(IR_S3_PIN, (channel >> 3) & 1);
    delayMicroseconds(10);                    // give the multiplexer a moment to switch
    int raw = analogRead(IR_SIGNAL_PIN);      // raw reading 0-4095
    if (mode == CALIBRATE) {                  // while calibrating, remember the smallest and biggest readings
      sensorMin[i] = min(sensorMin[i], raw);
      sensorMax[i] = max(sensorMax[i], raw);
    }
    int value = (raw - sensorMin[i]) * 1000L / max(sensorMax[i] - sensorMin[i], 1);   // squeeze into 0-1000
    value = constrain(value, 0, 1000);        // keep it inside 0-1000
    if (!WHITE_LINE_IS_HIGH) value = 1000 - value;   // flip it if white gives a smaller number
    sensorValue[i] = value;                   // save it
    if (value > 500) sensorsOnLine++;         // this sensor sees the line
    weightedSum += value * IR_POSITION[i];    // strong sensors on the right push the average to the right
    totalSum += value;
  }

  // ===== 3. DECIDE WHAT TO DO =====
  int leftSpeed = 0;                          // power for the left motor (0 = stopped unless we change it)
  int rightSpeed = 0;                         // power for the right motor

  if (mode == GYRO_ZERO) {                    // --- finding the gyro zero (robot still) ---
    gyroTotal += gyroRaw;                     // add up the readings
    gyroCount++;
    if (millis() - startTime > 1000) {        // 1 second is over
      gyroBias = gyroTotal / gyroCount;       // the average reading = the zero value
      Serial.println("Calibrating for 4 seconds: slide the array over the line and the black floor");
      digitalWrite(LED_PIN, HIGH);            // LED on while calibrating
      startTime = millis();
      mode = CALIBRATE;
    }
  }
  else if (mode == CALIBRATE) {               // --- calibrating the IR sensors ---
    if (millis() - startTime > 4000) {        // 4 seconds are over
      digitalWrite(LED_PIN, LOW);             // LED off
      Serial.println("Put the robot on the line and press the button");
      mode = WAIT;
    }
  }
  else if (mode == WAIT) {                    // --- waiting for the button ---
    if (digitalRead(BUTTON_PIN) == BUTTON_PRESSED) {   // button pressed
      delay(1000);                            // time to take your hand away
      lastLeftTicks = leftTicks;              // start measuring the speed from now
      lastRightTicks = rightTicks;
      lastMicros = micros();                  // restart the loop timer
      mode = FOLLOW;
    }
  }
  else if (mode == FOLLOW) {                  // --- following the line ---
    long leftNow = leftTicks;                 // copy the counts (the interrupts keep changing them)
    long rightNow = rightTicks;
    float realSpeed = ((leftNow - lastLeftTicks) + (rightNow - lastRightTicks)) / 2.0;   // average ticks since the last loop
    lastLeftTicks = leftNow;                  // remember the counts for the next loop
    lastRightTicks = rightNow;
    basePower = constrain(basePower + SPEED_GAIN * (TARGET_TICKS - realSpeed), 0, MAX_PWM);   // too slow -> more power, too fast -> less
    float correction = 0;                     // how much to steer
    if (sensorsOnLine > 0) {                  // we can see the line
      lostSince = 0;                          // not lost
      error = weightedSum / totalSum;         // average position of the line (-7.5 to +7.5)
      integral = constrain(integral + error, -100, 100);   // I part: errors added up (limited)
      correction = KP * error + KI * integral + KD * (error - lastError);   // the PID formula
      lastError = error;                      // remember for the next loop
    }
    else {                                    // the line is lost
      if (lostSince == 0) {                   // it was just lost
        lostSince = millis();                 // remember when
        headingWhenLost = heading;            // and which way we were facing
      }
      correction = HEADING_GAIN * (headingWhenLost - heading);   // the gyro keeps the robot straight
    }
    if (lostSince != 0 && millis() - lostSince > LOST_TIME) {    // lost for too long: stop
      basePower = 100;                        // restart value for the speed control
    }
    else {                                    // normal driving
      leftSpeed = basePower + correction;     // positive correction = turn right
      rightSpeed = basePower - correction;
    }
    if (millis() - lastPrintTime > 100) {     // print 10 times per second
      lastPrintTime = millis();
      Serial.print("error "); Serial.print(error);
      Serial.print("  speed "); Serial.print(realSpeed);
      Serial.print("  power "); Serial.print(basePower);
      Serial.print("  heading "); Serial.println(heading);
    }
  }

  // ===== 4. SEND THE POWER TO THE MOTORS =====
  leftSpeed = constrain(leftSpeed * LEFT_MOTOR_DIRECTION, -MAX_PWM, MAX_PWM);      // flip if wired backwards, never above the limit
  rightSpeed = constrain(rightSpeed * RIGHT_MOTOR_DIRECTION, -MAX_PWM, MAX_PWM);
  digitalWrite(LEFT_MOTOR_IN1, leftSpeed >= 0);    // forward: IN1 high, IN2 low
  digitalWrite(LEFT_MOTOR_IN2, leftSpeed <= 0);    // backward: IN1 low, IN2 high (both high = brake)
  ledcWrite(LEFT_MOTOR_PWM, abs(leftSpeed));       // speed of the left motor
  digitalWrite(RIGHT_MOTOR_IN1, rightSpeed >= 0);
  digitalWrite(RIGHT_MOTOR_IN2, rightSpeed <= 0);
  ledcWrite(RIGHT_MOTOR_PWM, abs(rightSpeed));     // speed of the right motor
  delay(10);                                       // one loop = about 10 ms (the speed control counts ticks per loop)
}
