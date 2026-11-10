// TEST EVERYTHING - one program to check all the hardware.
// Open the Serial Monitor (115200) and TYPE a letter, then press Enter:
//   h = help     c = chip (ESP32)   l = LED       b = button
//   i = IR array g = IMU            e = encoders  m = motors (lift the robot!)
//   d = drift test (robot drives straight for 3 seconds, put it on the floor)
#include "../../pins.h"                       // all the pin numbers live in this file
#if !PINS_FILLED
  #error "Open pins.h, fill in the pin numbers, then set PINS_FILLED to true"
#endif
#include <Wire.h>                             // library for the IMU's 2-wire connection

volatile long leftTicks = 0;                  // encoder counts
volatile long rightTicks = 0;
byte imuAddress = 0;                          // IMU address (found in setup)

// The 5 motor steps: m runs steps 0-3 (one motor at a time), d runs step 4 (both motors together).
const char* STEP_NAME[5] = {"LEFT forward", "LEFT backward", "RIGHT forward", "RIGHT backward", "BOTH forward"};
const int LEFT_TEST[5]  = {120, -120, 0, 0, 120};
const int RIGHT_TEST[5] = {0, 0, 120, -120, 120};
const int STEP_TIME[5]  = {1500, 1500, 1500, 1500, 3000};     // how long each step runs (milliseconds)

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
  pinMode(LED_PIN, OUTPUT);                   // LED sends power out
  pinMode(BUTTON_PIN, INPUT_PULLUP);          // button uses the built-in pull-up
  pinMode(IR_S0_PIN, OUTPUT); pinMode(IR_S1_PIN, OUTPUT); pinMode(IR_S2_PIN, OUTPUT); pinMode(IR_S3_PIN, OUTPUT);   // IR select wires
  analogReadResolution(12);                   // analog readings go from 0 to 4095
  analogSetPinAttenuation(IR_SIGNAL_PIN, ADC_11db);   // read up to about 3.3 volts
  pinMode(LEFT_MOTOR_IN1, OUTPUT);  pinMode(LEFT_MOTOR_IN2, OUTPUT);     // motor direction wires
  pinMode(RIGHT_MOTOR_IN1, OUTPUT); pinMode(RIGHT_MOTOR_IN2, OUTPUT);
  if (MOTOR_STANDBY_PIN >= 0) { pinMode(MOTOR_STANDBY_PIN, OUTPUT); digitalWrite(MOTOR_STANDBY_PIN, HIGH); }   // wake the driver up
  ledcAttach(LEFT_MOTOR_PWM, 20000, 8);       // motor speed wires
  ledcAttach(RIGHT_MOTOR_PWM, 20000, 8);
  pinMode(LEFT_ENCODER_A, INPUT);  pinMode(LEFT_ENCODER_B, INPUT);       // encoder wires only listen
  pinMode(RIGHT_ENCODER_A, INPUT); pinMode(RIGHT_ENCODER_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), rightEncoderISR, RISING);
  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);       // start the IMU connection
  for (byte address = 0x6A; address <= 0x6B; address++) {   // look for the IMU at 0x6A or 0x6B
    Wire.beginTransmission(address); Wire.write(0x0F); Wire.endTransmission(false);
    Wire.requestFrom(address, (byte)1);
    if (Wire.read() == 0x6C) imuAddress = address;          // the LSM6DSO answers 0x6C
  }
  if (imuAddress != 0) {                      // turn the accelerometer and the gyro on
    Wire.beginTransmission(imuAddress); Wire.write(0x10); Wire.write(0x74); Wire.endTransmission();
    Wire.beginTransmission(imuAddress); Wire.write(0x11); Wire.write(0x74); Wire.endTransmission();
  }
  Serial.println("TEST EVERYTHING ready. Type h for help.");
}

void loop() {
  if (Serial.available() == 0) return;        // nothing typed yet, try again
  char key = Serial.read();                   // read the letter you typed

  if (key == 'h') {                           // HELP
    Serial.println("c=chip  l=LED  b=button  i=IR array  g=IMU  e=encoders  m=motors  d=drift");
  }

  if (key == 'c') {                           // ESP32 CHECK
    Serial.print("Chip: ");  Serial.print(ESP.getChipModel());
    Serial.print("  speed MHz: ");  Serial.print(ESP.getCpuFreqMHz());
    Serial.print("  free memory: ");  Serial.println(ESP.getFreeHeap());
    Serial.println("ESP32 WORKING");
  }

  if (key == 'l') {                           // LED CHECK: blink 3 times
    for (int i = 0; i < 3; i++) {
      digitalWrite(LED_PIN, HIGH); delay(300);   // on
      digitalWrite(LED_PIN, LOW);  delay(300);   // off
    }
    Serial.println("LED blinked 3 times - did you see it?");
  }

  if (key == 'b') {                           // BUTTON CHECK: watch for 5 seconds
    Serial.println("Press the button within 5 seconds...");
    bool pressed = false;                     // becomes true if we see a press
    unsigned long startTime = millis();
    while (millis() - startTime < 5000) {
      if (digitalRead(BUTTON_PIN) == BUTTON_PRESSED) pressed = true;
    }
    if (pressed) Serial.println("BUTTON WORKING");
    else Serial.println("Button was not pressed");
  }

  if (key == 'i') {                           // IR ARRAY CHECK: 10 readings
    Serial.println("IR array (left to right):");
    for (int reading = 0; reading < 10; reading++) {
      for (int i = 0; i < 16; i++) {          // i = sensor number, 0 = left-most
        int channel = IR_CHANNEL0_IS_RIGHT ? 15 - i : i;   // flip the order if sensor 0 is on the right
        digitalWrite(IR_S0_PIN, channel & 1);
        digitalWrite(IR_S1_PIN, (channel >> 1) & 1);
        digitalWrite(IR_S2_PIN, (channel >> 2) & 1);
        digitalWrite(IR_S3_PIN, (channel >> 3) & 1);
        delayMicroseconds(10);                // give the multiplexer a moment
        Serial.print(analogRead(IR_SIGNAL_PIN)); Serial.print("  ");
      }
      Serial.println();
      delay(200);
    }
    Serial.println("IR WORKING if the numbers change when you move the array over white and black");
  }

  if (key == 'g') {                           // IMU CHECK: 10 readings
    if (imuAddress == 0) Serial.println("IMU NOT FOUND - check SDA, SCL, power, GND");
    else {
      for (int reading = 0; reading < 10; reading++) {
        Wire.beginTransmission(imuAddress); Wire.write(0x22); Wire.endTransmission(false);   // gyro numbers start at 0x22
        Wire.requestFrom(imuAddress, (byte)12);   // 12 bytes: gyro X,Y,Z then accelerometer X,Y,Z
        byte data[12];
        for (int i = 0; i < 12; i++) data[i] = Wire.read();
        Serial.print("Gyro Z (deg/s): ");  Serial.print((int16_t)(data[5] << 8 | data[4]) * 0.0175);
        Serial.print("   Accel Z (g): ");  Serial.println((int16_t)(data[11] << 8 | data[10]) * 0.000061);
        delay(300);
      }
      Serial.println("IMU WORKING if the gyro changes when you turn the robot");
    }
  }

  if (key == 'e') {                           // ENCODER CHECK: 5 seconds
    Serial.println("Turn the wheels by hand for 5 seconds...");
    for (int i = 0; i < 10; i++) {
      Serial.print("Left ticks: ");  Serial.print(leftTicks);
      Serial.print("   Right ticks: ");  Serial.println(rightTicks);
      delay(500);
    }
    Serial.println("ENCODERS WORKING if the numbers changed");
  }

  if (key == 'm' || key == 'd') {             // MOTOR CHECK (m) or DRIFT CHECK (d)
    int firstStep = 0, lastStep = 3;          // m = steps 0 to 3
    if (key == 'd') { firstStep = 4; lastStep = 4; }   // d = only step 4 (both motors)
    Serial.println("LIFT THE ROBOT for m, put it on the floor for d. Starting in 3 seconds...");
    delay(3000);
    for (int step = firstStep; step <= lastStep; step++) {
      Serial.println(STEP_NAME[step]);        // say what we are testing
      leftTicks = 0; rightTicks = 0;          // count from zero
      int leftSpeed = LEFT_TEST[step] * LEFT_MOTOR_DIRECTION;      // flip if the wheel is wired backwards
      int rightSpeed = RIGHT_TEST[step] * RIGHT_MOTOR_DIRECTION;
      digitalWrite(LEFT_MOTOR_IN1, leftSpeed >= 0);   // forward: IN1 high, IN2 low
      digitalWrite(LEFT_MOTOR_IN2, leftSpeed <= 0);   // backward: IN1 low, IN2 high
      ledcWrite(LEFT_MOTOR_PWM, abs(leftSpeed));      // left motor speed
      digitalWrite(RIGHT_MOTOR_IN1, rightSpeed >= 0);
      digitalWrite(RIGHT_MOTOR_IN2, rightSpeed <= 0);
      ledcWrite(RIGHT_MOTOR_PWM, abs(rightSpeed));    // right motor speed
      delay(STEP_TIME[step]);                 // let the motors run
      ledcWrite(LEFT_MOTOR_PWM, 0);           // stop both motors
      ledcWrite(RIGHT_MOTOR_PWM, 0);
      Serial.print("  ticks -> left: "); Serial.print(leftTicks);
      Serial.print("  right: "); Serial.println(rightTicks);
      delay(500);
    }
    if (key == 'd') Serial.println("A big difference between left and right ticks means the robot curves");
    else Serial.println("MOTORS done - did each wheel move the right way?");
  }
}
