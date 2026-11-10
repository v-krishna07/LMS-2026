// PID NORMAL - follows the white line using ONLY the IR sensor array (no encoders, no gyro).
// How it works, in simple words:
//   1. Find where the line is under the array (left side, middle or right side). This number is the "error".
//   2. error = 0 means the line is exactly in the middle.
//   3. If the line is on the right, speed up the left wheel and slow the right wheel (and the other way round).
//      The PID formula decides how strongly to do that.
// How to start: the LED turns on for 4 seconds = slide the array over the line AND the black floor
// (this teaches the sensors what white and black look like). Then put the robot on the line and press the button.
#include "../../pins.h"                       // all the pin numbers live in this file
#if !PINS_FILLED
  #error "Open pins.h, fill in the pin numbers, then set PINS_FILLED to true"
#endif

// ---------- numbers you can change (tune these) ----------
float KP = 12.0;          // P = steer harder when the line is far from the middle (too big = wobbles)
float KI = 0.0;           // I = fixes a small constant sideways drift (keep 0 at first)
float KD = 30.0;          // D = calms the wobble (too big = twitchy)
int SPEED = 120;          // power of both wheels on a straight line (0-255)

// ---------- what the robot is doing right now (the "mode") ----------
const int CALIBRATE = 0;  // learning what white and black look like
const int WAIT = 1;       // waiting for the button
const int FOLLOW = 2;     // following the line
int mode = CALIBRATE;     // we start with calibrating

// ---------- variables the program uses ----------
int sensorMin[16];        // smallest reading of each sensor (black floor)
int sensorMax[16];        // biggest reading of each sensor (white line)
int sensorValue[16];      // each sensor as a number 0-1000 (1000 = this sensor sees the line)
float error = 0;          // where the line is: 0 = middle, negative = left, positive = right
float lastError = 0;      // the error in the previous loop (used by the D part)
float integral = 0;       // the errors added up over time (used by the I part)
unsigned long startTime = 0;       // when the calibration started
unsigned long lastPrintTime = 0;   // when we last printed

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
  ledcAttach(LEFT_MOTOR_PWM, 20000, 8);       // motor speed wires: 20000 pulses per second, 8 bit (0-255)
  ledcAttach(RIGHT_MOTOR_PWM, 20000, 8);
  pinMode(LED_PIN, OUTPUT);                   // LED sends power out
  pinMode(BUTTON_PIN, INPUT_PULLUP);          // button uses the built-in pull-up
  for (int i = 0; i < 16; i++) {              // start values for the calibration
    sensorMin[i] = 4095;                      // biggest possible, so any reading is smaller
    sensorMax[i] = 0;                         // smallest possible, so any reading is bigger
  }
  Serial.println("Calibrating for 4 seconds: slide the array over the line and the black floor");
  digitalWrite(LED_PIN, HIGH);                // LED on while calibrating
  startTime = millis();                       // remember when we started
}

void loop() {
  // ===== 1. READ THE 16 SENSORS =====
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
    int value = (raw - sensorMin[i]) * 1000L / max(sensorMax[i] - sensorMin[i], 1);   // squeeze the reading into 0-1000
    value = constrain(value, 0, 1000);        // keep it inside 0-1000
    if (!WHITE_LINE_IS_HIGH) value = 1000 - value;   // flip it if white gives a smaller number
    sensorValue[i] = value;                   // save it
    if (value > 500) sensorsOnLine++;         // this sensor sees the line
    weightedSum += value * IR_POSITION[i];    // strong sensors on the right push the average to the right
    totalSum += value;
  }

  // ===== 2. DECIDE WHAT TO DO =====
  int leftSpeed = 0;                          // power for the left motor (0 = stopped unless we change it)
  int rightSpeed = 0;                         // power for the right motor
  float correction = 0;                       // how much to steer (used for printing too)

  if (mode == CALIBRATE) {                    // --- calibrating ---
    if (millis() - startTime > 4000) {        // 4 seconds are over
      digitalWrite(LED_PIN, LOW);             // LED off
      Serial.println("Put the robot on the line and press the button");
      mode = WAIT;                            // next: wait for the button
    }
  }
  else if (mode == WAIT) {                    // --- waiting for the button ---
    if (digitalRead(BUTTON_PIN) == BUTTON_PRESSED) {   // button pressed
      delay(1000);                            // time to take your hand away
      mode = FOLLOW;                          // start following the line
    }
  }
  else if (mode == FOLLOW) {                  // --- following the line ---
    if (sensorsOnLine > 0) {                  // we can see the line
      error = weightedSum / totalSum;         // average position of the line (-7.5 to +7.5)
      integral = constrain(integral + error, -100, 100);   // I part: errors added up (limited)
      correction = KP * error + KI * integral + KD * (error - lastError);   // the PID formula
      lastError = error;                      // remember for the next loop
      leftSpeed = SPEED + correction;         // line on the right -> left wheel faster -> robot turns right
      rightSpeed = SPEED - correction;        // and the right wheel slower
    }                                         // if no line is seen, both speeds stay 0 = the robot stops
    if (millis() - lastPrintTime > 100) {     // print 10 times per second
      lastPrintTime = millis();
      Serial.print("error "); Serial.print(error);
      Serial.print("  correction "); Serial.print(correction);
      Serial.print("  left "); Serial.print(leftSpeed);
      Serial.print("  right "); Serial.println(rightSpeed);
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
  delay(2);                                        // tiny wait, then repeat
}
