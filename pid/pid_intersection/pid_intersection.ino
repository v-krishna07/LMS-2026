// PID INTERSECTION - follows the line with PID and also HANDLES junctions (T, +, corners, dead end, end box).
// Uses only the IR array and the motors. It prints what kind of junction it found.
// How it works, in simple words:
//   * Normal line: PID steering (same as pid_normal).
//   * A far-left or far-right sensor sees white = a junction or a corner.
//     The robot creeps forward a little (so the wheels sit over the junction), then looks ahead.
//   * It then picks a way: LEFT first, then STRAIGHT, then RIGHT (the "left-hand rule").
//   * Nothing under the sensors = dead end, so it turns around.
//   * Almost all sensors on white after creeping = the big white END box. The LED turns on and the robot stops.
// How to start: the LED is on for 4 seconds = slide the array over the line and the black floor. Then press the button.
#include "../../pins.h"                       // all the pin numbers live in this file
#if !PINS_FILLED
  #error "Open pins.h, fill in the pin numbers, then set PINS_FILLED to true"
#endif

// ---------- numbers you can change (tune these) ----------
float KP = 12.0;          // P = steer harder when the line is far from the middle
float KI = 0.0;           // I = fixes a small constant sideways drift (keep 0 at first)
float KD = 30.0;          // D = calms the wobble
int SPEED = 120;          // power on a straight line (0-255)
int CREEP_SPEED = 90;     // power while creeping over a junction
int CREEP_TIME = 150;     // how long to creep (milliseconds). Tune it so the wheels stop in the middle of the junction
int TURN_SPEED = 100;     // power while turning on the spot
int END_COUNT = 14;       // this many sensors on white = the END box

// ---------- what the robot is doing right now (the "mode") ----------
const int CALIBRATE = 0;  // learning what white and black look like
const int WAIT = 1;       // waiting for the button
const int FOLLOW = 2;     // following the line
const int CREEP = 3;      // creeping forward over a junction
const int TURN = 4;       // turning on the spot until a line is found
const int FINISHED = 5;   // reached the end box
int mode = CALIBRATE;     // we start with calibrating

// ---------- variables the program uses ----------
int sensorMin[16];        // smallest reading of each sensor (black floor)
int sensorMax[16];        // biggest reading of each sensor (white line)
int sensorValue[16];      // each sensor as a number 0-1000 (1000 = sees the line)
float error = 0;          // where the line is: 0 = middle, negative = left, positive = right
float lastError = 0;      // the error in the previous loop
float integral = 0;       // the errors added up over time
bool sawLeft = false;     // did we see a line going left at this junction?
bool sawRight = false;    // did we see a line going right at this junction?
int turnDirection = 1;    // -1 = turn left, +1 = turn right
int turnStage = 0;        // 0 = still leaving the old line, 1 = looking for the new line
unsigned long startTime = 0;          // when the calibration started
unsigned long modeStartTime = 0;      // when the current creep or turn started
unsigned long lastJunctionTime = 0;   // when we last handled a junction

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
  pinMode(LED_PIN, OUTPUT);                   // LED sends power out
  pinMode(BUTTON_PIN, INPUT_PULLUP);          // button uses the built-in pull-up
  for (int i = 0; i < 16; i++) {              // start values for the calibration
    sensorMin[i] = 4095;
    sensorMax[i] = 0;
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
  bool leftEdge = false;                      // true if one of the 2 far-left sensors sees the line
  bool rightEdge = false;                     // true if one of the 2 far-right sensors sees the line
  bool middle = false;                        // true if one of the 2 middle sensors sees the line
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
    if (value > 500) {                        // this sensor sees the line
      sensorsOnLine++;
      if (i <= 1) leftEdge = true;            // sensors 0 and 1 are the far-left ones
      if (i >= 14) rightEdge = true;          // sensors 14 and 15 are the far-right ones
      if (i == 7 || i == 8) middle = true;    // sensors 7 and 8 are in the middle
    }
    weightedSum += value * IR_POSITION[i];    // strong sensors on the right push the average to the right
    totalSum += value;
  }

  // ===== 2. DECIDE WHAT TO DO =====
  int leftSpeed = 0;                          // power for the left motor (0 = stopped unless we change it)
  int rightSpeed = 0;                         // power for the right motor

  if (mode == CALIBRATE) {                    // --- calibrating ---
    if (millis() - startTime > 4000) {        // 4 seconds are over
      digitalWrite(LED_PIN, LOW);             // LED off
      Serial.println("Put the robot on the line and press the button");
      mode = WAIT;
    }
  }
  else if (mode == WAIT) {                    // --- waiting for the button ---
    if (digitalRead(BUTTON_PIN) == BUTTON_PRESSED) {   // button pressed
      delay(1000);                            // time to take your hand away
      mode = FOLLOW;
    }
  }
  else if (mode == FOLLOW) {                  // --- following the line ---
    if (millis() - lastJunctionTime > 300 && (leftEdge || rightEdge)) {   // a junction, a corner or the end box
      sawLeft = leftEdge;                     // remember what we see right now
      sawRight = rightEdge;
      modeStartTime = millis();               // start the creeping timer
      mode = CREEP;
    }
    else if (sensorsOnLine == 0) {            // no line at all = dead end
      Serial.println("Found: DEAD END -> turning around");
      turnDirection = 1;                      // turn around to the right
      turnStage = 0;                          // we are not on a line, so we only look for the next one
      modeStartTime = millis();
      mode = TURN;
    }
    else {                                    // normal line: follow it with PID
      error = weightedSum / totalSum;         // average position of the line (-7.5 to +7.5)
      integral = constrain(integral + error, -100, 100);   // I part: errors added up (limited)
      float correction = KP * error + KI * integral + KD * (error - lastError);   // the PID formula
      lastError = error;                      // remember for the next loop
      leftSpeed = SPEED + correction;         // line on the right -> left wheel faster -> robot turns right
      rightSpeed = SPEED - correction;
    }
  }
  else if (mode == CREEP) {                   // --- creeping forward over the junction ---
    leftSpeed = CREEP_SPEED;                  // go straight, slowly
    rightSpeed = CREEP_SPEED;
    if (leftEdge) sawLeft = true;             // keep looking for side lines while creeping
    if (rightEdge) sawRight = true;
    if (millis() - modeStartTime > CREEP_TIME) {   // creeping is finished, the wheels are over the junction
      leftSpeed = 0;                          // stop
      rightSpeed = 0;
      if (sensorsOnLine >= END_COUNT) {       // almost every sensor sees white = the end box
        Serial.println("END BOX reached!");
        digitalWrite(LED_PIN, HIGH);          // the LED says "I am at the end"
        mode = FINISHED;
      }
      else {                                  // not the end: work out what kind of junction it is
        bool sawAhead = middle;               // the middle sensors see a line AHEAD of us
        if (sawLeft && sawRight && sawAhead) Serial.println("Found: PLUS (+) -> turning LEFT");
        else if (sawLeft && sawRight) Serial.println("Found: T junction -> turning LEFT");
        else if (sawLeft && sawAhead) Serial.println("Found: line goes STRAIGHT and LEFT -> turning LEFT");
        else if (sawRight && sawAhead) Serial.println("Found: line goes STRAIGHT and RIGHT -> going STRAIGHT");
        else if (sawLeft) Serial.println("Found: LEFT corner -> turning LEFT");
        else Serial.println("Found: RIGHT corner -> turning RIGHT");
        if (sawLeft) {                        // left first
          turnDirection = -1; turnStage = 0; modeStartTime = millis(); mode = TURN;
        }
        else if (sawAhead) {                  // then straight: just carry on following the line
          lastJunctionTime = millis(); integral = 0; lastError = 0; mode = FOLLOW;
        }
        else {                                // then right
          turnDirection = 1; turnStage = 0; modeStartTime = millis(); mode = TURN;
        }
      }
    }
  }
  else if (mode == TURN) {                    // --- turning on the spot until a line is found ---
    leftSpeed = turnDirection * TURN_SPEED;   // left turn: left wheel back, right wheel forward
    rightSpeed = -turnDirection * TURN_SPEED;
    if (turnStage == 0 && !middle) turnStage = 1;   // the old line has left the middle: now look for the next one
    if (turnStage == 1 && middle) {           // the next line is under the middle: turn finished
      leftSpeed = 0;
      rightSpeed = 0;
      lastJunctionTime = millis(); integral = 0; lastError = 0;
      mode = FOLLOW;
    }
    if (millis() - modeStartTime > 2500) {    // turning takes too long: give up and carry on
      leftSpeed = 0;
      rightSpeed = 0;
      lastJunctionTime = millis(); integral = 0; lastError = 0;
      mode = FOLLOW;
    }
  }
  // mode == FINISHED: do nothing, both speeds stay 0

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
