// PID ENCODER - drives STRAIGHT using ONLY the wheel encoders (no line, no gyro).
// How it works, in simple words:
//   Both wheels get the same power, but one motor is always a bit stronger, so the robot curves.
//   The encoders count how much each wheel turned.
//   error = left count - right count. If the left wheel is ahead, slow it down and speed up the right one.
// How to start: put the robot on the floor, press the button. Press the button again to stop.
#include "../../pins.h"                       // all the pin numbers live in this file
#if !PINS_FILLED
  #error "Open pins.h, fill in the pin numbers, then set PINS_FILLED to true"
#endif

// ---------- numbers you can change (tune these) ----------
float KP = 1.0;           // P = how hard to fix the difference between the two wheels
float KI = 0.0;           // I = fixes a small constant difference (keep 0 at first)
float KD = 0.3;           // D = calms the correction
int SPEED = 120;          // power of both wheels (0-255)

// ---------- what the robot is doing right now (the "mode") ----------
const int WAIT = 0;       // waiting for the button
const int RUN = 1;        // driving straight
int mode = WAIT;          // we start by waiting

// ---------- variables the program uses ----------
volatile long leftTicks = 0;     // wheel counts (changed by the interrupts)
volatile long rightTicks = 0;
float error = 0;          // left count - right count
float lastError = 0;      // the error in the previous loop
float integral = 0;       // the errors added up over time
unsigned long lastPrintTime = 0;   // when we last printed

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
  pinMode(BUTTON_PIN, INPUT_PULLUP);          // button uses the built-in pull-up
  Serial.println("Put the robot on the floor and press the button (press again to stop)");
}

void loop() {
  int leftSpeed = 0;                          // power for the left motor (0 = stopped)
  int rightSpeed = 0;                         // power for the right motor

  if (mode == WAIT) {                         // --- waiting for the button ---
    if (digitalRead(BUTTON_PIN) == BUTTON_PRESSED) {   // button pressed
      delay(1000);                            // time to take your hand away
      leftTicks = 0;                          // start counting from zero
      rightTicks = 0;
      integral = 0;                           // forget old PID memory
      lastError = 0;
      mode = RUN;                             // start driving
    }
  }
  else if (mode == RUN) {                     // --- driving straight ---
    error = leftTicks - rightTicks;           // + means the LEFT wheel is ahead (robot curves right)
    integral = constrain(integral + error, -2000, 2000);   // I part: errors added up (limited)
    float correction = KP * error + KI * integral + KD * (error - lastError);   // the PID formula
    lastError = error;                        // remember for the next loop
    leftSpeed = SPEED - correction;           // slow the wheel that is ahead
    rightSpeed = SPEED + correction;          // speed up the other one
    if (millis() - lastPrintTime > 100) {     // print 10 times per second
      lastPrintTime = millis();
      Serial.print("left "); Serial.print(leftTicks);
      Serial.print("  right "); Serial.print(rightTicks);
      Serial.print("  error "); Serial.println(error);
    }
    if (digitalRead(BUTTON_PIN) == BUTTON_PRESSED) {   // button pressed again = stop
      delay(500);                             // avoid reading the same press twice
      Serial.println("Stopped. Press the button to run again.");
      mode = WAIT;
    }
  }

  // ===== SEND THE POWER TO THE MOTORS =====
  leftSpeed = constrain(leftSpeed * LEFT_MOTOR_DIRECTION, -MAX_PWM, MAX_PWM);      // flip if wired backwards, never above the limit
  rightSpeed = constrain(rightSpeed * RIGHT_MOTOR_DIRECTION, -MAX_PWM, MAX_PWM);
  digitalWrite(LEFT_MOTOR_IN1, leftSpeed >= 0);    // forward: IN1 high, IN2 low
  digitalWrite(LEFT_MOTOR_IN2, leftSpeed <= 0);    // backward: IN1 low, IN2 high (both high = brake)
  ledcWrite(LEFT_MOTOR_PWM, abs(leftSpeed));       // speed of the left motor
  digitalWrite(RIGHT_MOTOR_IN1, rightSpeed >= 0);
  digitalWrite(RIGHT_MOTOR_IN2, rightSpeed <= 0);
  ledcWrite(RIGHT_MOTOR_PWM, abs(rightSpeed));     // speed of the right motor
  delay(10);                                       // tiny wait, then repeat
}
