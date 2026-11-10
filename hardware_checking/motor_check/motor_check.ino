// MOTOR CHECK - spins each motor forward and backward, one after the other.
// !! LIFT THE ROBOT so the wheels are in the air. !!
// What to look for:
//   * "LEFT forward" must spin the LEFT wheel forward. If it spins backward,
//     set LEFT_MOTOR_DIRECTION = -1 in pins.h (same for the right wheel).
//   * If the wrong wheel moves, the left and right wires are swapped in pins.h.
#include "../../pins.h"                       // all the pin numbers live in this file
#if !PINS_FILLED
  #error "Open pins.h, fill in the pin numbers, then set PINS_FILLED to true"
#endif

// The 6 steps of the test. Each step gives a power (-255 to 255) to the left and right motor.
const char* STEP_NAME[6] = {"LEFT forward", "LEFT backward", "RIGHT forward", "RIGHT backward", "BOTH forward", "BOTH backward"};
const int LEFT_TEST[6]  = {120, -120, 0, 0, 120, -120};
const int RIGHT_TEST[6] = {0, 0, 120, -120, 120, -120};

void setup() {
  Serial.begin(115200);                       // start the Serial Monitor
  pinMode(LEFT_MOTOR_IN1, OUTPUT);            // direction wires send power out
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);
  if (MOTOR_STANDBY_PIN >= 0) {               // if the driver's STBY wire goes to the ESP32
    pinMode(MOTOR_STANDBY_PIN, OUTPUT);
    digitalWrite(MOTOR_STANDBY_PIN, HIGH);    // HIGH = wake the driver up
  }
  ledcAttach(LEFT_MOTOR_PWM, 20000, 8);       // speed wire: 20000 pulses per second, 8 bit (0 to 255)
  ledcAttach(RIGHT_MOTOR_PWM, 20000, 8);
  Serial.println("Motor check: LIFT THE ROBOT. Starting in 3 seconds...");
  delay(3000);                                // time to lift the robot
}

void loop() {
  for (int step = 0; step < 6; step++) {      // go through the 6 steps
    Serial.println(STEP_NAME[step]);          // say what we are testing
    int leftSpeed = LEFT_TEST[step] * LEFT_MOTOR_DIRECTION;      // flip if the wheel is wired backwards
    int rightSpeed = RIGHT_TEST[step] * RIGHT_MOTOR_DIRECTION;
    digitalWrite(LEFT_MOTOR_IN1, leftSpeed >= 0);   // forward: IN1 high, IN2 low
    digitalWrite(LEFT_MOTOR_IN2, leftSpeed <= 0);   // backward: IN1 low, IN2 high (both high = brake)
    ledcWrite(LEFT_MOTOR_PWM, abs(leftSpeed));      // speed of the left motor
    digitalWrite(RIGHT_MOTOR_IN1, rightSpeed >= 0);
    digitalWrite(RIGHT_MOTOR_IN2, rightSpeed <= 0);
    ledcWrite(RIGHT_MOTOR_PWM, abs(rightSpeed));    // speed of the right motor
    delay(1500);                              // let the motor run for 1.5 seconds
    ledcWrite(LEFT_MOTOR_PWM, 0);             // stop both motors
    ledcWrite(RIGHT_MOTOR_PWM, 0);
    delay(500);                               // short rest
  }
  Serial.println("--- done, repeating in 5 seconds ---");
  delay(5000);
}
