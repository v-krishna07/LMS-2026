// ============================================================================
//  pins.h  -  ONE PLACE FOR EVERY CONNECTION (WIRE) ON THE ROBOT
// ============================================================================
//  What is this file?
//    Every wire that goes from the ESP32 to another part has a NAME here.
//    All the .ino programs include this file. So if a wire is moved, you
//    change the number HERE (only once) and every program is updated.
//
//  How to fill it:
//    1. Next to each name write the ESP32 GPIO number (IO25 -> write 25).
//    2. When ALL numbers are filled, change  PINS_FILLED  from false to true.
//       (While it is false the programs refuse to compile and show a message,
//        so nobody runs them with empty pins by mistake.)
//    -1 means "not filled yet" or "this wire does not exist".
// ============================================================================
#pragma once

#define PINS_FILLED false        // <- change to true after filling every pin below


// ============================================================================
//  1. IR LINE SENSOR ARRAY (ARC-16)
//     It has 16 sensors but only ONE signal wire. A "multiplexer" chip picks
//     which sensor is connected to that wire. We choose the sensor by writing
//     a 4-bit binary number on the 4 select wires.
//     (0000 = sensor 0, 0001 = sensor 1 ... 1111 = sensor 15)
// ============================================================================
const int IR_S0_PIN = -1;        // select wire, bit 0 (S0 on the array)
const int IR_S1_PIN = -1;        // select wire, bit 1 (S1 on the array)
const int IR_S2_PIN = -1;        // select wire, bit 2 (S2 on the array)
const int IR_S3_PIN = -1;        // select wire, bit 3 (S3 on the array)
const int IR_SIGNAL_PIN = -1;    // analog wire: voltage of the chosen sensor (SIG on the array)
                                 // MUST be 32, 33, 34, 35, 36 or 39. Only these pins can read
                                 // analog voltage while WiFi is on. (GPIO 17 can NOT read analog!)
                                 // Remember: this wire goes through the resistor divider.
// The EN (enable) wire of the array is connected straight to GND on the board, so it has no pin here.

const bool IR_CHANNEL0_IS_RIGHT = true;   // the ARC-16 manual says sensor 0 is the RIGHT-most one.
                                          // Our programs flip it so sensor 0 = LEFT-most.
                                          // (set false only if your array is wired the other way)
const bool WHITE_LINE_IS_HIGH = true;     // true  = the number gets BIGGER over the white line
                                          // false = the number gets SMALLER over the white line
                                          // (run ir_sensor_check to see which one you have)

// Where each sensor sits, from the LEFT-most sensor to the RIGHT-most sensor.
// 0 = exact middle of the array. Negative = left, positive = right.
// The line position (error) is worked out from these numbers.
const float IR_POSITION[16] = {-7.5, -6.5, -5.5, -4.5, -3.5, -2.5, -1.5, -0.5,
                                0.5,  1.5,  2.5,  3.5,  4.5,  5.5,  6.5,  7.5};


// ============================================================================
//  2. MOTOR DRIVER (TB6612FNG) AND THE TWO MOTORS
//     Each motor has 2 direction wires (IN1, IN2) and 1 speed wire (PWM).
//     IN1 high + IN2 low  = forward
//     IN1 low  + IN2 high = backward
//     PWM is a number 0-255, bigger = faster
//     "Left" = motor on the A side of the driver, "Right" = motor on the B side.
//     If you find they are swapped on the robot, swap the numbers here.
// ============================================================================
const int LEFT_MOTOR_IN1 = -1;   // LEFT motor direction wire 1 (AIN1 on the driver)
const int LEFT_MOTOR_IN2 = -1;   // LEFT motor direction wire 2 (AIN2 on the driver)
const int LEFT_MOTOR_PWM = -1;   // LEFT motor speed wire       (PWMA on the driver)

const int RIGHT_MOTOR_IN1 = -1;  // RIGHT motor direction wire 1 (BIN1 on the driver)
const int RIGHT_MOTOR_IN2 = -1;  // RIGHT motor direction wire 2 (BIN2 on the driver)
const int RIGHT_MOTOR_PWM = -1;  // RIGHT motor speed wire       (PWMB on the driver)

const int MOTOR_STANDBY_PIN = -1;// STBY wire of the driver (the driver only works when it is HIGH).
                                 // Leave -1 if STBY is wired straight to 3.3V on the board.

const int LEFT_MOTOR_DIRECTION = 1;   // set to -1 if the LEFT wheel spins backward when it should go forward
const int RIGHT_MOTOR_DIRECTION = 1;  // set to -1 if the RIGHT wheel spins backward when it should go forward

const int MAX_PWM = 255;         // biggest power we allow (0-255).
                                 // If the battery voltage is higher than the motor voltage, lower it:
                                 // MAX_PWM = 255 x motor voltage / battery voltage


// ============================================================================
//  3. ENCODERS (the small sensors on the back of each motor that count how much the wheel turned)
//     Each encoder has 2 wires (A and B). Counting the A pulses tells how much
//     the wheel turned. Looking at B at that moment tells the direction.
// ============================================================================
const int LEFT_ENCODER_A = -1;   // LEFT encoder wire A
const int LEFT_ENCODER_B = -1;   // LEFT encoder wire B
const int RIGHT_ENCODER_A = -1;  // RIGHT encoder wire A
const int RIGHT_ENCODER_B = -1;  // RIGHT encoder wire B

const int LEFT_ENCODER_DIRECTION = 1;   // set -1 if the LEFT count goes DOWN when the wheel goes forward
const int RIGHT_ENCODER_DIRECTION = 1;  // set -1 if the RIGHT count goes DOWN when the wheel goes forward


// ============================================================================
//  4. IMU (LSM6DSO) - the chip that knows how fast the robot is turning (gyro)
//     It talks to the ESP32 with only 2 wires (called I2C).
// ============================================================================
const int IMU_SDA_PIN = -1;      // I2C data wire  (SDA)
const int IMU_SCL_PIN = -1;      // I2C clock wire (SCL)

const int IMU_DIRECTION = 1;     // set to -1 if the heading number goes DOWN when you turn the robot RIGHT


// ============================================================================
//  5. LED AND BUTTON
// ============================================================================
const int LED_PIN = -1;          // the red LED (it shows "I reached the end")
const int BUTTON_PIN = -1;       // push button. The other leg of the button goes to GND.
const int BUTTON_PRESSED = LOW;  // the pin reads LOW when the button is pressed
                                 // (we use the ESP32's built-in pull-up resistor)
