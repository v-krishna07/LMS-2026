// ENCODER CHECK - counts how much each wheel turns.
// What to look for: turn a wheel by hand. Its number must change.
// Turning FORWARD should make the number go UP. If it goes down, change
// LEFT_ENCODER_DIRECTION / RIGHT_ENCODER_DIRECTION in pins.h.
#include "../../pins.h"                       // all the pin numbers live in this file
#if !PINS_FILLED
  #error "Open pins.h, fill in the pin numbers, then set PINS_FILLED to true"
#endif

volatile long leftTicks = 0;                  // wheel counts. "volatile" because the interrupts change them behind our back
volatile long rightTicks = 0;

// This small function runs BY ITSELF every time the left encoder sends a pulse (the ESP32 needs it to be a function).
void IRAM_ATTR leftEncoderISR() {
  if (digitalRead(LEFT_ENCODER_B) == HIGH) leftTicks -= LEFT_ENCODER_DIRECTION;   // wire B tells the direction
  else leftTicks += LEFT_ENCODER_DIRECTION;
}
// Same for the right encoder.
void IRAM_ATTR rightEncoderISR() {
  if (digitalRead(RIGHT_ENCODER_B) == HIGH) rightTicks -= RIGHT_ENCODER_DIRECTION;
  else rightTicks += RIGHT_ENCODER_DIRECTION;
}

void setup() {
  Serial.begin(115200);                       // start the Serial Monitor
  pinMode(LEFT_ENCODER_A, INPUT);             // the 4 encoder wires only listen
  pinMode(LEFT_ENCODER_B, INPUT);
  pinMode(RIGHT_ENCODER_A, INPUT);
  pinMode(RIGHT_ENCODER_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), leftEncoderISR, RISING);     // run the function on every pulse
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), rightEncoderISR, RISING);
  Serial.println("Encoder check: turn each wheel by hand");
}

void loop() {
  Serial.print("Left ticks: ");  Serial.print(leftTicks);          // show the left count
  Serial.print("    Right ticks: "); Serial.println(rightTicks);    // show the right count
  delay(200);                                 // 5 times per second
}
