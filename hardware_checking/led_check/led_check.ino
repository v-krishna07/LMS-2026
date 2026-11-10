// LED CHECK - blinks the red LED. If you see it blink, the LED and its wire are OK.
#include "../../pins.h"                       // all the pin numbers live in this file
#if !PINS_FILLED                              // stop with a message if pins.h is not filled yet
  #error "Open pins.h, fill in the pin numbers, then set PINS_FILLED to true"
#endif

void setup() {
  Serial.begin(115200);                       // start the Serial Monitor (set it to 115200)
  pinMode(LED_PIN, OUTPUT);                   // the LED pin sends power OUT
  Serial.println("LED check: the LED should blink every second");   // tell the user what to look for
}

void loop() {
  digitalWrite(LED_PIN, HIGH);                // HIGH = LED on
  Serial.println("LED ON  - can you see the light?");   // say it on the screen
  delay(1000);                                // wait 1 second
  digitalWrite(LED_PIN, LOW);                 // LOW = LED off
  Serial.println("LED OFF");                  // say it on the screen
  delay(1000);                                // wait 1 second
}
