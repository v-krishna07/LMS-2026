// BUTTON CHECK - tells you when the push button is pressed.
// (The main power switch cannot be tested by code. If the robot turns on, that switch works.)
#include "../../pins.h"                       // all the pin numbers live in this file
#if !PINS_FILLED
  #error "Open pins.h, fill in the pin numbers, then set PINS_FILLED to true"
#endif

bool wasPressed = false;                      // remembers what the button was doing a moment ago

void setup() {
  Serial.begin(115200);                       // start the Serial Monitor
  pinMode(BUTTON_PIN, INPUT_PULLUP);          // built-in pull-up: pin reads HIGH until the button connects it to GND
  Serial.println("Button check: press and release the button");   // tell the user what to do
}

void loop() {
  bool isPressed = (digitalRead(BUTTON_PIN) == BUTTON_PRESSED);   // true while the button is held down
  if (isPressed != wasPressed) {              // only talk when something changes
    if (isPressed) Serial.println("Button PRESSED  - working!");  // button just went down
    else Serial.println("Button released");   // button just came up
    wasPressed = isPressed;                   // remember the new state
  }
  delay(20);                                  // small wait so the button does not bounce
}
