// IR SENSOR CHECK - prints the 16 sensor numbers (left sensor first, right sensor last).
// What to look for:
//   * Put the array over BLACK floor: all numbers should be similar.
//   * Slide it over the WHITE line: the numbers under the line should clearly change.
//   * Note if they go UP or DOWN over white, and set WHITE_LINE_IS_HIGH in pins.h.
#include "../../pins.h"                       // all the pin numbers live in this file
#if !PINS_FILLED
  #error "Open pins.h, fill in the pin numbers, then set PINS_FILLED to true"
#endif

void setup() {
  Serial.begin(115200);                       // start the Serial Monitor
  pinMode(IR_S0_PIN, OUTPUT);                 // the 4 select wires send the sensor number to the array
  pinMode(IR_S1_PIN, OUTPUT);
  pinMode(IR_S2_PIN, OUTPUT);
  pinMode(IR_S3_PIN, OUTPUT);
  analogReadResolution(12);                   // analog readings go from 0 to 4095
  analogSetPinAttenuation(IR_SIGNAL_PIN, ADC_11db);   // lets the ESP32 read up to about 3.3 volts
  Serial.println("IR sensor check: 16 numbers, left to right");
}

void loop() {
  for (int i = 0; i < 16; i++) {              // i = sensor number, 0 = left-most
    int channel = IR_CHANNEL0_IS_RIGHT ? 15 - i : i;   // flip the order if sensor 0 is on the right
    digitalWrite(IR_S0_PIN, channel & 1);     // bit 0 of the channel number
    digitalWrite(IR_S1_PIN, (channel >> 1) & 1);   // bit 1
    digitalWrite(IR_S2_PIN, (channel >> 2) & 1);   // bit 2
    digitalWrite(IR_S3_PIN, (channel >> 3) & 1);   // bit 3
    delayMicroseconds(10);                    // give the multiplexer chip a moment to switch
    Serial.print(analogRead(IR_SIGNAL_PIN));  // read the chosen sensor and print it
    Serial.print("  ");                       // space between the numbers
  }
  Serial.println();                           // new line after all 16 numbers
  delay(200);                                 // 5 readings per second
}
