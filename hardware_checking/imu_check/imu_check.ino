// IMU CHECK - the IMU has a gyro (how fast we turn) and an accelerometer (which way gravity pulls).
// What to look for:
//   * Robot still: gyro numbers near 0, one accelerometer number near 1.0 (gravity).
//   * Turn the robot flat on the table: the gyro Z number changes a lot.
#include "../../pins.h"                       // all the pin numbers live in this file
#if !PINS_FILLED
  #error "Open pins.h, fill in the pin numbers, then set PINS_FILLED to true"
#endif
#include <Wire.h>                             // library for the 2-wire (I2C) connection

byte imuAddress = 0;                          // the IMU's address on the 2 wires (found in setup)

void setup() {
  Serial.begin(115200);                       // start the Serial Monitor
  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);       // start the 2-wire connection on our pins
  for (byte address = 0x6A; address <= 0x6B; address++) {   // the IMU lives at address 0x6A or 0x6B
    Wire.beginTransmission(address);          // talk to this address
    Wire.write(0x0F);                         // register 0x0F is the "who am I" register
    Wire.endTransmission(false);              // finish asking, but keep the line
    Wire.requestFrom(address, (byte)1);       // ask for 1 byte
    if (Wire.read() == 0x6C) imuAddress = address;   // the LSM6DSO always answers 0x6C
  }
  if (imuAddress == 0) {                      // nobody answered
    Serial.println("IMU NOT FOUND - check SDA, SCL, power and GND wires");
    while (true) delay(1000);                 // stop here
  }
  Serial.print("IMU FOUND at address 0x");    // good news
  Serial.println(imuAddress, HEX);
  Wire.beginTransmission(imuAddress); Wire.write(0x10); Wire.write(0x74); Wire.endTransmission();   // turn the accelerometer on
  Wire.beginTransmission(imuAddress); Wire.write(0x11); Wire.write(0x74); Wire.endTransmission();   // turn the gyro on
}

void loop() {
  Wire.beginTransmission(imuAddress);         // talk to the IMU
  Wire.write(0x22);                           // 0x22 is where the gyro numbers start
  Wire.endTransmission(false);                // finish asking, keep the line
  Wire.requestFrom(imuAddress, (byte)12);     // read 12 bytes: gyro X,Y,Z then accelerometer X,Y,Z
  byte data[12];                              // place to keep the 12 bytes
  for (int i = 0; i < 12; i++) data[i] = Wire.read();   // read them one by one

  float gyroX = (int16_t)(data[1] << 8 | data[0]) * 0.0175;       // each number is 2 bytes; 0.0175 turns it into degrees per second
  float gyroY = (int16_t)(data[3] << 8 | data[2]) * 0.0175;
  float gyroZ = (int16_t)(data[5] << 8 | data[4]) * 0.0175;       // Z = turning left/right when the robot is flat
  float accelX = (int16_t)(data[7] << 8 | data[6]) * 0.000061;    // 0.000061 turns it into g (1.0 = gravity)
  float accelY = (int16_t)(data[9] << 8 | data[8]) * 0.000061;
  float accelZ = (int16_t)(data[11] << 8 | data[10]) * 0.000061;

  Serial.print("Gyro X Y Z: ");                // print the gyro numbers
  Serial.print(gyroX); Serial.print("  "); Serial.print(gyroY); Serial.print("  "); Serial.print(gyroZ);
  Serial.print("   | Accel X Y Z: ");          // print the accelerometer numbers
  Serial.print(accelX); Serial.print("  "); Serial.print(accelY); Serial.print("  "); Serial.println(accelZ);
  delay(200);                                 // 5 readings per second
}
