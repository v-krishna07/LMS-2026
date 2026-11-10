// ESP32 CHECK - prints facts about the ESP32 chip.
// If you can read this text, the ESP32, the USB cable and the Serial Monitor all work.

void setup() {
  Serial.begin(115200);                                       // start the Serial Monitor
  delay(500);                                                 // wait a moment for the screen to be ready
  Serial.println("ESP32 is alive!");                          // proof that the chip is running our program
  Serial.print("Chip name: ");           Serial.println(ESP.getChipModel());      // for example ESP32-D0WD
  Serial.print("Chip version: ");        Serial.println(ESP.getChipRevision());   // chip revision number
  Serial.print("Number of cores: ");     Serial.println(ESP.getChipCores());      // the ESP32 has 2 brains (cores)
  Serial.print("Speed (MHz): ");         Serial.println(ESP.getCpuFreqMHz());     // usually 240
  Serial.print("Flash size (bytes): ");  Serial.println(ESP.getFlashChipSize());  // where the program is stored
}

void loop() {
  Serial.print("Running for (seconds): ");   Serial.print(millis() / 1000);       // time since power on
  Serial.print("   free memory (bytes): ");  Serial.println(ESP.getFreeHeap());   // memory that is still free
  delay(1000);                                                // once every second
}
