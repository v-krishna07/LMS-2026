// PID WITH GRAPH (UDP over WiFi) - the simple PID line follower that sends its numbers to your phone / PC,
// so you can draw the error graph and TUNE the PID WHILE THE ROBOT IS DRIVING.
// Steering uses ONLY the IR array (like pid_normal). The gyro and encoders are only measured and sent as data.
//
// HOW TO CONNECT (UDP over WiFi):
//   1. Upload this program. The robot makes its own WiFi network called "LMS-Bot" (password 12345678).
//   2. Join that WiFi with your phone or PC. The robot's address is 192.168.4.1 and the UDP port is 4210.
//   3. PC: run plot_graph.py udp 192.168.4.1   (or type  nc -u 192.168.4.1 4210  in a terminal).
//      Phone: use any "UDP terminal" app, send to 192.168.4.1 port 4210.
//   4. Send a command (for example  kp 15  or  go). The first message you send tells the robot where to send the
//      DATA lines back to.
//
// Commands you can type (press Enter):
//   go            start following the line (same as the button)
//   stop          stop the robot
//   kp 15         change KP (also ki 0.1, kd 40)
//   speed 130     change the base power (0-255)
//   help          list the commands
//
// The DATA line (comma separated, 20 lines per second):
//   DATA,time_ms,mode,error,correction,leftPower,rightPower,heading,gyroSpeed,leftTicks,rightTicks,realSpeed,sensorsOnLine,ir0 ... ir15
//   error = where the line is (0 = middle, negative = left, positive = right) -> this is what you plot while tuning.
//
// How to start: keep the robot still for 1 second at power on (the gyro finds its zero), then the LED is on for
// 4 seconds = slide the array over the line and the black floor. Then send  go  or press the button.
// NOTE: WiFi and Bluetooth are NOT allowed in the competition arena. This program is for tuning only.
#include "../../pins.h"                       // all the pin numbers live in this file
#if !PINS_FILLED
  #error "Open pins.h, fill in the pin numbers, then set PINS_FILLED to true"
#endif
#include <Wire.h>                             // library for the IMU's 2-wire connection
#include <WiFi.h>                           // WiFi library
#include <WiFiUdp.h>                          // UDP = a fast way to send short text messages over WiFi

// ---------- numbers you can change (tune these, or change them live with commands) ----------
float KP = 12.0;          // P = steer harder when the line is far from the middle
float KI = 0.0;           // I = fixes a small constant sideways drift (keep 0 at first)
float KD = 30.0;          // D = calms the wobble
int SPEED = 120;          // power of both wheels on a straight line (0-255)

// ---------- what the robot is doing right now (the "mode") ----------
const int GYRO_ZERO = 0;  // keeping still while the gyro finds its zero
const int CALIBRATE = 1;  // learning what white and black look like
const int WAIT = 2;       // waiting for "go" or the button
const int FOLLOW = 3;     // following the line
int mode = GYRO_ZERO;     // we start with the gyro zero

// ---------- variables the program uses ----------
int sensorMin[16];        // smallest reading of each sensor (black floor)
int sensorMax[16];        // biggest reading of each sensor (white line)
int sensorValue[16];      // each sensor as a number 0-1000 (1000 = sees the line)
float error = 0;          // where the line is: 0 = middle, negative = left, positive = right
float lastError = 0;      // the error in the previous loop
float integral = 0;       // the errors added up over time
volatile long leftTicks = 0;      // wheel counts (changed by the interrupts)
volatile long rightTicks = 0;
long lastLeftTicks = 0;           // wheel counts at the end of the previous loop
long lastRightTicks = 0;
byte imuAddress = 0;              // IMU address (found in setup)
float gyroBias = 0;               // the gyro's "zero" value when the robot is still
float gyroTotal = 0;              // adds up the gyro readings while still
int gyroCount = 0;                // how many readings we added
float heading = 0;                // degrees turned so far (+ = right)
unsigned long lastMicros = 0;     // time of the last loop
unsigned long startTime = 0;      // when the current start-up step began
unsigned long lastSendTime = 0;   // when we last sent the data
WiFiUDP udp;                                  // the UDP connection
IPAddress clientIP;                           // address of the phone / PC that talks to us
unsigned int clientPort = 0;                  // its port number
bool clientKnown = false;                     // true after the first message arrived

void IRAM_ATTR leftEncoderISR() {             // runs by itself on every left encoder pulse
  if (digitalRead(LEFT_ENCODER_B) == HIGH) leftTicks -= LEFT_ENCODER_DIRECTION;   // wire B tells the direction
  else leftTicks += LEFT_ENCODER_DIRECTION;
}
void IRAM_ATTR rightEncoderISR() {            // runs by itself on every right encoder pulse
  if (digitalRead(RIGHT_ENCODER_B) == HIGH) rightTicks -= RIGHT_ENCODER_DIRECTION;
  else rightTicks += RIGHT_ENCODER_DIRECTION;
}

void setup() {
  Serial.begin(115200);                       // start the Serial Monitor (for the USB cable)
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
  pinMode(LEFT_ENCODER_A, INPUT);             // encoder wires only listen
  pinMode(LEFT_ENCODER_B, INPUT);
  pinMode(RIGHT_ENCODER_A, INPUT);
  pinMode(RIGHT_ENCODER_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), leftEncoderISR, RISING);     // run the function on every pulse
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), rightEncoderISR, RISING);
  pinMode(LED_PIN, OUTPUT);                   // LED sends power out
  pinMode(BUTTON_PIN, INPUT_PULLUP);          // button uses the built-in pull-up
  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);       // start the IMU connection
  for (byte address = 0x6A; address <= 0x6B; address++) {   // look for the IMU at 0x6A or 0x6B
    Wire.beginTransmission(address); Wire.write(0x0F); Wire.endTransmission(false);
    Wire.requestFrom(address, (byte)1);
    if (Wire.read() == 0x6C) imuAddress = address;          // the LSM6DSO answers 0x6C
  }
  if (imuAddress == 0) { Serial.println("IMU NOT FOUND"); while (true) delay(1000); }
  Wire.beginTransmission(imuAddress); Wire.write(0x11); Wire.write(0x74); Wire.endTransmission();   // turn the gyro on
  for (int i = 0; i < 16; i++) {              // start values for the calibration
    sensorMin[i] = 4095;
    sensorMax[i] = 0;
  }
  WiFi.mode(WIFI_AP);                         // the robot makes its own WiFi network
  WiFi.softAP("LMS-Bot", "12345678");         // network name and password
  udp.begin(4210);                            // listen for messages on port 4210
  Serial.print("WiFi started. Robot address: ");
  Serial.println(WiFi.softAPIP());            // normally 192.168.4.1
  Serial.println("Keep the robot still for 1 second...");
  startTime = millis();                       // remember when the stillness started
  lastMicros = micros();                      // start the loop timer
}

void loop() {
  // ===== 1. COMMANDS FROM THE PHONE / COMPUTER =====
  String command = "";                        // empty = no new command
  String reply = "";                          // text to send back
  bool startNow = false;                      // becomes true when we get the "go" command
  int packetSize = udp.parsePacket();         // is a message waiting?
  if (packetSize > 0) {
    char text[100];                           // place to keep the message
    int length = udp.read(text, 99);          // read it
    text[length] = 0;                         // put an "end of text" mark
    command = String(text);                   // use it as the command
    clientIP = udp.remoteIP();                // remember who sent it...
    clientPort = udp.remotePort();            // ...and from which port
    clientKnown = true;                       // now we know where to send the DATA lines
  }
  command.trim();                             // remove spaces and line endings
  if (command.startsWith("kp ")) { KP = command.substring(3).toFloat(); reply = "OK KP=" + String(KP, 3); }          // "kp 15" changes KP
  else if (command.startsWith("ki ")) { KI = command.substring(3).toFloat(); reply = "OK KI=" + String(KI, 3); }
  else if (command.startsWith("kd ")) { KD = command.substring(3).toFloat(); reply = "OK KD=" + String(KD, 3); }
  else if (command.startsWith("speed ")) { SPEED = command.substring(6).toInt(); reply = "OK SPEED=" + String(SPEED); }
  else if (command == "go") { startNow = true; reply = "OK go"; }
  else if (command == "stop") { if (mode >= FOLLOW) mode = WAIT; reply = "OK stop"; }
  else if (command == "help") reply = "commands: go, stop, kp x, ki x, kd x, speed x";
  else if (command != "") reply = "unknown command, send help";

  // ===== 2. READ THE GYRO =====
  Wire.beginTransmission(imuAddress);         // talk to the IMU
  Wire.write(0x26);                           // gyro Z numbers start at 0x26
  Wire.endTransmission(false);
  Wire.requestFrom(imuAddress, (byte)2);      // read 2 bytes
  int low = Wire.read();                      // first byte (low part)
  int high = Wire.read();                     // second byte (high part)
  float gyroRaw = (int16_t)(high << 8 | low) * 0.0175;   // degrees per second (still includes the zero offset)
  float gyroSpeed = gyroRaw - gyroBias;       // real turning speed
  unsigned long now = micros();               // time now
  heading += IMU_DIRECTION * gyroSpeed * (now - lastMicros) * 0.000001;   // speed x time = angle turned
  lastMicros = now;                           // remember the time for the next loop

  // ===== 3. MEASURE THE REAL WHEEL SPEED =====
  long leftNow = leftTicks;                   // copy the counts (the interrupts keep changing them)
  long rightNow = rightTicks;
  float realSpeed = ((leftNow - lastLeftTicks) + (rightNow - lastRightTicks)) / 2.0;   // average ticks since the last loop
  lastLeftTicks = leftNow;                    // remember the counts for the next loop
  lastRightTicks = rightNow;

  // ===== 4. READ THE 16 SENSORS =====
  float weightedSum = 0;                      // adds up (sensor value x sensor position)
  float totalSum = 0;                         // adds up all the sensor values
  int sensorsOnLine = 0;                      // counts how many sensors see the line
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
    if (value > 500) sensorsOnLine++;         // this sensor sees the line
    weightedSum += value * IR_POSITION[i];    // strong sensors on the right push the average to the right
    totalSum += value;
  }

  // ===== 5. DECIDE WHAT TO DO =====
  int leftSpeed = 0;                          // power for the left motor (0 = stopped unless we change it)
  int rightSpeed = 0;                         // power for the right motor
  float correction = 0;                       // how much to steer

  if (mode == GYRO_ZERO) {                    // --- finding the gyro zero (robot still) ---
    gyroTotal += gyroRaw;                     // add up the readings
    gyroCount++;
    if (millis() - startTime > 1000) {        // 1 second is over
      gyroBias = gyroTotal / gyroCount;       // the average reading = the zero value
      Serial.println("Calibrating for 4 seconds: slide the array over the line and the black floor");
      digitalWrite(LED_PIN, HIGH);            // LED on while calibrating
      startTime = millis();
      mode = CALIBRATE;
    }
  }
  else if (mode == CALIBRATE) {               // --- calibrating the IR sensors ---
    if (millis() - startTime > 4000) {        // 4 seconds are over
      digitalWrite(LED_PIN, LOW);             // LED off
      Serial.println("Put the robot on the line and send go (or press the button)");
      mode = WAIT;
    }
  }
  else if (mode == WAIT) {                    // --- waiting for "go" or the button ---
    if (startNow || digitalRead(BUTTON_PIN) == BUTTON_PRESSED) {   // go command or button
      integral = 0;                           // forget old PID memory
      lastError = 0;
      mode = FOLLOW;
    }
  }
  else if (mode == FOLLOW) {                  // --- following the line ---
    if (sensorsOnLine > 0) {                  // we can see the line
      error = weightedSum / totalSum;         // average position of the line (-7.5 to +7.5)
      integral = constrain(integral + error, -100, 100);   // I part: errors added up (limited)
      correction = KP * error + KI * integral + KD * (error - lastError);   // the PID formula
      lastError = error;                      // remember for the next loop
      leftSpeed = SPEED + correction;         // line on the right -> left wheel faster -> robot turns right
      rightSpeed = SPEED - correction;        // and the right wheel slower
    }                                         // if no line is seen, both speeds stay 0 = the robot stops
  }

  // ===== 5. SEND THE POWER TO THE MOTORS =====
  leftSpeed = constrain(leftSpeed * LEFT_MOTOR_DIRECTION, -MAX_PWM, MAX_PWM);      // flip if wired backwards, never above the limit
  rightSpeed = constrain(rightSpeed * RIGHT_MOTOR_DIRECTION, -MAX_PWM, MAX_PWM);
  digitalWrite(LEFT_MOTOR_IN1, leftSpeed >= 0);    // forward: IN1 high, IN2 low
  digitalWrite(LEFT_MOTOR_IN2, leftSpeed <= 0);    // backward: IN1 low, IN2 high (both high = brake)
  ledcWrite(LEFT_MOTOR_PWM, abs(leftSpeed));       // speed of the left motor
  digitalWrite(RIGHT_MOTOR_IN1, rightSpeed >= 0);
  digitalWrite(RIGHT_MOTOR_IN2, rightSpeed <= 0);
  ledcWrite(RIGHT_MOTOR_PWM, abs(rightSpeed));     // speed of the right motor

  // ===== 6. SEND THE DATA (and the answer to a command) =====
  String message = reply;                     // an answer to a command goes first
  if (message == "" && millis() - lastSendTime >= 50) {   // otherwise send the data 20 times per second
    lastSendTime = millis();
    message = "DATA," + String(millis()) + "," + String(mode) + "," + String(error, 3) + "," + String(correction, 2)
            + "," + String(leftSpeed) + "," + String(rightSpeed) + "," + String(heading, 1) + "," + String(gyroSpeed, 1)
            + "," + String(leftTicks) + "," + String(rightTicks) + "," + String(realSpeed, 1) + "," + String(sensorsOnLine);
    for (int i = 0; i < 16; i++) message += "," + String(sensorValue[i]);   // then the 16 IR sensor values
  }
  if (message != "") {                        // is there something to send?
    if (clientKnown) {                        // only send when we know the address
      udp.beginPacket(clientIP, clientPort);  // start a message to the phone / PC
      udp.print(message);                     // put the text in it
      udp.endPacket();                        // send it
    }
  }
  delay(10);                                  // one loop = about 10 ms
}
