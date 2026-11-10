# LMS code (hardware checks, PID, PID with graph)

Every program is a normal Arduino sketch (`.ino`) in its own folder, because the Arduino IDE needs that.
Board: ESP32 Dev Module, Arduino-ESP32 core 3.x. The code style is simple on purpose: no helper functions
(only the two small encoder interrupt functions, which the ESP32 requires), one sensor block and one motor block per program,
and a comment on almost every line.

## First step: fill `pins.h`
Every wire of the robot has a name there with a comment. Write the GPIO numbers, then change `PINS_FILLED` to `true`.
Until then every program stops at compile time with a message. All programs include it with `#include "../../pins.h"`.

## Folder 1: hardware_checking (Serial Monitor at 115200)
| Sketch | What it checks |
|---|---|
| esp32_check | ESP32, USB, Serial Monitor |
| led_check | red LED |
| button_check | push button |
| ir_sensor_check | the 16 IR sensors |
| imu_check | gyro and accelerometer |
| encoder_check | wheel encoders |
| motor_check | both motors forward and backward (lift the robot) |
| drift_check | both motors with the same power: does the robot curve? |
| test_everything | all of them in one program: type a letter (`h` = help) |

## Folder 2: pid
| Sketch | What it does |
|---|---|
| pid_normal | line follower with the IR array only |
| pid_encoder | drives straight using only the encoders |
| pid_imu | holds a direction using only the gyro (button = turn 90 degrees) |
| pid_intersection | IR PID that also handles T, +, corners, dead end, end box (left first, then straight, then right) |
| pid_combined | IR PID + encoder speed control + gyro when the line is lost |
| pid_plus | everything: corner slow-down, gyro turns, junctions, end box |

The IR sketches calibrate first: the LED is on for 4 seconds, slide the array over the white line and the black floor.
Then put the robot on the line and press the button.

## Folder 3: pid_with_graph (tuning while the robot drives)
| Sketch | Program | Connection |
|---|---|---|
| pid_bluetooth | simple PID | Bluetooth |
| pid_plus_bluetooth | PID plus | Bluetooth |
| pid_udp_wifi | simple PID | UDP over WiFi |
| pid_plus_udp_wifi | PID plus | UDP over WiFi |

- They send a `DATA` line 20 times per second: time, mode, error, correction, left and right power, heading, gyro speed,
  left and right encoder ticks, real speed, sensors on line, and the 16 IR values.
- You can type commands while it drives: `go`, `stop`, `kp 15`, `ki 0.1`, `kd 40`, `speed 130` (`ticks` and `slow` in the plus versions), `help`.
- `plot_graph.py` draws the error graph live and lets you type the commands:
  `python plot_graph.py udp 192.168.4.1` or `python plot_graph.py serial COM5` (needs `pip install matplotlib pyserial`).
- Bluetooth here is Bluetooth Classic serial (a text terminal over Bluetooth, used like Telnet). Real Telnet runs over WiFi, so the WiFi files use UDP.
- If a Bluetooth sketch is too big to upload, choose Tools > Partition Scheme > Huge APP.
- WiFi and Bluetooth are not allowed in the competition arena. These sketches are for tuning only.

## Suggested order
1. esp32_check, led_check, button_check
2. ir_sensor_check (set `WHITE_LINE_IS_HIGH` in pins.h), imu_check
3. motor_check (fix the direction flags), encoder_check (fix the encoder direction flags), drift_check
4. pid_normal, then pid_encoder, pid_imu, pid_intersection
5. pid_with_graph: tune KP and KD with the graph, then pid_combined / pid_plus
