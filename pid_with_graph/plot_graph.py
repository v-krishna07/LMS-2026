# plot_graph.py - draws a live graph of the robot's DATA lines and lets you type commands (kp 15, go, stop ...).
# Needs:  pip install matplotlib pyserial
# Use:
#   UDP over WiFi (join the WiFi "LMS-Bot" first):   python plot_graph.py udp 192.168.4.1
#   Bluetooth (pair LMS-Bot first, use its COM port): python plot_graph.py serial COM5
# Type a command in this terminal and press Enter. Close the graph window to stop.
import sys, socket, threading, time
import matplotlib.pyplot as plt

mode = sys.argv[1]                                  # "udp" or "serial"
target = sys.argv[2]                                # robot IP address, or the COM port name
errors = []                                         # the line error over time (what we plot)
corrections = []                                    # the PID correction over time
times = []                                          # time in seconds
lock = threading.Lock()                             # stops the two threads from touching the lists at the same time

if mode == "udp":
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)     # a UDP connection
    sock.settimeout(0.2)                                        # do not wait forever for a message
    robot = (target, 4210)                                      # robot address and port
    sock.sendto(b"help\n", robot)                               # say hello so the robot knows where to send the data
else:
    import serial
    port = serial.Serial(target, 115200, timeout=0.2)           # the Bluetooth COM port

def send(text):                                     # send one command line to the robot
    if mode == "udp": sock.sendto((text + "\n").encode(), robot)
    else: port.write((text + "\n").encode())

def read_lines():                                   # runs in the background: reads the robot's messages
    buffer = ""
    while True:
        try:
            if mode == "udp": buffer += sock.recv(2000).decode(errors="ignore")
            else: buffer += port.read(200).decode(errors="ignore")
        except Exception:
            continue                                # nothing arrived this time
        while "\n" in buffer:                       # a full line is ready
            line, buffer = buffer.split("\n", 1)
            line = line.strip()
            if line.startswith("DATA,"):            # DATA,time,mode,error,correction,...
                part = line.split(",")
                with lock:
                    times.append(float(part[1]) / 1000.0)
                    errors.append(float(part[3]))
                    corrections.append(float(part[4]))
            elif line:
                print("robot:", line)               # answers to commands

def type_commands():                                # runs in the background: sends what you type
    while True:
        send(input())

threading.Thread(target=read_lines, daemon=True).start()
threading.Thread(target=type_commands, daemon=True).start()

plt.ion()                                           # live graph
figure, axis = plt.subplots()
while plt.fignum_exists(figure.number):
    with lock:
        x = times[-300:]; e = errors[-300:]; c = corrections[-300:]    # show the last 300 points (about 15 seconds)
    axis.clear()
    axis.plot(x, e, label="error (0 = line in the middle)")
    axis.plot(x, [v / 10 for v in c], label="correction / 10")
    axis.axhline(0, color="gray")
    axis.legend(loc="upper left")
    axis.set_xlabel("time (s)")
    plt.pause(0.1)
