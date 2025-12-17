# ESP32-S3 Robot + Edge Impulse (Computer Vision)

This robot uses an ESP32-S3 microcontroller and an Edge Impulse model to detect a measuring cup from the camera image.
Operating logic: frame capture → inference → movement; when the cup is close enough, the robot stops, performs a small “nudge”, grips the cup with a servo/gripper, and lifts it using a linear actuator.

## Hardware
- ESP32-S3 (with camera interface)
- Camera (RGB565, QVGA 320×240)
- DRV8833 (TT DC motors, 2 wheel pairs)
- TB6612FNG (linear actuator)
- 3× servos (2× MG90S, 1× MG996R)
- Ultrasonic sensor(s)
- Power: 6V battery; motors powered via a separate MINI360 DC-DC branch at ~3V; servos powered from a separate 5V rail; common GND

### Camera connections (ESP32-S3)
- XCLK → GPIO15
- SIOD / SDA → GPIO4
- SIOC / SCL → GPIO5
- D0 → GPIO11
- D1 → GPIO9
- D2 → GPIO8
- D3 → GPIO10
- D4 → GPIO12
- D5 → GPIO18
- D6 → GPIO17
- D7 → GPIO16
- VSYNC → GPIO6
- HREF → GPIO7
- PCLK → GPIO13
- RESET: not used
- PWDN: not used

### Motors (DRV8833)
- RIGHT AIN1 → GPIO45
- RIGHT AIN2 → GPIO46
- LEFT  BIN1 → GPIO47
- LEFT  BIN2 → GPIO48

Motor speed is not PWM-modulated; motors run at a constant speed.

### Linear actuator (TB6612FNG)
- AIN1 → GPIO40
- AIN2 → GPIO39  
VM: 6V battery supply (depending on the build)

### Servos
- Front MG90S → GPIO42
- Rear  MG90S → GPIO43
- MG996R (gripper) → GPIO41  
Servo power: separate 5V supply rail.

### Ultrasonic sensor #1
- TRIG → GPIO1
- ECHO → GPIO2
- VCC → 5V
- GND → common ground

### Ultrasonic sensor #2 (optional)
- TRIG → GPIO19 (USB D+)
- ECHO → GPIO20 (USB D−)

**Note:** when sensor #2 is connected to GPIO19/20, USB debugging and Serial monitoring are not available. For testing, this sensor is commented out in the code.

### I2C
- SDA → GPIO14
- SCL → GPIO21

## Build and upload (Arduino IDE)
1. Install the ESP32 board package (Espressif)
2. Open `src/robot_edge_impulse.ino`
3. Ensure the `edge-impulse/` folder is included in the project (or that the Edge Impulse library is installed in Arduino Libraries)
4. Select the correct board (ESP32-S3) and port
5. Upload

<img width="622" height="861" alt="image" src="https://github.com/user-attachments/assets/68ae742d-7973-4ffe-866a-2c86daf47928" />


