# MotorCell

<img src="https://microbots.io/cdn/shop/files/NCNBnM-Copy_800x.png?v=1732024552" alt="MotorCell" width="300" align="right" style="margin-left: 20px;">

MotorCell is an ultra-compact, shaftless PCB motor designed for high-speed, low-torque applications. Featuring planar PCB windings, sensorless control, and a unique pancake design, MotorCell is ideal for robotics, DIY projects, and art installations. This innovative motor simplifies integration by eliminating the need for external sensors while offering speed control via PWM.

## Setting Up the MotorCell

### 1. Connecting Your MotorCell

- **Pins**:
  - `IN`: PWM input for speed control.
  - `OUT`: Speed reading (requires pull-up resistor).
  - `FR`: Direction control.

### 2. Using the MotorCell Library

Install the library from the Arduino Library manager and include it in your Arduino project:

```cpp
#include "MotorCell.h"

#define IN_pin1 2
#define OUT_pin2 3
#define FR_pin2 1

MotorCell myMotorCell(IN_PIN, OUT_PIN, FR_PIN);

void setup() {
    myMotorCell.Init();
}
```

---

## MotorCell Library Functions

### 1. **Init**
- **Function:** `Init()`
- **Description:** Sets up the GPIO pins and initializes the motor.

```cpp
myMotorCell.Init();
```

### 2. **Spin**
- **Function:** `Spin(uint8_t speed_percent)`
- **Description:** Spins the motor at the specified speed.

```cpp
uint16_t rpm = myMotorCell.Spin(75); // Spin at 75% speed
```

### 3. **SpinPID**
- **Function:** `SpinPID(uint16_t target_rpm)`
- **Description:** Uses PID control to maintain a target RPM (requires ESP32 or similar).

```cpp
uint16_t rpm = myMotorCell.SpinPID(10000); // Spin at 10,000 RPM
```

### 4. **ReverseSpin**
- **Function:** `ReverseSpin()`
- **Description:** Reverses the motor's direction.

```cpp
myMotorCell.ReverseSpin();
```

### 5. **RPMRead**
- **Function:** `RPMRead()`
- **Description:** Reads the motor's current RPM.

```cpp
uint16_t currentRPM = myMotorCell.RPMRead();
```

### 6. **MaxSpin**
- **Function:** `MaxSpin()`
- **Description:** Spins the motor at maximum speed.

```cpp
myMotorCell.MaxSpin();
```

---

## Usage Notes:

1. **Speed Control**:
   - Motor speed decreases as load increases. 
   
2. **3D-Printed Parts**:
   - Recommended press-fit diameter: 16.4mm–16.6mm.
   - Use superglue for additional security if needed.

3. **Soldering**:
   - Take care when soldering near the rotor's magnets; the tip may be drawn to the motor.

---

## Example Code

Here’s a basic example to get started with MotorCell:

```cpp
#include "MotorCell.h"

MotorCell myMotorCell(5, 6, 7);  // Define MotorCell pins (IN, OUT, FR)

void setup() {
    myMotorCell.Init();         // Initialize MotorCell
    myMotorCell.Spin(50);       // Spin at 50% speed
}

void loop() {
    uint16_t rpm = myMotorCell.RPMRead();  // Read current RPM
    Serial.println(rpm);
    delay(1000);
}
```

---

