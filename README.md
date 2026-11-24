# Saratech

Saratech combines the three supplied Arduino libraries and adds the movement,
gyro, and MV sensor logic from the supplied `2C.ino` sketch.

## Included components

| Original component | Main class | Purpose |
|---|---|---|
| `GTD35` | `GTD35` | Four-motor serial driver |
| `TDAxis12` | `TDAxis12` | Angle reading over I2C |
| `Adafruit_SH1106_STM32` | `Adafruit_SH1106` | SH1106 OLED display driver |
| `Saratech` | `Saratech` | One-call hardware setup, MV reading, gyro values, and two-axis movement |

The original `GTD35`, `TDAxis12`, and SH1106 source files and APIs are retained.
The high-level functions are implemented in `src/Saratech.cpp`.

## Install

1. Copy the `Saratech` folder into your Arduino `libraries` folder.
2. Install **Adafruit GFX Library** through Arduino Library Manager.
3. Use a compatible STM32 Arduino core that provides `HardWire.h` and the
   `TwoWire` API used by the supplied project.
4. Restart Arduino IDE.

## Simple setup (recommended)

The default `Saratech` constructor creates the hardware objects that were
previously declared as separate global objects in `2C.ino`:

- `Adafruit_SH1106 display(-1)`
- `GTD35 driver(Serial1)`
- `TwoWire i2c(2, I2C_FAST_MODE)`
- `TDAxis12 gyro(&i2c, 0x10)`

Declare only one controller object:

```cpp
#include <Saratech.h>

Saratech robot;

void setup() {
    robot.setup();
}

void loop() {
    robot.read_mv();
    robot.read_gyro();
    robot.move(-robot.ball_x * 5.0f, -robot.ball_y * 5.0f);
}
```

`robot.setup()` performs the original initialization sequence and values:

1. `Serial1` motor driver starts at `115200` baud.
2. Wait `200` milliseconds.
3. Start I2C bus `TwoWire(2, I2C_FAST_MODE)`.
4. Initialize the OLED with the same `display.begin(0x3c)` call as the supplied sketch.
5. Set OLED text size to `2` and text color to `WHITE`.

The original OLED object is available as `robot.display`. Use its existing
Adafruit_SH1106 methods through that member, for example:

```cpp
robot.display.clearDisplay();
robot.display.setCursor(0, 0);
robot.display.println(robot.ball_x);
robot.display.display();
```

## Compatibility mode (external hardware objects)

The previous constructor remains available. Existing code that creates the
components itself can continue to use it:

```cpp
GTD35 driver(Serial1);
TwoWire i2c(2, I2C_FAST_MODE);
TDAxis12 gyro(&i2c, 0x10);
Saratech robot(driver, gyro, i2c);

void setup() {
    robot.setup();
}
```

This mode uses the supplied motor driver, gyro, and I2C objects. The OLED is
still owned by `robot` and is available as `robot.display`.

## High-level functions

### `read_mv()`

```cpp
bool received = robot.read_mv();
int x = robot.ball_x;
int y = robot.ball_y;
```

This keeps the supplied sketch's 4-byte I2C read and coordinate conversion.
It reads two little-endian 16-bit values from address `0x42`. When both values
are zero, both coordinates remain zero. Otherwise, it subtracts `160` from X
and `120` from Y. If a complete 4-byte packet is not available, the previous
coordinates remain unchanged and the function returns `false`.

### `read_gyro()`

```cpp
float rawGyro = robot.read_gyro();
float scaledGyro = robot.gyu;
```

- `robot.gy` stores the raw value returned by `TDAxis12::read()`.
- `robot.gyu` stores `robot.gy * 300`.
- `TDAxis12::read()` itself is unchanged.

### `move(x, y)`

```cpp
robot.move(x, y);
```

This combines the old `movexy()` wheel mix and the old gyro correction in one
function. The four motor commands keep the original order and calculation:

```text
motor1 =  y + x - gy * coefficient
motor2 =  y - x - gy * coefficient
motor3 = -y - x - gy * coefficient
motor4 = -y + x - gy * coefficient
```

The default correction coefficient is `3`, which matches the supplied code's
`gy * 3` correction.

Change the public variable before calling `move()` when needed:

```cpp
robot.speedCoefficient = 4.0f;
robot.move(x, y);
```

Or set the coefficient for one call only:

```cpp
robot.move(x, y, 4.0f);
```

The `move()` coefficient is used for the original raw gyro correction. The
separate `gyu` value is provided as `gy * 300`; it does not replace the original
`gy * 3` correction. This keeps the supplied movement behavior unchanged at
the default setting.

## Complete example

See `examples/2C_Original_Functionality/2C_Original_Functionality.ino`.
It uses the self-contained setup and controller object.

## Compatibility and verification notes

- `GTD35.h/.cpp`, `TDAxis12.h/.cpp`, and `Adafruit_SH1106_STM32.h/.cpp` are
  retained without changes.
- The original component class names and functions remain available through
  `#include <Saratech.h>`.
- The display driver depends on **Adafruit GFX Library**. It is not included
  because it was not one of the three supplied libraries.
- The display driver uses the STM32-specific `HardWire` implementation from
  the supplied library.
- The one-call setup uses the hardware assignments from the supplied sketch:
  `Serial1`, I2C instance `2`, `I2C_FAST_MODE`, gyro address `0x10`, and MV
  address `0x42`.
- Physical operation requires validation on the target STM32 board and hardware.

## Original source and notices

The SH1106 source retains its original Adafruit attribution and BSD-license
notice. The GTD35 and TDAxis12 source files are preserved from the supplied
archive. No new license is asserted for those two components.


## STM32 compatibility fix (1.2.1)

Removed an unused AVR-only `avr/pgmspace.h` include from the SH1106 driver. The driver contains no `PROGMEM` or `pgm_read_*` calls, so this header is not required on STM32. The display driver still uses the STM32 `HardWire` interface.
