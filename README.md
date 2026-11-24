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
3. Use the official **STM32duino / STM32 MCU based boards** core.
   This version uses its pin-based `TwoWire(SDA, SCL)` API.
4. Restart Arduino IDE.

## Simple setup (recommended)

The default `Saratech` constructor creates the hardware objects that were
previously declared as separate global objects in `2C.ino`:

- `Adafruit_SH1106 display(-1)`
- `GTD35 driver(Serial1)`
- `TwoWire i2c(PB11, PB10)` with a 400 kHz bus clock
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
3. Start I2C2 on PB11 (SDA) and PB10 (SCL) at 400 kHz. The OLED, MV sensor, and gyro share this same `TwoWire` object.
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
TwoWire i2c(PB11, PB10);
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

- The original GTD35 and TDAxis12 APIs are retained.
- The SH1106 display API is retained. Its I2C transport now uses the official STM32duino `Wire` API.
- The display, MV sensor, and TDAxis12 gyro share one `TwoWire` object when used through `Saratech`.
- For the STM32F103C8T6 Blue Pill, the default I2C2 pins are PB11 (SDA) and PB10 (SCL). The bus clock is set to 400 kHz.
- The display driver depends on **Adafruit GFX Library**. Install it through Arduino Library Manager.
- Physical operation requires validation on the target STM32 board and hardware.

## Original source and notices

The SH1106 source retains its original Adafruit attribution and BSD-license
notice. The GTD35 and TDAxis12 source files are preserved from the supplied
archive. No new license is asserted for those two components.


## Version 1.2.3

- Uses the official STM32duino `TwoWire(SDA, SCL)` API.
- Configures STM32F103C8T6 I2C2 as SDA=PB11 and SCL=PB10, with a 400 kHz clock.
- Shares one `TwoWire` object between the OLED, MV sensor, and TDAxis12 gyro.
- Uses master-mode `begin()` for the I2C bus.
- Uses `SPISettings` instead of the removed `SPI.setClockDivider()` API.
