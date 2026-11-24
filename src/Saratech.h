#ifndef SARATECH_H
#define SARATECH_H

/*
 * Saratech - Combined Arduino library.
 *
 * This header exposes the original public APIs of:
 *   - GTD35
 *   - TDAxis12
 *   - Adafruit_SH1106_STM32
 *
 * It also provides a high-level robot controller that combines the supplied
 * 2C.ino movement, gyro-correction, and read_MV logic.
 */

#include <Arduino.h>
#include <Wire.h>
#include "GTD35.h"
#include "TDAxis12.h"
#include "Adafruit_SH1106_STM32.h"

class Saratech {
public:
    /*
     * Self-contained mode.
     * Uses the STM32duino pin-based API for I2C2:
     * Serial1, TwoWire(PB11, PB10), 400 kHz, TDAxis12 address 0x10,
     * MV address 0x42, and SH1106 display initialization value 0x3C.
     */
    Saratech();

    /*
     * Compatibility mode for projects that create GTD35, TDAxis12, and
     * TwoWire objects outside this class. Existing constructor remains valid.
     */
    Saratech(GTD35 &motorDriver, TDAxis12 &gyro, TwoWire &mvWire,
             uint8_t mvAddress = 0x42);

    // OLED display owned by this Saratech object.
    // Use robot.display to call the original Adafruit_SH1106 methods.
    Adafruit_SH1106 display;

    // Run the hardware setup sequence from the original sketch.
    // Call once from the Arduino global setup() function: robot.setup();
    void setup();

    // Raw values from the original sketch.
    int ball_x = 0;
    int ball_y = 0;
    float gy = 0.0f;

    // Scaled gyro value: gyu = gy * 300. The raw gy value remains unchanged.
    float gyu = 0.0f;
    static constexpr float GYU_SCALE = 300.0f;

    // Gyro correction coefficient used by move(x, y). Original default: 3.
    // Change this variable before calling move() when required.
    float speedCoefficient = 3.0f;

    // Read 4 bytes from the MV sensor at address 0x42 by default.
    // Coordinates use the same center offsets as the supplied sketch: 160, 120.
    // Returns true when a complete 4-byte reading is available.
    bool read_mv();

    // Read TDAxis12. Return and store the raw value in gy.
    // Also update gyu = gy * 300. The TDAxis12::read() method is not changed.
    float read_gyro();

    // Combined mecanum movement. x/y replace the old four motor inputs.
    // This overload uses the configurable speedCoefficient variable (default 3).
    bool move(float x, float y);

    // Same movement with a one-call coefficient override.
    bool move(float x, float y, float correctionCoefficient);

private:
    // Internal hardware objects used by the self-contained constructor.
    // They are also initialized in compatibility mode, but are not selected.
    GTD35 _ownedMotorDriver;
    TwoWire _ownedMvWire;
    TDAxis12 _ownedGyro;

    // Pointers select either the internal objects or external objects.
    GTD35 *_motorDriver;
    TDAxis12 *_gyro;
    TwoWire *_mvWire;
    uint8_t _mvAddress;
};

#endif // SARATECH_H
