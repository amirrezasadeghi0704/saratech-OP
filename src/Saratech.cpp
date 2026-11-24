#include "Saratech.h"

Saratech::Saratech()
    : display(-1),
      _ownedMotorDriver(Serial1),
      _ownedMvWire(2, I2C_FAST_MODE),
      _ownedGyro(&_ownedMvWire, 0x10),
      _motorDriver(&_ownedMotorDriver),
      _gyro(&_ownedGyro),
      _mvWire(&_ownedMvWire),
      _mvAddress(0x42) {
}

Saratech::Saratech(GTD35 &motorDriver, TDAxis12 &gyro, TwoWire &mvWire,
                   uint8_t mvAddress)
    : display(-1),
      _ownedMotorDriver(Serial1),
      _ownedMvWire(2, I2C_FAST_MODE),
      _ownedGyro(&_ownedMvWire, 0x10),
      _motorDriver(&motorDriver),
      _gyro(&gyro),
      _mvWire(&mvWire),
      _mvAddress(mvAddress) {
}

void Saratech::setup() {
    // Keep the initialization sequence and values from the original 2C.ino.
    _motorDriver->begin(115200);
    delay(200);
    _mvWire->begin();
    display.begin(0x3c);
    display.setTextSize(2);
    display.setTextColor(WHITE);
}

bool Saratech::read_mv() {
    _mvWire->requestFrom(_mvAddress, static_cast<uint8_t>(4));

    // Keep the previous coordinates if a complete packet is not available.
    if (_mvWire->available() != 4) {
        return false;
    }

    uint8_t buffer[4];
    for (uint8_t i = 0; i < 4; ++i) {
        buffer[i] = static_cast<uint8_t>(_mvWire->read());
    }

    ball_x = static_cast<int>(buffer[0]) |
             (static_cast<int>(buffer[1]) << 8);
    ball_y = static_cast<int>(buffer[2]) |
             (static_cast<int>(buffer[3]) << 8);

    // Preserve the coordinate conversion from the original read_MV() code.
    if (ball_x == 0 && ball_y == 0) {
        ball_x = 0;
        ball_y = 0;
    } else {
        ball_x -= 160;
        ball_y -= 120;
    }

    return true;
}

float Saratech::read_gyro() {
    gy = _gyro->read();       // Raw TDAxis12 result: unchanged.
    gyu = gy * GYU_SCALE;    // Scaled value requested by the user.
    return gy;
}

bool Saratech::move(float x, float y) {
    return move(x, y, speedCoefficient);
}

bool Saratech::move(float x, float y, float correctionCoefficient) {
    // Keep the original movexy() wheel mix and original gyro correction:
    // motor(y+x, y-x, -y-x, -y+x), then subtract gy * coefficient.
    // The old movexy() passed these float expressions to motor(int, int, int, int).
    // Preserve that first conversion to int before applying gyro correction.
    const int baseMotor1 = static_cast<int>(y + x);
    const int baseMotor2 = static_cast<int>(y - x);
    const int baseMotor3 = static_cast<int>(-y - x);
    const int baseMotor4 = static_cast<int>(-y + x);

    const float correction = gy * correctionCoefficient;
    const int16_t motor1 = static_cast<int16_t>(baseMotor1 - correction);
    const int16_t motor2 = static_cast<int16_t>(baseMotor2 - correction);
    const int16_t motor3 = static_cast<int16_t>(baseMotor3 - correction);
    const int16_t motor4 = static_cast<int16_t>(baseMotor4 - correction);

    return _motorDriver->motor(motor1, motor2, motor3, motor4);
}
