#include "saratech_op.h"

// ── Global singleton ─────────────────────────────────────────────────────────
SaratechOP saratech;

// ── Constructor / destructor ─────────────────────────────────────────────────
SaratechOP::SaratechOP()
    : _gtd35(SARATECH_MOTOR_UART),
      _imu(SARATECH_IMU_I2C, SARATECH_IMU_ADDR),
      _lidar(nullptr),
      _servo(),
      _display(SARATECH_OLED_RST)
{
}

SaratechOP::~SaratechOP() {
    delete _lidar;
}

// ── Initialization ───────────────────────────────────────────────────────────
void SaratechOP::begin() {
    begin(_motor, _sensor, _shooter, _motion);
}

void SaratechOP::begin(const MotorConfig &mc, const SensorConfig &sc,
                       const ShooterConfig &shc, const MotionConfig &moc)
{
    _motor    = mc;
    _sensor   = sc;
    _shooter  = shc;
    _motion   = moc;
    _locCfg.initX_cm      = 0.0f;
    _locCfg.initY_cm      = 0.0f;
    _locCfg.initTheta_deg = 0.0f;
    _locCfg.imuGain       = 0.7f;
    _locCfg.lidarGain     = 0.3f;
    _locCfg.featureMin_cm = 30.0f;
    _locCfg.featureMax_cm = 500.0f;
    _locCfg.matchThresh   = 0.5f;

    // Re-initialise GTD35 with configured serial
    _gtd35 = GTD35(*_motor.serial);
    _gtd35.begin(_motor.baud, _motor.dirPin, (uint8_t)_motor.txLevel);
    _gtd35.setMotorIDs(_motor.idFL, _motor.idFR, _motor.idRR, _motor.idRL);
    _gtd35.setEnabled(true);
    _gtd35.setAngle(_motor.txAxisDefault);

    // IMU
    _imu.begin(_sensor.i2c, _sensor.imuAddr);

    // Analog sensor pins
    pinMode(SARATECH_SNSR_BOTTOM, INPUT_ANALOG);
    pinMode(SARATECH_SNSR_RIGHT,  INPUT_ANALOG);
    pinMode(SARATECH_SNSR_LEFT,   INPUT_ANALOG);

    // LDR pins
    pinMode(SARATECH_LDR_FRONT,  INPUT_ANALOG);
    pinMode(SARATECH_LDR_RIGHT,  INPUT_ANALOG);
    pinMode(SARATECH_LDR_BACK,   INPUT_ANALOG);
    pinMode(SARATECH_LDR_LEFT,   INPUT_ANALOG);

    // Kicker detect + trigger
    pinMode(SARATECH_KICKER_DETECT,  INPUT);
    pinMode(SARATECH_KICKER_TRIGGER, OUTPUT);
    digitalWrite(SARATECH_KICKER_TRIGGER, LOW);

    // OLED SH1106 (I2C2)
    _display.begin(SH1106_SWITCHCAPVCC, SARATECH_OLED_ADDR);
    _display.clearDisplay();
    _display.setTextSize(1);
    _display.setTextColor(WHITE);
    _display.setCursor(0, 0);
    _display.print("Saratech OP");
    _display.display();

    // Servo
    _servoIdx = _servo.setupServo(_shooter.servoPin,
                                   _shooter.servoMinUs,
                                   _shooter.servoMaxUs);

    // Spinback PWM pin
    if (_shooter.spinbackIsPwm) {
        pinMode(SARATECH_SPINBACK_PIN, OUTPUT);
    }

    // YDLIDAR T-mini Plus on Serial2 (PA2=RX, PA3=TX) — optional
    // Try to initialize; if it fails, localization falls back to IMU-only
    _lidar = new YDLIDAR_Tmini(SARATECH_LIDAR_UART);
    if (!_lidar->begin(SARATECH_LIDAR_BAUD)) {
        // LiDAR failed to initialize — will use IMU-only localization
        delete _lidar;
        _lidar = nullptr;
    }

    // Initialize pose estimator
    _poseEst.begin(_locCfg.initX_cm, _locCfg.initY_cm, _locCfg.initTheta_deg);
    _poseEst.setIMUGain(_locCfg.imuGain);
    _poseEst.setLidarGain(_locCfg.lidarGain);
    _poseEst.setFeatureDistanceMin(_locCfg.featureMin_cm);
    _poseEst.setFeatureDistanceMax(_locCfg.featureMax_cm);
    _poseEst.setMatchThreshold(_locCfg.matchThresh);

    // Reset odometry and state
    _posX = 0.0f; _posY = 0.0f;
    _yaw = 0.0f; _yawRaw = 0.0f;
    _lastSpeed_cms = 0.0f;
    _lastMoveMs = millis();
    _initialized = true;
    _localized = false;
    _lidarConf = 0.0f;
    _lidarHealth = 0;
}

// ── Wheel geometry setters ───────────────────────────────────────────────────
void SaratechOP::setWheelDiameter(float cm)  { _motor.wheelRadius_cm = cm / 2.0f; }
void SaratechOP::setWheelRadius(float cm)     { _motor.wheelRadius_cm = cm; }
void SaratechOP::setWheelBase(float cm)       { _motor.wheelBase_cm = cm; }
void SaratechOP::setTrackWidth(float cm)      { _motor.track_cm = cm; }

// ── Speed / accel setters ────────────────────────────────────────────────────
void SaratechOP::setMaxSpeed(float cms)     { _motor.maxSpeed_cms = cms; }
void SaratechOP::setMaxAccel(float cms2)    { _motor.maxAccel_cms2 = cms2; }
void SaratechOP::setMaxMotorRPM(float rpm)  { _motor.maxMotorRPM = rpm; }

// ── TDAXIS / TXAXIS setters ─────────────────────────────────────────────────
void SaratechOP::setTDAxisCoefficient(uint16_t v) {
    if (v > 512) v = 512;
    _motion.tdAxisCoeff = v;
}
void SaratechOP::setTDAxis(uint16_t v)       { setTDAxisCoefficient(v); }
void SaratechOP::setTXAxisCoefficient(uint16_t v) { setTDAxisCoefficient(v); }
void SaratechOP::setTXAxisGain(float g)      { _motor.txAxisGain = g; }
void SaratechOP::setTXAxisAngle(uint16_t a)  {
    _motor.txAxisDefault = a;
    _gtd35.setAngle(a);
}

// ── Sensor setters ───────────────────────────────────────────────────────────
void SaratechOP::setLDRThreshold(int t)                        { _sensor.ldrThresh = t; }
void SaratechOP::setLDRBaseline(int idx, int v) {
    if (idx >= 0 && idx < 4) _sensor.ldrBasis[idx] = v;
}
void SaratechOP::setKickerThreshold(float dist, float ang) {
    _sensor.kickerThresh = dist;
    _sensor.kickerAngleLim = ang;
}
void SaratechOP::setServoLimits(uint16_t minUs, uint16_t maxUs,
                                 float minDeg, float maxDeg) {
    _shooter.servoMinUs  = minUs;
    _shooter.servoMaxUs  = maxUs;
    _shooter.servoMinDeg = minDeg;
    _shooter.servoMaxDeg = maxDeg;
    if (_servoIdx >= 0) {
        _servo.deleteServo(_servoIdx);
        _servoIdx = _servo.setupServo(_shooter.servoPin,
                                       _shooter.servoMinUs,
                                       _shooter.servoMaxUs);
    }
}
void SaratechOP::setSpinbackMaxSpeed(uint8_t s) { _shooter.spinbackMax = s; }
void SaratechOP::setPositionTolerance(float cm)  { _motion.posTol_cm = cm; }
void SaratechOP::setAngleTolerance(float deg)    { _motion.angTol_deg = deg; }
void SaratechOP::setShiftCoefficient(float c)    { _motion.shiftCoeff = c; }

// ── Localization setters ─────────────────────────────────────────────────────
void SaratechOP::setInitialPose(float x_cm, float y_cm, float theta_deg) {
    _locCfg.initX_cm = x_cm;
    _locCfg.initY_cm = y_cm;
    _locCfg.initTheta_deg = theta_deg;
    _poseEst.reset(x_cm, y_cm, theta_deg);
    _posX = x_cm;
    _posY = y_cm;
    _yaw = theta_deg;
    _localized = false;
}

void SaratechOP::resetLocalization() {
    _poseEst.reset(0.0f, 0.0f, 0.0f);
    _posX = 0.0f; _posY = 0.0f;
    _yaw = 0.0f;
    _localized = false;
}

float SaratechOP::getPositionX() { return _posX; }
float SaratechOP::getPositionY() { return _posY; }
float SaratechOP::getHeading() { return _yaw; }

bool SaratechOP::updateLocalization() {
    // Always read IMU (works without LiDAR)
    float imuYaw = _imu.read();
    uint32_t now = millis();
    static uint32_t lastIMUms = 0;
    float dt_s = (float)(now - lastIMUms) / 1000.0f;
    lastIMUms = now;
    if (dt_s <= 0.0f) dt_s = 0.016f; // assume ~60Hz

    // IMU prediction (always works)
    _poseEst.predictIMU(imuYaw, dt_s);

    // Update heading from IMU always
    _yaw = normAngle(imuYaw + _motion.yawOffsetDeg);
    _yawRaw = imuYaw;

    // LiDAR update (only if LiDAR is present)
    if (_lidar != nullptr) {
        _lidar->tick();
        _lidarHealth = _lidar->getHealth();

        if (_lidar->isScanComplete()) {
            LiDARScan scan;
            scan.count = 0;
            for (int i = 0; i < 360; i++) {
                const YDLidarPoint& p = _lidar->getPoint((float)i);
                scan.angles[i] = p.angle_deg;
                scan.dists[i] = p.dist_mm;
                scan.qualities[i] = p.quality;
                if (p.valid && p.dist_mm > 0) scan.count++;
            }
            scan.timestamp = millis();

            if (_poseEst.updateLiDAR(scan)) {
                _posX = _poseEst.getPositionX();
                _posY = _poseEst.getPositionY();
                _yaw = _poseEst.getHeading();
                _localized = _poseEst.isLocalized();
                _lidarConf = _poseEst.getConfidence();
                return true;
            }
        }
    } else {
        // No LiDAR — use IMU-only heading estimation
        // Position is maintained via dead reckoning from motor commands
        _localized = false;
        _lidarConf = 0.0f;
        _lidarHealth = 2; // Error: no LiDAR
    }

    return _localized;
}

bool SaratechOP::isLocalizationValid() const { return _localized; }
float SaratechOP::getLocalizationConfidence() const { return _lidarConf; }
const LiDARScan& SaratechOP::getLastScan() const { return _poseEst.getLastScan(); }

void SaratechOP::setIMUGain(float gain) {
    _locCfg.imuGain = gain;
    _poseEst.setIMUGain(gain);
}
void SaratechOP::setLidarGain(float gain) {
    _locCfg.lidarGain = gain;
    _poseEst.setLidarGain(gain);
}
void SaratechOP::setFeatureDistanceRange(float min_cm, float max_cm) {
    _locCfg.featureMin_cm = min_cm;
    _locCfg.featureMax_cm = max_cm;
    _poseEst.setFeatureDistanceMin(min_cm);
    _poseEst.setFeatureDistanceMax(max_cm);
}
void SaratechOP::setMatchThreshold(float thresh) {
    _locCfg.matchThresh = thresh;
    _poseEst.setMatchThreshold(thresh);
}

// ── LiDAR control ────────────────────────────────────────────────────────────
void SaratechOP::updateLiDAR() {
    if (!_lidar) return;
    _lidar->tick();
    _lidarHealth = _lidar->getHealth();
}

bool SaratechOP::startLiDAR() {
    if (!_lidar) return false;
    return _lidar->startScan();
}

bool SaratechOP::stopLiDAR() {
    if (!_lidar) return false;
    return _lidar->stopScan();
}

uint8_t SaratechOP::getLidarHealth() const {
    if (!_lidar) return 2; // Error: LiDAR not initialized
    return _lidar->getHealth();
}

// ═════════════════════════════════════════════════════════════════════════════
// Core movement functions
// ═════════════════════════════════════════════════════════════════════════════

bool SaratechOP::move(float speed_cm_s, float accel_cm_s2, float direction_deg) {
    if (!_initialized) return false;

    // Clamp speed
    float spd = constrain(speed_cm_s, -_motor.maxSpeed_cms, _motor.maxSpeed_cms);

    // Acceleration ramp
    if (accel_cm_s2 > 0.0f) {
        uint32_t dt_ms = millis() - _lastMoveMs;
        if (dt_ms > 0) {
            float delta = spd - _lastSpeed_cms;
            float maxStep = (_motor.maxAccel_cms2 * (float)dt_ms) / 1000.0f;
            if (fabs(delta) > maxStep) spd = _lastSpeed_cms + (delta > 0 ? maxStep : -maxStep);
        }
    }
    _lastMoveMs = millis();
    _lastSpeed_cms = spd;

    // Convert direction to vx/vy (robot frame: 0=forward, 90=right)
    float rad = direction_deg * (M_PI / 180.0f);
    float vx  = sinf(rad) * spd;   // right-positive
    float vy  = cosf(rad) * spd;   // forward-positive

    // Compute wheel speeds and apply compensation
    int16_t fl, fr, rr, rl;
    computeWheelSpeeds(vx, vy, 0.0f, fl, fr, rr, rl);
    applyTXAXISComp(fl, fr, rr, rl);
    applyTDAXISInfluence(fl, fr, rr, rl);

    _gtd35.motor(fl, fr, rr, rl);
    return true;
}

bool SaratechOP::goTo(float targetX_cm, float targetY_cm, float speed_cm_s) {
    if (!_initialized) return false;

    // Use fused pose if available, otherwise use odometry
    float robotX = _posX;
    float robotY = _posY;

    float dx = targetX_cm - robotX;
    float dy = targetY_cm - robotY;
    float dist = sqrtf(dx * dx + dy * dy);

    if (dist <= _motion.posTol_cm) {
        stopMotors();
        return true;
    }

    // Desired heading toward target (robot frame)
    float targetAngle = degrees(atan2f(dx, dy));
    targetAngle = normAngle(targetAngle);

    // Apply shift offset if configured
    targetAngle += (_motion.shiftCoeff != 0.0f)
                   ? (_motion.shiftCoeff * 10.0f) : 0.0f;
    targetAngle = normAngle(targetAngle);

    // Check angular tolerance
    float angErr = fabsf(normAngle(targetAngle - _yaw));
    if (angErr <= _motion.angTol_deg && dist <= _motion.posTol_cm * 2.0f) {
        stopMotors();
        return true;
    }

    move(speed_cm_s, _motor.maxAccel_cms2, targetAngle);
    return false;
}

void SaratechOP::stopMotors() {
    _gtd35.motor(0, 0, 0, 0);
    _lastSpeed_cms = 0.0f;
}

// ── TXAXIS / TDAXIS ──────────────────────────────────────────────────────────
void SaratechOP::updateTXAXISCompensation() {
    if (!_initialized) return;

    _yawRaw = _imu.read();
    _yaw = normAngle(_yawRaw + _motion.yawOffsetDeg);

    // Map yaw to TXAXIS range for GTD35
    float txVal = ((_yaw + 180.0f) / 360.0f) * 65535.0f;
    _gtd35.setAngle((uint16_t)constrain(txVal, 0.0f, 65535.0f));
}

void SaratechOP::rotateYaw(float angleDeg) {
    _motion.yawOffsetDeg = normAngle(angleDeg);
    updateTXAXISCompensation();
}

void SaratechOP::resetYaw() {
    _motion.yawOffsetDeg = 0.0f;
    updateTXAXISCompensation();
}

// ── Shooter ──────────────────────────────────────────────────────────────────
void SaratechOP::shoot(float servoAngle_deg, uint8_t spinback) {
    if (!_initialized) return;

    servoAngle_deg = constrain(servoAngle_deg, _shooter.servoMinDeg,
                                _shooter.servoMaxDeg);
    spinback = constrain(spinback, (uint8_t)0, _shooter.spinbackMax);

    if (_servoIdx >= 0)
        _servo.setPosition(_servoIdx, servoAngle_deg);

    if (_shooter.spinbackIsPwm)
        analogWrite(SARATECH_SPINBACK_PIN, spinback);

    digitalWrite(SARATECH_KICKER_TRIGGER, HIGH);
    delay(100);
    digitalWrite(SARATECH_KICKER_TRIGGER, LOW);
}

void SaratechOP::setShooterServoAngle(float deg) {
    if (_servoIdx < 0) return;
    _servo.setPosition(_servoIdx, constrain(deg, _shooter.servoMinDeg,
                                             _shooter.servoMaxDeg));
}

void SaratechOP::setSpinbackSpeed(uint8_t speed) {
    if (!_shooter.spinbackIsPwm) return;
    analogWrite(SARATECH_SPINBACK_PIN, constrain(speed, 0, _shooter.spinbackMax));
}

// ── Shift (curved approach) ──────────────────────────────────────────────────
float SaratechOP::shift(float robotX_cm, float robotY_cm,
                        float ballX_cm, float ballY_cm,
                        float shiftCoeff) {
    if (!_initialized) return 0.0f;

    float dx = ballX_cm - robotX_cm;
    float dy = ballY_cm - robotY_cm;
    float baseAngle = degrees(atan2f(dx, dy));   // 0=forward, 90=right
    baseAngle = normAngle(baseAngle);

    // Add curvature based on shift coefficient and robot yaw
    float shiftAngle = baseAngle + shiftCoeff * 30.0f * sinf(_yaw * M_PI / 180.0f);
    return normAngle(shiftAngle);
}

// ── Sensor polling ───────────────────────────────────────────────────────────
void SaratechOP::updateSensors() {
    updateIMU();
    updateLDR();
    updateAnalogSensors();
    updateKicker();
    updateLiDAR();
}

void SaratechOP::updateIMU() {
    _yawRaw = _imu.read();
    _yaw = normAngle(_yawRaw + _motion.yawOffsetDeg);
}

void SaratechOP::updateLDR() {
    _ldrFront = ((analogRead(SARATECH_LDR_FRONT)  - _sensor.ldrBasis[0]) > _sensor.ldrThresh);
    _ldrRight = ((analogRead(SARATECH_LDR_RIGHT)  - _sensor.ldrBasis[1]) > _sensor.ldrThresh);
    _ldrBack  = ((analogRead(SARATECH_LDR_BACK)   - _sensor.ldrBasis[2]) > _sensor.ldrThresh);
    _ldrLeft  = ((analogRead(SARATECH_LDR_LEFT)   - _sensor.ldrBasis[3]) > _sensor.ldrThresh);
}

void SaratechOP::updateAnalogSensors() {
    _snsrBottom = analogRead(SARATECH_SNSR_BOTTOM);
    _snsrRight  = analogRead(SARATECH_SNSR_RIGHT);
    _snsrLeft   = analogRead(SARATECH_SNSR_LEFT);
}

void SaratechOP::updateKicker() {
    bool pinLow = !digitalRead(SARATECH_KICKER_DETECT);
    _ballInKicker = pinLow;
}

// ── Low-level motor control ──────────────────────────────────────────────────
bool SaratechOP::setMotorSpeeds(int16_t fl, int16_t fr, int16_t rr, int16_t rl) {
    return _gtd35.motor(fl, fr, rr, rl);
}

bool SaratechOP::setMotorSpeedsRPM(float fl_rpm, float fr_rpm,
                                    float rr_rpm, float rl_rpm) {
    float scale = 32767.0f / _motor.maxMotorRPM;
    int16_t fl = (int16_t)constrain(fl_rpm * scale, -32767.0f, 32767.0f);
    int16_t fr = (int16_t)constrain(fr_rpm * scale, -32767.0f, 32767.0f);
    int16_t rr = (int16_t)constrain(rr_rpm * scale, -32767.0f, 32767.0f);
    int16_t rl = (int16_t)constrain(rl_rpm * scale, -32767.0f, 32767.0f);
    return _gtd35.motor(fl, fr, rr, rl);
}

// ── Private helpers ──────────────────────────────────────────────────────────
void SaratechOP::computeWheelSpeeds(float vx, float vy, float omega,
                                     int16_t &fl, int16_t &fr,
                                     int16_t &rr, int16_t &rl)
{
    float r   = _motor.wheelRadius_cm;
    float wb  = _motor.wheelBase_cm;
    float tw  = _motor.track_cm;

    float vFL =  vy + vx - omega * (wb * 0.5f + tw * 0.5f);
    float vFR =  vy - vx - omega * (wb * 0.5f + tw * 0.5f);
    float vRR = -vy - vx - omega * (wb * 0.5f - tw * 0.5f);
    float vRL = -vy + vx - omega * (wb * 0.5f - tw * 0.5f);

    float cmsPerRPM = (2.0f * M_PI * r) / 60.0f;

    auto cmsToInt = [&](float cms) -> int16_t {
        float rpm = cms / cmsPerRPM;
        rpm = constrain(rpm, -_motor.maxMotorRPM, _motor.maxMotorRPM);
        return (int16_t)(rpm * (32767.0f / _motor.maxMotorRPM));
    };

    fl = cmsToInt(vFL);
    fr = cmsToInt(vFR);
    rr = cmsToInt(vRR);
    rl = cmsToInt(vRL);
}

void SaratechOP::applyTXAXISComp(int16_t &fl, int16_t &fr,
                                  int16_t &rr, int16_t &rl) {
    float corr = constrain(_yaw * ((_motion.tdAxisCoeff / 256.0f) * _motor.txAxisGain),
                           -90.0f, 90.0f);
    int16_t c = (int16_t)(corr * 300.0f);
    fl += c; fr += c; rr += c; rl += c;

    auto clamp16 = [](int16_t &v) {
        if (v > 32767) v = 32767;
        if (v < -32768) v = -32768;
    };
    clamp16(fl); clamp16(fr); clamp16(rr); clamp16(rl);
}

void SaratechOP::applyTDAXISInfluence(int16_t &fl, int16_t &fr,
                                       int16_t &rr, int16_t &rl) {
    // TDAXIS coefficient already applied in applyTXAXISComp
    (void)fl; (void)fr; (void)rr; (void)rl;
}

float SaratechOP::normAngle(float a) {
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

float SaratechOP::cmsToRPM(float cms) const {
    float circ = 2.0f * M_PI * _motor.wheelRadius_cm;
    return (cms / circ) * 60.0f;
}

float SaratechOP::rpmToCMS(float rpm) const {
    float circ = 2.0f * M_PI * _motor.wheelRadius_cm;
    return (rpm / 60.0f) * circ;
}

void SaratechOP::updateOdometry(float vx, float vy, float dt_s) {
    _posX += vx * dt_s;
    _posY += vy * dt_s;
}

// ── Debug ────────────────────────────────────────────────────────────────────
void SaratechOP::printDebug() {
    // Serial output
    Serial.print(F("yaw="));   Serial.print(_yaw, 2);
    Serial.print(F(" x="));    Serial.print(_posX, 1);
    Serial.print(F(" y="));    Serial.print(_posY, 1);
    Serial.print(F(" kf="));   Serial.print(_ldrFront);
    Serial.print(F(" kr="));   Serial.print(_ldrRight);
    Serial.print(F(" kb="));   Serial.print(_ldrBack);
    Serial.print(F(" kl="));   Serial.print(_ldrLeft);
    Serial.print(F(" kicker=")); Serial.println(_ballInKicker ? "Y" : "N");

    // OLED output
    _display.clearDisplay();
    _display.setCursor(0, 0);
    _display.setTextSize(1);
    _display.setTextColor(WHITE);

    _display.print("YAW:");
    _display.println(_yaw, 1);
    _display.print("X:");
    _display.print(_posX, 1);
    _display.print(" Y:");
    _display.println(_posY, 1);
    _display.print("KF:");
    _display.print(_ldrFront);
    _display.print(" KR:");
    _display.println(_ldrRight);
    _display.print("KB:");
    _display.print(_ldrBack);
    _display.print(" KL:");
    _display.println(_ldrLeft);
    _display.print("KICK:");
    _display.println(_ballInKicker ? "YES" : "NO");
    _display.print("LOC:");
    _display.println(_localized ? "OK" : "--");
    _display.display();
}
