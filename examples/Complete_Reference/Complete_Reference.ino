// =============================================
// مرجع کامل کتابخانه saratech_op
// Complete saratech_op Library Reference
//
// تمامی توابع و دستورات عمومی کتابخانه در این مثال نمایش داده شده‌اند.
// Every public function and command of the library is demonstrated here.
// =============================================

#include <saratech_op.h>

void setup() {
    // ─── بخش ۱: راه‌اندازی / Initialization ────────────────────────────────────

    // روش ساده: استفاده از تنظیمات پیش‌فرض
    // Simple method: use default settings
    saratech.begin();

    // روش پیشرفته: ارسال تنظیمات سفارشی
    // Advanced method: pass custom configuration structs
    SaratechOP::MotorConfig   motorCfg;
    SaratechOP::SensorConfig  sensorCfg;
    SaratechOP::ShooterConfig shooterCfg;
    SaratechOP::MotionConfig  motionCfg;

    // ── MotorConfig ────────────────────────────────────────────────────────────
    motorCfg.serial     = &Serial1;   // پورت UART موتور / Motor UART port
    motorCfg.baud       = 115200;     // نرخ baud / Baud rate
    motorCfg.dirPin     = -1;          // پین جهت RS485 (-1 = بدون پین) / RS485 DE pin
    motorCfg.idFL       = 3;           // ID موتور جلو-چپ / Front-left motor ID
    motorCfg.idFR       = 0;           // ID موتور جلو-راست / Front-right motor ID
    motorCfg.idRR       = 1;           // ID موتور عقب-راست / Rear-right motor ID
    motorCfg.idRL       = 2;           // ID موتور عقب-چپ / Rear-left motor ID
    motorCfg.wheelRadius_cm = 2.5f;   // شعاع چرخ (cm) / Wheel radius (cm) — قطر 5cm تأیید شده
    motorCfg.wheelBase_cm   = 16.0f;  // فاصله محور جلو-عقب (cm) / Front-rear wheelbase
    motorCfg.track_cm       = 16.0f;  // فاصله محور چپ-راست (cm) / Left-right track width
    motorCfg.maxSpeed_cms   = 200.0f; // حداکثر سرعت خطی (cm/s) / Max linear speed
    motorCfg.maxAccel_cms2  = 100.0f; // حداکثر شتاب (cm/s²) / Max acceleration
    motorCfg.maxMotorRPM    = 1000.0f;// حداکثر دور موتور (RPM) / Max motor RPM
    motorCfg.txAxisDefault  = 4000;   // زاویه پیش‌فرض TXAXIS / Default TXAXIS angle
    motorCfg.txAxisGain     = 1.0f;   // ضریب تقویت TXAXIS / TXAXIS gain

    // ── SensorConfig ───────────────────────────────────────────────────────────
    sensorCfg.i2c       = &Wire2;     // I2C برای IMU / I2C for IMU
    sensorCfg.imuAddr   = 0x10;       // آدرس I2C IMU / IMU I2C address
    sensorCfg.ldrThresh = 300;        // آستانه حساسیت LDR / LDR sensitivity threshold

    // ── ShooterConfig ──────────────────────────────────────────────────────────
    shooterCfg.triggerPin     = PC13;   // پین تریگر شوتر / Shooter trigger pin
    shooterCfg.servoPin       = PA15;   // پین سروو شوتر / Shooter servo pin
    shooterCfg.spinbackPin    = PB5;    // پین اسپین‌بک / Spinback motor pin
    shooterCfg.spinbackIsPwm  = true;   // PWM برای اسپین‌بک / Use PWM for spinback
    shooterCfg.servoMinDeg    = 0.0f;   // حداقل زاویه سروو (درجه) / Min servo angle (deg)
    shooterCfg.servoMaxDeg    = 180.0f; // حداکثر زاویه سروو (درجه) / Max servo angle (deg)
    shooterCfg.servoMinUs     = 500;    // حداقل پالس سروو (میکروثانیه) / Min servo pulse (us)
    shooterCfg.servoMaxUs     = 2400;   // حداکثر پالس سروو (میکروثانیه) / Max servo pulse (us)
    shooterCfg.spinbackMax    = 255;    // حداکثر سرعت اسپین‌بک / Max spinback speed

    // ── MotionConfig ───────────────────────────────────────────────────────────
    motionCfg.tdAxisCoeff  = 200;      // ضریب TDAXIS (0-512) / TDAXIS coefficient (0-512)
    motionCfg.yawOffsetDeg = 0.0f;     // offset عمودی اولیه / Initial yaw offset
    motionCfg.posTol_cm    = 5.0f;     // تلورانس رسیدن به هدف (cm) / Position tolerance (cm)
    motionCfg.angTol_deg   = 5.0f;     // تلورانس زاویه هدف (درجه) / Angle tolerance (deg)
    motionCfg.shiftCoeff   = 1.0f;     // ضریب پیش‌روی منحنی / Curved approach coefficient

    saratech.begin(motorCfg, sensorCfg, shooterCfg, motionCfg);

    // ═══════════════════════════════════════════════════════════════════════
    // بخش ۲: تنظیم پارامترها / Parameter Settings
    // ═══════════════════════════════════════════════════════════════════════

    // ── هندسه چرخ‌ها / Wheel Geometry ──────────────────────────────────────────
    saratech.setWheelDiameter(5.0);     // قطر چرخ (cm) / Wheel diameter (cm)
    saratech.setWheelRadius(2.5);       // شعاع چرخ (cm) / Wheel radius (cm)
    saratech.setWheelBase(16.0);        // فاصله جلو-عقب (cm) / Front-rear wheelbase
    saratech.setTrackWidth(16.0);       // فاصله چپ-راست (cm) / Left-right track width

    // ── محدودیت‌های حرکت / Motion Limits ──────────────────────────────────────
    saratech.setMaxSpeed(200.0);        // حداکثر سرعت خطی (cm/s) / Max linear speed
    saratech.setMaxAccel(100.0);        // حداکثر شتاب (cm/s²) / Max acceleration
    saratech.setMaxMotorRPM(1000.0);    // حداکثر دور موتور (RPM) / Max motor RPM

    // ── TDAXIS / TXAXIS ────────────────────────────────────────────────────────
    saratech.setTDAxisCoefficient(200); // ضریب TDAXIS (0-512) / TDAXIS coefficient
    saratech.setTDAxis(200);            // روش کوتاه‌تر / Shorter alias
    saratech.setTXAxisCoefficient(200); // روش دیگر با نام TXAXIS / Another alias
    saratech.setTXAxisGain(1.0f);       // ضریب تقویت جبران‌سازی / Gain
    saratech.setTXAxisAngle(4000);      // زاویه خام ارسالی به GTD35 / Raw TXAXIS angle

    // ── سنسورها / Sensors ─────────────────────────────────────────────────────
    saratech.setLDRThreshold(300);      // آستانه فعال‌سازی LDR / LDR trigger threshold
    saratech.setLDRBaseline(0, 0);      // کالیبراسیون LDR جلو / Front LDR baseline
    saratech.setLDRBaseline(1, 0);      // کالیبراسیون LDR راست / Right LDR baseline
    saratech.setLDRBaseline(2, 0);      // کالیبراسیون LDR عقب / Back LDR baseline
    saratech.setLDRBaseline(3, 0);      // کالیبراسیون LDR چپ / Left LDR baseline
    saratech.setServoLimits(500, 2400, 0.0, 180.0); // محدودیت‌های سروو / Servo limits
    saratech.setSpinbackMaxSpeed(255);  // حداکثر سرعت اسپین‌بک / Max spinback
    saratech.setPositionTolerance(5.0); // تلورانس موقعیت هدف (cm) / Position tolerance
    saratech.setAngleTolerance(5.0);    // تلورانس زاویه هدف (deg) / Angle tolerance
    saratech.setShiftCoefficient(1.0);  // ضریب منحنی / Shift coefficient

    // ═══════════════════════════════════════════════════════════════════════
    // بخش ۳: توابع حرکتی اصلی / Main Movement Functions
    // ═══════════════════════════════════════════════════════════════════════

    // ── Function 1: حرکت با واحدهای فیزیکی ─────────────────────────────────────
    // Move with physical units (cm/s, cm/s², degrees)
    saratech.move(100.0, 50.0, 0.0);    // جلو / Forward at 100 cm/s
    saratech.move(-80.0, 40.0, 180.0);  // عقب / Backward at 80 cm/s
    saratech.move(60.0, 30.0, 90.0);    // راست / Right at 60 cm/s
    saratech.move(60.0, 30.0, -90.0);   // چپ / Left at 60 cm/s
    saratech.move(50.0, 25.0, 45.0);    // قطری / Diagonal
    saratech.stopMotors();              // توقف / Stop

    // ── Function 2: حرکت مختصاتی ───────────────────────────────────────────────
    // Coordinate-based movement toward a target (cm)
    bool reached = saratech.goTo(50.0, 30.0, 100.0);  // حرکت به هدف / Move to target
    if (reached) {
        saratech.stopMotors();  // توقف وقتی به هدف رسید / Stop when reached
    }

    // ═══════════════════════════════════════════════════════════════════════
    // بخش ۴: جبران‌سازی و عمودی / Compensation & Yaw
    // ═══════════════════════════════════════════════════════════════════════

    // ── Function 3: جبران‌سازی TXAXIS ─────────────────────────────────────────
    // Update TXAXIS: reads IMU and applies gyro compensation
    saratech.updateTXAXISCompensation();

    // ── Function 4A: کالیبراسیون TDAXIS ───────────────────────────────────────
    // Set TDAXIS coefficient (scales gyro correction strength)
    saratech.setTDAxis(200);           // ضریب 200 (0-512) / Coefficient 200

    // ── Function 4B: چرخش عمدی عمودی ─────────────────────────────────────────
    // Intentional yaw rotation
    saratech.rotateYaw(5.0);           // 5 درجه به راست / 5° right turn
    saratech.rotateYaw(-10.0);         // 10 درجه به چپ / 10° left turn
    saratech.resetYaw();               // بازگشت به مرجع اصلی / Reset to reference

    // ═══════════════════════════════════════════════════════════════════════
    // بخش ۵: شوتر / Shooter
    // ═══════════════════════════════════════════════════════════════════════

    // ── Function 4C: شوتر ─────────────────────────────────────────────────────
    // Shoot with servo angle and spinback speed
    saratech.shoot(90.0, 200);         // شوت مستقیم / Straight shot
    saratech.shoot(45.0, 150);         // شوت کج / Angled shot
    saratech.shoot(0.0, 0);            // بستن شوتر / Close shooter
    saratech.shoot(180.0, 255);        // شوت باز / Full open shot

    // کنترل جداگانه سروو و اسپین‌بک
    // Separate servo and spinback control
    saratech.setShooterServoAngle(90.0);  // تنظیم زاویه سروو / Set servo angle
    saratech.setSpinbackSpeed(128);       // تنظیم سرعت اسپین‌بک / Set spinback speed

    // ═══════════════════════════════════════════════════════════════════════
    // بخش ۶: پیش‌روی منحنی / Curved Approach (Shift)
    // ═══════════════════════════════════════════════════════════════════════

    // ── Function 5: Shift ──────────────────────────────────────────────────────
    // Calculate curved approach heading angle
    float shiftResult = saratech.shift(
        saratech.getRobotX(),     // موقعیت X فعلی ربات (cm) / Current robot X (cm)
        saratech.getRobotY(),     // موقعیت Y فعلی ربات (cm) / Current robot Y (cm)
        100.0,                    // موقعیت X توپ (cm) / Ball X position (cm)
        50.0,                     // موقعیت Y توپ (cm) / Ball Y position (cm)
        1.0                       // ضریb شیفت / Shift coefficient
    );
    saratech.move(100.0, 50.0, shiftResult);

    // ═══════════════════════════════════════════════════════════════════════
    // بخش ۷: خواندن سنسورها / Reading Sensors
    // ═══════════════════════════════════════════════════════════════════════

    // بروزرسانی همه سنسورها
    // Update all sensors
    saratech.updateSensors();

    // هر سنسور به‌طور جداگانه
    // Each sensor individually
    saratech.updateIMU();         // خواندن ژیروسکوپ / Read gyroscope yaw
    saratech.updateLDR();         // خواندن ۴ جهت نوری / Read 4-direction LDRs
    saratech.updateAnalogSensors(); // خواندن سنسورهای آنالوگ / Read analog sensors
    saratech.updateKicker();      // بررسی وضعیت شوتر / Check kicker state
    saratech.updateLiDAR();       // خواندن لیدار (اگر متصل باشد) / Read LiDAR (if present)

    // دریافت مقادیر خوانده‌شده
    // Get read values
    float robotYaw      = saratech.getRobotYaw();     // زاویه عمودی (درجه) / Robot yaw (deg)
    float robotYawRaw   = saratech.getRobotYawRaw();  // زاویه خام ژیروسکوپ / Raw gyro angle
    float robotX        = saratech.getRobotX();       // موقعیت X ربات (cm) / Robot X position
    float robotY        = saratech.getRobotY();       // موقعیت Y ربات (cm) / Robot Y position
    bool  inKicker      = saratech.isBallInKicker();  // آیا توپ در شوتر است؟ / Is ball in kicker?
    bool  isLoc         = saratech.isLocalizationValid(); // آیا موقعیت‌یابی معتبر است؟ / Is localization valid?
    uint16_t tdAxisVal  = saratech.getTDAxisCoeff();  // ضریb فعلی TDAXIS / Current TDAXIS coeff
    float  yawOff       = saratech.getYawOffset();    // offset عمودی فعلی / Current yaw offset
    float  maxSpd       = saratech.getMaxSpeed();     // حداکثر سرعت تنظیم‌شده / Max configured speed
    float  maxAcc       = saratech.getMaxAccel();     // حداکثر شتاب تنظیم‌شده / Max configured acceleration
    float  wRad         = saratech.getWheelRadius();  // شعاع چرخ فعلی (cm) / Current wheel radius
    float  wBase        = saratech.getWheelBase();    // فاصله محور چرخ‌ها (cm) / Wheelbase distance
    float  trackW       = saratech.getTrackWidth();   // عرض محور چرخ‌ها (cm) / Track width
    float  locConf      = saratech.getLocalizationConfidence(); // اعتماد موقعیت‌یابی / Localization confidence

    // وضعیت LDRها
    // LDR status
    bool ldrFront = saratech.getLDRFront();
    bool ldrRight = saratech.getLDRRight();
    bool ldrBack  = saratech.getLDRBack();
    bool ldrLeft  = saratech.getLDRLeft();

    // مقادیر سنسورهای آنالوگ
    // Analog sensor values
    int bottomSensor = saratech.getBottomSensor();
    int rightSensor  = saratech.getRightSensor();
    int leftSensor   = saratech.getLeftSensor();

    // ═══════════════════════════════════════════════════════════════════════
    // بخش ۸: کنترل مستقیم موتورها / Direct Motor Control
    // ═══════════════════════════════════════════════════════════════════════

    // کنترل با مقادیر مستقیم (signed int16)
    // Direct control with signed int16 values
    saratech.setMotorSpeeds(1000, 1000, 1000, 1000);  // FL, FR, RR, RL
    saratech.setMotorSpeeds(-500, -500, -500, -500);  // معکوس / Reverse
    saratech.setMotorSpeeds(0, 0, 0, 0);              // توقف / Stop

    // کنترل بر حسب RPM
    // Control by RPM
    saratech.setMotorSpeedsRPM(50.0, 50.0, 50.0, 50.0);  // 50 RPM همه موتورها
    saratech.setMotorSpeedsRPM(0.0, 0.0, 0.0, 0.0);      // توقف

    // ═══════════════════════════════════════════════════════════════════════
    // بخش ۹: دسترسی به سخت‌افزار / Hardware Access
    // ═══════════════════════════════════════════════════════════════════════

    // دسترسی مستقیم به GTD35
    // Direct GTD35 access
    GTD35& gtd = saratech.gtd35();
    gtd.motor(100, 100, 100, 100);        // ارسال فرمان مستقیم / Send direct command
    gtd.setEnabled(false);                // غیرفعال کردن موتور / Disable motor
    gtd.setEnabled(true);                 // فعال‌سازی موتور / Enable motor
    gtd.setAngle(4000);                   // تنظیم TXAXIS مستقیم / Set TXAXIS directly
    gtd.setMotorIDs(3, 0, 1, 2);          // تغییر IDs موتورها / Change motor IDs

    // دسترسی مستقیم به IMU
    // Direct IMU access
    TDAxis12& imuDev = saratech.imu();
    float gyroVal = imuDev.read();        // خواندن زاویه / Read angle

    // دسترسی مستقیم به کنترل‌کننده سروو
    // Direct servo controller access
    STM32_ISR_Servo& srv = saratech.servoCtrl();
    srv.setPosition(0, 90.0);             // تنظیم سروو 0 روی 90 درجه
    int pos = srv.getPosition(0);         // خواندن موقعیت سروو

    // دسترسی مستقیم به OLED (SH1106)
    // Direct OLED access (SH1106)
    Adafruit_SH1106& disp = saratech.display();
    disp.clearDisplay();                  // پاک کردن صفحه / Clear display
    disp.setCursor(0, 0);                 // تنظیم مکان‌نما / Set cursor position
    disp.setTextSize(1);                  // اندازه متن / Text size
    disp.setTextColor(WHITE);             // رنگ متن / Text color
    disp.print("Saratech OP");            // چاپ متن / Print text
    disp.println("v1.0");                 // چاپ با خط جدید / Print with newline
    disp.drawCircle(64, 32, 20, WHITE);   // رسم دایره / Draw circle
    disp.drawLine(0, 0, 127, 63, WHITE); // رسم خط / Draw line
    disp.display();                       // بروزرسانی فیزیکی صفحه / Actually update display

    // ═══════════════════════════════════════════════════════════════════════
    // بخش ۱۰: لیدار (اختیاری) / LiDAR (Optional)
    // ═══════════════════════════════════════════════════════════════════════

    // دسترسی مستقیم به YDLIDAR
    // Direct YDLIDAR access
    if (saratech.lidar().getModel() > 0) {  // فقط اگر لیدار متصل باشد
        // فقط اگر لیدار متصل باشد / Only if LiDAR is present
        saratech.startLiDAR();              // شروع اسکن / Start scanning
        saratech.stopLiDAR();               // توقف اسکن / Stop scanning

        // خواندن اطلاعات لیدار
        // Read LiDAR info
        uint8_t health = saratech.getLidarHealth();  // سلامت دستگاه / Device health
        float scanFreq = saratech.lidar().getScanFreqHz();  // فرکانس اسکن / Scan frequency
    }

    // ═══════════════════════════════════════════════════════════════════════
    // بخش ۱۱: موقعیت‌یابی / Localization
    // ═══════════════════════════════════════════════════════════════════════

    // تنظیم موقعیت اولیه (وقتی ربات در مکان مشخصی قرار دارد)
    // Set initial pose (when robot is placed at known location)
    saratech.setInitialPose(0.0f, 0.0f, 0.0f);  // (x_cm, y_cm, theta_deg)

    // به‌روزرسانی موقعیت‌یابی (ترکیب IMU + LiDAR اگر موجود باشد)
    // Update localization (fuses IMU + LiDAR if available)
    bool localized = saratech.updateLocalization();

    // اگر موقعیت‌یابی معتبر است، از مختصات فازی استفاده کن
    // If localization is valid, use fused coordinates
    if (localized) {
        float fx = saratech.getRobotX();
        float fy = saratech.getRobotY();
        float fh = saratech.getHeading();
    }

    // تنظیم پارامترهای موقعیت‌یابی
    // Configure localization parameters
    saratech.setIMUGain(0.7f);                    // اعتماد به IMU (0-2) / IMU trust
    saratech.setLidarGain(0.3f);                  // اعتماد به LiDAR (0-2) / LiDAR trust
    saratech.setFeatureDistanceRange(30.0f, 500.0f); // محدوده ویژگی‌ها (cm) / Feature distance range
    saratech.setMatchThreshold(0.5f);             // آستانه تطبیق / Match threshold

    // بازنشانی موقعیت‌یابی به مبدأ
    // Reset localization to origin
    saratech.resetLocalization();

    // ═══════════════════════════════════════════════════════════════════════
    // بخش ۱۲: نمایش و دیباگ / Display & Debug
    // ═══════════════════════════════════════════════════════════════════════

    // چاپ اطلاعات روی Serial و OLED
    // Print debug info to both Serial and OLED
    saratech.printDebug();
}

void loop() {
    // ─── حلقه اصلی: خواندن سنسور + حرکت خودکار ────────────────────────────────
    // Main loop: sensor reading + autonomous movement

    // بروزرسانی سنسورها (شامل IMU, LDR, LiDAR و ...)
    // Update all sensors (IMU, LDR, LiDAR, etc.)
    saratech.updateSensors();

    // حرکت بر اساس LDR
    // Move based on LDR inputs
    if (saratech.getLDRFront()) {
        saratech.move(80.0, 40.0, 0.0);    // جلو / Forward
    } else if (saratech.getLDRRight()) {
        saratech.move(80.0, 40.0, 90.0);   // راست / Right
    } else if (saratech.getLDRLeft()) {
        saratech.move(80.0, 40.0, -90.0);  // چپ / Left
    } else if (saratech.getLDRBack()) {
        saratech.move(80.0, 40.0, 180.0);  // عقب / Backward
    } else {
        saratech.stopMotors();             // توقف / Stop
    }

    // به‌روزرسانی جبران‌سازی
    // Update compensation
    saratech.updateTXAXISCompensation();

    // به‌روزرسانی موقعیت‌یابی
    // Update localization (IMU + LiDAR fusion)
    saratech.updateLocalization();

    // نمایش اطلاعات
    // Display debug info
    saratech.printDebug();
}
