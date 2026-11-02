#pragma once
#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>

// Lightweight 2D pose estimator for STM32F103 (64KB flash, 20KB SRAM)
// Fuses IMU yaw rate with LiDAR scan matching
// State: [x, y, theta] in robot-local coordinates

#define POSE_ESTIMATOR_MAX_FEATURES  60   // Max features for scan matching
#define POSE_ESTIMATOR_MATCH_SCORES  30  // Number of candidate positions to evaluate

struct Pose2D {
    float x;       // Position X (cm), right-positive
    float y;       // Position Y (cm), forward-positive
    float theta;   // Heading angle (degrees), 0=forward, CCW positive
};

struct LiDARScan {
    float angles[360];   // Angles in degrees
    uint16_t dists[360]; // Distances in mm
    uint8_t  qualities[360]; // Signal quality
    uint16_t count;      // Number of valid points
    uint32_t timestamp;  // Capture timestamp in ms
};

class PoseEstimator {
public:
    PoseEstimator();
    ~PoseEstimator();

    // Initialize with starting pose
    void begin(float initX_cm, float initY_cm, float initTheta_deg);

    // Reset to known pose (e.g., when user places robot at known location)
    void reset(float x_cm, float y_cm, float theta_deg);

    // Update pose using IMU reading
    // yaw_deg: current IMU yaw reading (absolute, from TDAxis12)
    // dt_s: time since last update in seconds
    void predictIMU(float yaw_deg, float dt_s);

    // Update pose using LiDAR scan matching
    // currentScan: current LiDAR scan (360 points)
    // returns true if match was successful
    bool updateLiDAR(const LiDARScan& scan);

    // Get current pose estimate
    const Pose2D& getPose() const { return _pose; }
    float getPositionX() const { return _pose.x; }
    float getPositionY() const { return _pose.y; }
    float getHeading() const { return _pose.theta; }

    // Check if localization is valid
    bool isLocalized() const { return _localized; }
    float getConfidence() const { return _confidence; }

    // Reset localization
    void clear() { reset(0.0f, 0.0f, 0.0f); }

    // Configuration
    void setIMUGain(float gain) { _imuGain = constrain(gain, 0.0f, 2.0f); }
    void setLidarGain(float gain) { _lidarGain = constrain(gain, 0.0f, 2.0f); }
    void setFeatureDistanceMin(float cm) { _featureMin = cm; }
    void setFeatureDistanceMax(float cm) { _featureMax = cm; }
    void setMatchThreshold(float thresh) { _matchThresh = thresh; }

    // Get last processed scan
    const LiDARScan& getLastScan() const { return _prevScan; }

    // Debug
    void printDebug() const;

private:
    Pose2D _pose;
    bool   _localized;
    float  _confidence;     // 0.0 to 1.0

    // IMU state
    float _prevYaw;
    float _yawRate;
    float _imuGain;

    // LiDAR state
    LiDARScan _prevScan;
    float _lidarGain;
    float _featureMin;   // Minimum feature distance (cm)
    float _featureMax;   // Maximum feature distance (cm)
    float _matchThresh;  // Minimum correlation score to accept match

    // Scan matching
    bool matchScans(const LiDARScan& curr, const LiDARScan& prev,
                    float& dx_cm, float& dy_cm, float& dtheta_deg);
    float correlateScans(const LiDARScan& curr, const LiDARScan& prev,
                         float dx_cm, float dy_cm, float dtheta_deg);
    void downsampleScan(const LiDARScan& src, LiDARScan& dst, uint16_t maxFeatures);
    bool extractFeatures(const LiDARScan& scan, float* featAngle, uint16_t* featDist,
                         uint8_t* featQuality, uint16_t& count);
};
