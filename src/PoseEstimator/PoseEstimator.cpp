#include "PoseEstimator.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

PoseEstimator::PoseEstimator()
    : _localized(false), _confidence(0.0f),
      _prevYaw(0.0f), _yawRate(0.0f), _imuGain(0.7f),
      _lidarGain(0.3f), _featureMin(30.0f), _featureMax(500.0f),
      _matchThresh(0.5f) {
}

PoseEstimator::~PoseEstimator() {
}

void PoseEstimator::begin(float initX_cm, float initY_cm, float initTheta_deg) {
    _pose.x      = initX_cm;
    _pose.y      = initY_cm;
    _pose.theta  = initTheta_deg;
    _prevYaw     = initTheta_deg;
    _localized   = false;
    _confidence  = 0.0f;
    memset(&_prevScan, 0, sizeof(_prevScan));
}

void PoseEstimator::reset(float x_cm, float y_cm, float theta_deg) {
    _pose.x      = x_cm;
    _pose.y      = y_cm;
    _pose.theta  = theta_deg;
    _prevYaw     = theta_deg;
    _localized   = false;
    _confidence  = 0.0f;
    memset(&_prevScan, 0, sizeof(_prevScan));
}

void PoseEstimator::predictIMU(float yaw_deg, float dt_s) {
    // Compute yaw rate from IMU
    float dtheta = yaw_deg - _prevYaw;
    // Normalize angle difference
    while (dtheta > 180.0f) dtheta -= 360.0f;
    while (dtheta < -180.0f) dtheta += 360.0f;

    if (dt_s > 0.0f) {
        _yawRate = dtheta / dt_s;
    }

    // Update pose using IMU prediction (pure rotation)
    // Note: IMU gives absolute yaw, not rate. We use rate for integration.
    _prevYaw = yaw_deg;

    // For now, just store the IMU reading - actual pose update
    // happens when LiDAR data is available
}

bool PoseEstimator::updateLiDAR(const LiDARScan& scan) {
    // Need at least one previous scan to match
    if (_prevScan.count == 0) {
        _prevScan = scan;
        return false;
    }

    float dx_cm = 0.0f, dy_cm = 0.0f, dtheta_deg = 0.0f;

    if (matchScans(scan, _prevScan, dx_cm, dy_cm, dtheta_deg)) {
        // Convert mm to cm
        dx_cm /= 10.0f;
        dy_cm /= 10.0f;

        // Apply gain (fusion factor)
        dx_cm *= _lidarGain;
        dy_cm *= _lidarGain;
        dtheta_deg *= _lidarGain;

        // Update pose
        float rad = _pose.theta * M_PI / 180.0f;
        _pose.x +=  dx_cm * cosf(rad) - dy_cm * sinf(rad);
        _pose.y +=  dx_cm * sinf(rad) + dy_cm * cosf(rad);
        _pose.theta = _pose.theta + dtheta_deg;

        // Normalize theta
        while (_pose.theta > 180.0f) _pose.theta -= 360.0f;
        while (_pose.theta < -180.0f) _pose.theta += 360.0f;

        _localized = true;
        _confidence = 1.0f - (_matchThresh - scan.count / 360.0f);
        if (_confidence < 0.0f) _confidence = 0.0f;
        if (_confidence > 1.0f) _confidence = 1.0f;
    }

    // Store current scan as previous for next iteration
    _prevScan = scan;
    return _localized;
}

// Correlation-based scan matching
float PoseEstimator::correlateScans(const LiDARScan& curr, const LiDARScan& prev,
                                     float dx_cm, float dy_cm, float dtheta_deg) {
    float totalCorr = 0.0f;
    int   count = 0;

    float rad = dtheta_deg * M_PI / 180.0f;
    float cosT = cosf(rad);
    float sinT = sinf(rad);

    // Transform previous scan points and compare
    for (uint16_t i = 0; i < prev.count; i++) {
        if (!prev.angles[i] > 0.0f || prev.dists[i] == 0) continue;

        // Convert polar to cartesian (in cm)
        float px = (float)prev.dists[i] / 10.0f * sinf(prev.angles[i] * M_PI / 180.0f);
        float py = (float)prev.dists[i] / 10.0f * cosf(prev.angles[i] * M_PI / 180.0f);

        // Apply rotation and translation
        float rx = px * cosT - py * sinT + dx_cm;
        float ry = px * sinT + py * cosT + dy_cm;

        // Convert back to polar
        float rAngle = degrees(atan2f(rx, ry));
        float rDist = sqrtf(rx * rx + ry * ry) * 10.0f; // back to mm

        // Find nearest point in current scan
        int bestIdx = -1;
        float bestDiff = 999999.0f;

        // Search nearby angles (±30 degrees)
        int startIdx = (int)((rAngle - 30.0f) / 1.0f);
        int endIdx   = (int)((rAngle + 30.0f) / 1.0f);
        if (startIdx < 0) startIdx = 0;
        if (endIdx >= 360) endIdx = 359;

        for (int j = startIdx; j <= endIdx; j++) {
            if (curr.dists[j] > 0) {
                float diff = fabsf((float)curr.dists[j] - rDist);
                if (diff < bestDiff) {
                    bestDiff = diff;
                    bestIdx = j;
                }
            }
        }

        if (bestIdx >= 0 && bestDiff < 100.0f) { // 10cm tolerance
            totalCorr += (1.0f - bestDiff / 100.0f);
            count++;
        }
    }

    if (count == 0) return 0.0f;
    return totalCorr / (float)count;
}

bool PoseEstimator::matchScans(const LiDARScan& curr, const LiDARScan& prev,
                                float& dx_cm, float& dy_cm, float& dtheta_deg) {
    // Downsample to features for efficiency
    LiDARScan currFeat, prevFeat;
    uint16_t currCount, prevCount;

    downsampleScan(curr, currFeat, POSE_ESTIMATOR_MAX_FEATURES);
    downsampleScan(prev, prevFeat, POSE_ESTIMATOR_MAX_FEATURES);

    // Extract features (points with significant distance changes)
    extractFeatures(currFeat, NULL, NULL, NULL, currCount);
    extractFeatures(prevFeat, NULL, NULL, NULL, prevCount);

    if (currCount < 10 || prevCount < 10) return false;

    // Brute-force search for best translation and rotation
    float bestScore = -1.0f;
    dx_cm = 0.0f; dy_cm = 0.0f; dtheta_deg = 0.0f;

    // Try a few candidate positions
    float candidates[POSE_ESTIMATOR_MATCH_SCORES][3];
    int numCandidates = 0;

    // Generate candidates around estimated position
    for (int di = -2; di <= 2; di++) {
        for (int dj = -2; dj <= 2; dj++) {
            for (int dk = -1; dk <= 1; dk++) {
                if (numCandidates >= POSE_ESTIMATOR_MATCH_SCORES) break;
                candidates[numCandidates][0] = (float)di * 5.0f;  // dx in cm
                candidates[numCandidates][1] = (float)dj * 5.0f;  // dy in cm
                candidates[numCandidates][2] = (float)dk * 2.0f;  // dtheta in deg
                numCandidates++;
            }
        }
    }

    // Evaluate each candidate
    for (int i = 0; i < numCandidates; i++) {
        float score = correlateScans(curr, prev,
                                     candidates[i][0], candidates[i][1],
                                     candidates[i][2]);
        if (score > bestScore) {
            bestScore = score;
            dx_cm = candidates[i][0];
            dy_cm = candidates[i][1];
            dtheta_deg = candidates[i][2];
        }
    }

    // Accept if score is above threshold
    if (bestScore < _matchThresh) return false;

    return true;
}

void PoseEstimator::downsampleScan(const LiDARScan& src, LiDARScan& dst, uint16_t maxFeatures) {
    dst.count = 0;
    float step = 360.0f / (float)maxFeatures;

    for (uint16_t i = 0; i < 360 && dst.count < maxFeatures; i++) {
        if (src.dists[i] > 0) {
            dst.angles[dst.count] = src.angles[i];
            dst.dists[dst.count] = src.dists[i];
            dst.qualities[dst.count] = src.qualities[i];
            dst.count++;
        }
    }
}

bool PoseEstimator::extractFeatures(const LiDARScan& scan, float* featAngle,
                                     uint16_t* featDist, uint8_t* featQuality,
                                     uint16_t& count) {
    count = 0;
    for (uint16_t i = 1; i < scan.count - 1; i++) {
        // Look for distance discontinuities (edges)
        float d1 = (float)scan.dists[i-1] / 10.0f;
        float d2 = (float)scan.dists[i]   / 10.0f;
        float d3 = (float)scan.dists[i+1] / 10.0f;

        float change1 = fabsf(d2 - d1);
        float change2 = fabsf(d3 - d2);

        // Edge detection: significant distance change
        if ((change1 > _featureMin && change1 < _featureMax) ||
            (change2 > _featureMin && change2 < _featureMax)) {
            if (featAngle) featAngle[count] = scan.angles[i];
            if (featDist)  featDist[count]  = scan.dists[i];
            if (featQuality) featQuality[count] = scan.qualities[i];
            count++;
        }
    }
    return count > 0;
}

void PoseEstimator::printDebug() const {
    Serial.print(F("Pose: x="));
    Serial.print(_pose.x, 1);
    Serial.print(F(" y="));
    Serial.print(_pose.y, 1);
    Serial.print(F(" theta="));
    Serial.print(_pose.theta, 1);
    Serial.print(F(" localized="));
    Serial.print(_localized ? "YES" : "NO");
    Serial.print(F(" conf="));
    Serial.print(_confidence, 2);
    Serial.println();
}
