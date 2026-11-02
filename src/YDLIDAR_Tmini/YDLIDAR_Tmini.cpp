#include "YDLIDAR_Tmini.h"
#include <string.h>
#include <math.h>

YDLIDAR_Tmini::YDLIDAR_Tmini(HardwareSerial &port)
    : _port(port),
      _rxHead(0), _rxTail(0), _rxCount(0),
      _pointCount(0), _scanComplete(false),
      _parseState(PARSE_WAIT_HEADER),
      _pktType(0), _pktLen(0), _pktIdx(0),
      _model(0), _firmwareVer(0), _hardwareVer(0),
      _health(0), _baudRate(230400), _scanFreqHz(5.0f) {
    memset(_rxBuf, 0, sizeof(_rxBuf));
    memset(_points, 0, sizeof(_points));
    memset(_pktBuf, 0, sizeof(_pktBuf));
}

YDLIDAR_Tmini::~YDLIDAR_Tmini() {
}

bool YDLIDAR_Tmini::begin(uint32_t baud) {
    _baudRate = baud;
    _port.begin(baud);
    delay(100);

    // Send stop command to ensure clean state
    _port.write(CMD_STOP, 2);
    delay(50);

    // Send start scan
    return startScan();
}

bool YDLIDAR_Tmini::startScan() {
    _port.write(CMD_START, 2);
    delay(10);
    // Reset parser state
    _parseState = PARSE_WAIT_HEADER;
    _rxHead = _rxTail = _rxCount = 0;
    _pointCount = 0;
    _scanComplete = false;
    memset(_points, 0, sizeof(_points));
    return true;
}

bool YDLIDAR_Tmini::stopScan() {
    _port.write(CMD_STOP, 2);
    _scanComplete = true;
    return true;
}

bool YDLIDAR_Tmini::restart() {
    stopScan();
    delay(100);
    return startScan();
}

void YDLIDAR_Tmini::tick() {
    // Read available bytes into circular buffer
    while (_port.available() > 0 && _rxCount < YDLIDAR_RX_BUF_SIZE) {
        uint8_t b = _port.read();
        _rxBuf[_rxTail] = b;
        _rxTail = (_rxTail + 1) % YDLIDAR_RX_BUF_SIZE;
        _rxCount++;
    }

    // Parse packets from buffer
    while (_rxCount >= 2 && parsePacket()) {
        // Packet consumed
    }
}

bool YDLIDAR_Tmini::parsePacket() {
    // Look for header 0xAA 0x55 in buffer
    while (_rxCount >= 2) {
        uint8_t b0 = _rxBuf[_rxHead];
        uint8_t b1 = _rxBuf[(_rxHead + 1) % YDLIDAR_RX_BUF_SIZE];

        if (b0 == 0xAA && b1 == 0x55) break;

        // Skip non-header byte
        _rxHead = (_rxHead + 1) % YDLIDAR_RX_BUF_SIZE;
        _rxCount--;
    }

    if (_rxCount < 2) return false;

    // Read header
    uint8_t h0 = _rxBuf[_rxHead];
    uint8_t h1 = _rxBuf[(_rxHead + 1) % YDLIDAR_RX_BUF_SIZE];
    if (h0 != 0xAA || h1 != 0x55) {
        _rxHead = (_rxHead + 1) % YDLIDAR_RX_BUF_SIZE;
        _rxCount--;
        return false;
    }

    // Need at least 8 bytes for header+body
    if (_rxCount < 8) return false;

    // Extract packet fields
    uint8_t  type     = _rxBuf[(_rxHead + 2) % YDLIDAR_RX_BUF_SIZE];
    uint8_t  speed    = _rxBuf[(_rxHead + 3) % YDLIDAR_RX_BUF_SIZE];
    uint16_t count    = _rxBuf[(_rxHead + 4) % YDLIDAR_RX_BUF_SIZE]
                      | ((_rxBuf[(_rxHead + 5) % YDLIDAR_RX_BUF_SIZE]) << 8);
    uint16_t startAng = _rxBuf[(_rxHead + 6) % YDLIDAR_RX_BUF_SIZE]
                      | ((_rxBuf[(_rxHead + 7) % YDLIDAR_RX_BUF_SIZE]) << 8);
    uint16_t endAng   = _rxBuf[(_rxHead + 8) % YDLIDAR_RX_BUF_SIZE]
                      | ((_rxBuf[(_rxHead + 9) % YDLIDAR_RX_BUF_SIZE]) << 8);
    uint16_t cksum    = _rxBuf[(_rxHead + 10) % YDLIDAR_RX_BUF_SIZE]
                      | ((_rxBuf[(_rxHead + 11) % YDLIDAR_RX_BUF_SIZE]) << 8);

    // Total packet size: 12 header bytes + count*3 data bytes
    uint16_t totalSize = 12 + (count * 3);

    if (_rxCount < totalSize) return false;

    // Validate checksum (XOR of bytes 2 to totalSize-2)
    uint16_t calcCksum = 0;
    for (uint16_t i = 2; i < totalSize - 2; i++) {
        calcCksum ^= _rxBuf[(_rxHead + i) % YDLIDAR_RX_BUF_SIZE];
    }

    // Consume packet from buffer
    _rxHead = (_rxHead + totalSize) % YDLIDAR_RX_BUF_SIZE;
    _rxCount -= totalSize;

    // Handle different packet types
    if (type == YDLIDAR_PKT_TYPE_SCAN || type == YDLIDAR_PKT_TYPE_TINY) {
        return handleScanPacket(count, startAng, endAng, cksum, calcCksum);
    } else if (type == 0x06) {
        return handleInfoPacket();
    } else if (type == 0x07) {
        return handleHealthPacket();
    }

    return true;
}

bool YDLIDAR_Tmini::handleScanPacket(uint16_t count, uint16_t startAng, uint16_t endAng,
                                      uint16_t cksum, uint16_t calcCksum) {
    // Calculate angles in degrees
    float startAngleDeg = (float)startAng / 64.0f;
    float endAngleDeg   = (float)endAng / 64.0f;
    float angleDiff     = endAngleDeg - startAngleDeg;
    if (angleDiff < 0) angleDiff += 360.0f;

    // Read sample data
    for (uint16_t i = 0; i < count; i++) {
        uint16_t bufIdx = (_rxHead + 12 + i * 3) % YDLIDAR_RX_BUF_SIZE;
        uint8_t  intensity = _rxBuf[bufIdx];
        uint8_t  s2        = _rxBuf[(bufIdx + 1) % YDLIDAR_RX_BUF_SIZE];
        uint8_t  s3        = _rxBuf[(bufIdx + 2) % YDLIDAR_RX_BUF_SIZE];

        // T-mini Plus distance formula
        uint16_t distance = (uint16_t(s3) << 6) | (s2 >> 2);

        // Interpolated angle
        float angle = startAngleDeg;
        if (count > 1) {
            angle = startAngleDeg + angleDiff * (float)i / (float)(count - 1);
        }
        while (angle >= 360.0f) angle -= 360.0f;
        while (angle < 0.0f)    angle += 360.0f;

        // Store point
        uint16_t idx = (uint16_t)angle;
        if (idx < 360 && distance > 0) {
            _points[idx].angle_deg   = angle;
            _points[idx].dist_mm     = distance;
            _points[idx].quality     = intensity;
            _points[idx].valid       = true;
        }
        _pointCount++;
    }

    // Check if we have a complete 360 scan
    if (_pointCount >= 360) {
        _scanComplete = true;
        _pointCount = 360;
    }

    return true;
}

bool YDLIDAR_Tmini::handleInfoPacket() {
    // Device info: model, firmware, hardware, serial, baud, etc.
    // Simplified parsing
    _model      = _pktBuf[0];
    _firmwareVer = (_pktBuf[1] << 8) | _pktBuf[2];
    _hardwareVer = (_pktBuf[3] << 8) | _pktBuf[4];
    return true;
}

bool YDLIDAR_Tmini::handleHealthPacket() {
    // Device health status
    _health = _pktBuf[0];
    return true;
}

bool YDLIDAR_Tmini::setScanFrequency(float freq_hz) {
    // T-mini Plus: A5 09 (increment) / A5 0A (decrement) / A5 0B (inc 1Hz) / A5 0C (dec 1Hz) / A5 0D (read)
    // For now, just send a basic command
    // Actual frequency control may vary by firmware version
    (void)freq_hz;
    return true;
}

bool YDLIDAR_Tmini::readDeviceParams() {
    _port.write(CMD_INFO, 2);
    delay(50);
    _port.write(CMD_HEALTH, 2);
    delay(50);
    return true;
}

const YDLidarPoint& YDLIDAR_Tmini::getPoint(float angle_idx) const {
    static YDLidarPoint empty = {0, 0, 0, false};
    uint16_t idx = (uint16_t)angle_idx;
    if (idx >= 360) return empty;
    return _points[idx];
}

void YDLIDAR_Tmini::printDebug() const {
    Serial.print(F("YDLIDAR: points="));
    Serial.print(_pointCount);
    Serial.print(F(" complete="));
    Serial.print(_scanComplete ? "YES" : "NO");
    Serial.print(F(" health="));
    Serial.print((int)_health);
    Serial.print(F(" model="));
    Serial.print((int)_model);
    Serial.println();
}
