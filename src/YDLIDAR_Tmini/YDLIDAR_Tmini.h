#pragma once
#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>

// T-mini Plus protocol constants
#define YDLIDAR_PACKET_HEADER     0x55AA   // Packet header (little-endian: 0xAA, 0x55)
#define YDLIDAR_PKT_TYPE_SCAN     0x01
#define YDLIDAR_PKT_TYPE_QSYNC    0x5A
#define YDLIDAR_PKT_TYPE_ICSYNC   0x5A
#define YDLIDAR_PKT_TYPE_TINY     0x02

// T-mini Plus command bytes
#define YDLIDAR_CMD_STOP          0x00
#define YDLIDAR_CMD_SCAN          0x01
#define YDLIDAR_CMD_FORCE_SCAN    0x02
#define YDLIDAR_CMD_RST           0x05

// Command packets
static const uint8_t CMD_STOP[]     = {0xA5, 0x60};
static const uint8_t CMD_START[]    = {0xA5, 0x61};
static const uint8_t CMD_INFO[]     = {0xA5, 0x90};
static const uint8_t CMD_HEALTH[]   = {0xA5, 0x92};

// Scan packet structure (CT packet):
// Header(2) + Type(1) + Speed(1) + Count(2) + StartAngle(2) + EndAngle(2) + Checksum(2) + Data(N*3)
#define YDLIDAR_HEADER_SIZE   8   // 2 header + 1 type + 1 speed + 2 count + 2 angle + 2 checksum (before data)
#define YDLIDAR_SAMPLE_SIZE   3   // intensity(1) + dist_low(1) + dist_high(1)

// Circular buffer for UART receive
#define YDLIDAR_RX_BUF_SIZE   512

struct YDLidarPoint {
    float angle_deg;   // 0-360 degrees
    uint16_t dist_mm;  // distance in mm
    uint8_t  quality;  // signal quality 0-255
    bool     valid;
};

class YDLIDAR_Tmini {
public:
    YDLIDAR_Tmini(HardwareSerial &port = Serial2);
    ~YDLIDAR_Tmini();

    // Initialize the LiDAR
    bool begin(uint32_t baud = 230400);

    // Start/stop scanning
    bool startScan();
    bool stopScan();
    bool restart();

    // Call frequently from loop() — non-blocking
    void tick();

    // Get latest complete scan (360 points)
    const YDLidarPoint& getPoint(float angle_idx) const;
    uint16_t getPointCount() const { return _pointCount; }
    bool isScanComplete() const { return _scanComplete; }

    // Device info
    uint8_t getModel() const { return _model; }
    uint16_t getFirmwareVer() const { return _firmwareVer; }
    uint8_t getHardwareVer() const { return _hardwareVer; }
    uint8_t getHealth() const { return _health; }
    uint32_t getBaudRate() const { return _baudRate; }
    float getScanFreqHz() const { return _scanFreqHz; }

    // Configuration
    bool setScanFrequency(float freq_hz);
    bool readDeviceParams();

    // Debug
    void printDebug() const;

private:
    HardwareSerial &_port;

    // Receive buffer
    uint8_t  _rxBuf[YDLIDAR_RX_BUF_SIZE];
    uint16_t _rxHead;
    uint16_t _rxTail;
    uint16_t _rxCount;

    // Current scan data
    YDLidarPoint _points[360];
    uint16_t     _pointCount;
    bool         _scanComplete;

    // Packet parsing state
    enum ParseState : uint8_t {
        PARSE_WAIT_HEADER,
        PARSE_HEADER_1,
        PARSE_HEADER_2,
        PARSE_HEADER_3,
        PARSE_BODY,
        PARSE_DATA
    };
    ParseState _parseState;
    uint8_t    _pktType;
    uint16_t   _pktLen;
    uint16_t   _pktIdx;
    uint8_t    _pktBuf[256];  // Max packet size for T-mini Plus

    // Device info
    uint8_t  _model;
    uint16_t _firmwareVer;
    uint8_t  _hardwareVer;
    uint8_t  _health;
    uint32_t _baudRate;
    float    _scanFreqHz;

    // Parsing helpers
    void pushByte(uint8_t b);
    bool parsePacket();
    bool handleScanPacket();
    bool handleInfoPacket();
    bool handleHealthPacket();
    bool parseToPoint(uint8_t i, uint8_t s2, uint8_t s3, float startAngle, float endAngle, uint16_t count);
    uint16_t crc16(const uint8_t *data, uint16_t len);
};
