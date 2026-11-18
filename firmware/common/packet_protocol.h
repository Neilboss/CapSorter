// =========================================================================
// packet_protocol.h - Shared Inter-Controller Wire Definitions (V1 & V2)
// Ensures Type Safety, CRC16-CCITT Checking & Atomic Batch Execution
// Python Packing Format:
//   BatchHeader : struct.pack('<BHBH', magic, batchId, totalCaps, crc16) [6B]
//   CapTarget   : struct.pack('<BBHHffB', id, color, pxX, pxY, x_mm, y_mm, bin) [15B]
// =========================================================================
#ifndef PACKET_PROTOCOL_H
#define PACKET_PROTOCOL_H

#include <stdint.h>

// Frame & Batch Magic Bytes
#define FRAME_SYNC_BYTE_1     0x55
#define FRAME_SYNC_BYTE_2     0xAA
#define PROTOCOL_MAGIC_BYTE   0x5A

// Inter-Controller Command & Handshake Opcodes
#define CMD_CAPTURE_FRAME     0xAA  // Teensy -> ESP32-CAM: Trigger QVGA capture
#define CMD_BATCH_ACK         0xA1  // Mega -> Vision Host: Batch CRC valid, starting motion
#define CMD_BATCH_NACK        0xA2  // Mega -> Vision Host: CRC mismatch, retransmit
#define CMD_BATCH_COMPLETE    0xA3  // Mega -> Vision Host: All caps sorted & parked at (0,0)
#define CMD_HEARTBEAT_PING    0xF0  // Vision Host -> Mega: 250ms keep-alive pulse
#define CMD_RECOVERY_HALT     0xEE  // Mega -> Vision Host: Comms-loss fail-safe triggered

enum CapColor : uint8_t {
  COLOR_NONE   = 0x00,
  COLOR_RED    = 0x01,
  COLOR_YELLOW = 0x02,
  COLOR_GREEN  = 0x03,
  COLOR_BLUE   = 0x04
};

#pragma pack(push, 1)
// 15 bytes packed
struct CapTarget {
  uint8_t  capId;        // 1..100 sequential ID in batch
  uint8_t  color;        // CapColor enum (1=Red, 2=Yellow, 3=Green, 4=Blue)
  uint16_t pixelX;       // Detected image centroid u (px)
  uint16_t pixelY;       // Detected image centroid v (px)
  float    boardX_mm;    // Homography-mapped physical board X (0.0 - 300.0 mm)
  float    boardY_mm;    // Homography-mapped physical board Y (0.0 - 300.0 mm)
  uint8_t  targetBinId;  // Assigned disposal bin index (1..4)
};

// 6 bytes packed
struct BatchHeader {
  uint8_t  magic;        // Must equal PROTOCOL_MAGIC_BYTE (0x5A)
  uint16_t batchId;      // Monotonic batch counter
  uint8_t  totalCaps;    // Number of CapTarget structs following header (1..100)
  uint16_t crc16;        // CRC16-CCITT (poly 0x1021, init 0xFFFF) over CapTarget payload
};
#pragma pack(pop)

// Shared CRC16-CCITT implementation (Polynomial 0x1021, Initial 0xFFFF)
static inline uint16_t computeCRC16(const uint8_t* data, uint32_t length) {
  uint16_t crc = 0xFFFF;
  for (uint32_t i = 0; i < length; i++) {
    crc ^= ((uint16_t)data[i] << 8);
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc = (crc << 1);
      }
    }
  }
  return crc;
}

#endif
