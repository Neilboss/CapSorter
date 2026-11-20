// =========================================================================
// packet_protocol.h - Shared Inter-Controller Wire Definitions (V1 & V2)
// Ensures Type Safety, CRC16-CCITT Checking, Explicit Handshake & Supervision
// Python Packing Formats:
//   SingleTarget: struct.pack('<B', 0x5B) + CapTarget[15B] + struct.pack('<H', crc16) [18B]
//   BatchHeader : struct.pack('<BHBH', magic, batchId, totalCaps, crc16) [6B]
//   CapTarget   : struct.pack('<BBHHffB', id, color, pxX, pxY, x_mm, y_mm, bin) [15B]
// =========================================================================
#ifndef PACKET_PROTOCOL_H
#define PACKET_PROTOCOL_H

#include <stdint.h>

// Shared Batch Capacity & Continuous Board Reachability Limits
#define MAX_BATCH_CAPS        100   // Maximum CapTarget entries accepted in one legacy batch
#define BOARD_SIZE_MM         300.0f // Continuous 300x300 mm open flat plywood sorting plate
#define CAP_DIAMETER_NOM_MM    30.0f // Nominal beverage bottle cap outer diameter (28-32 mm)
#define SAFE_PICK_MIN_X_MM     18.0f // Min safe cap center X (keeps rim inside plate & clear of X-home)
#define SAFE_PICK_MAX_X_MM    282.0f // Max safe cap center X (keeps rim inside 300 mm plate)
#define SAFE_PICK_MIN_Y_MM     18.0f // Min safe cap center Y (keeps rim inside plate & clear of Y-home)
#define SAFE_PICK_MAX_Y_MM    282.0f // Max safe cap center Y (keeps rim inside 300 mm plate)

// Frame, Legacy Batch & Closed-Loop Single-Target Magic Bytes
#define FRAME_SYNC_BYTE_1     0x55
#define FRAME_SYNC_BYTE_2     0xAA
#define PROTOCOL_MAGIC_BYTE   0x5A  // Legacy multi-cap batch packet header
#define PROTOCOL_SINGLE_MAGIC 0x5B  // Closed-loop single-cap target: [0x5B][CapTarget 15B][CRC16 2B]

// Inter-Controller Command, Handshake & Supervision Opcodes
#define CMD_CAPTURE_FRAME     0xAA  // Teensy -> ESP32-CAM: Trigger fresh QVGA capture
#define CMD_BATCH_ACK         0xA1  // Mega -> Vision Host: Packet CRC16 & bounds valid, starting motion
#define CMD_BATCH_NACK        0xA2  // Mega -> Vision Host: CRC16 mismatch, out-of-bounds target, or busy
#define CMD_BATCH_COMPLETE    0xA3  // Mega -> Vision Host: Session/batch finished & gantry parked at (0,0)
#define CMD_CAP_PROGRESS      0xA4  // Mega -> Vision Host: One cap cycle completed (resets stall timer)
#define CMD_SCAN_WINDOW_OPEN  0xA5  // Mega -> Vision Host: Stationary at bin (Z raised, settled); capture fresh frame now
#define CMD_AWAIT_NEXT        0xA6  // Mega -> Vision Host: Bin retract (or clear-FOV park) complete; ready for next command

#define CMD_QUERY_STATUS      0xB0  // Vision Host -> Mega: Query readiness/fault state
#define CMD_STATUS_READY      0xB1  // Mega -> Vision Host: Homed, fault-free, ready in STATE_IDLE
#define CMD_STATUS_BUSY       0xB2  // Mega -> Vision Host: Homing or actively executing a session/batch
#define CMD_STATUS_FAULT      0xB3  // Mega -> Vision Host: Fail-safe halt or hard fault latched
#define CMD_REARM_HOME        0xB4  // Vision Host -> Mega: Explicit operator re-arm & re-home request
#define CMD_PARK_FOR_SCAN     0xB5  // Vision Host -> Mega: Move to clear-FOV (0,0) and emit CMD_AWAIT_NEXT
#define CMD_FINISH_SESSION    0xB6  // Vision Host -> Mega: Park at (0,0) if needed, emit CMD_BATCH_COMPLETE -> STATE_IDLE

#define CMD_HEARTBEAT_PING    0xF0  // Vision Host -> Mega: 250ms keep-alive pulse during active session/batch
#define CMD_RECOVERY_HALT     0xEE  // Mega -> Vision Host: Comms-loss fail-safe halt latched
#define CMD_HARD_FAULT        0xEF  // Mega -> Vision Host: E-stop, NC limit trip, or homing fault

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
  uint8_t  totalCaps;    // Number of CapTarget structs following header (1..MAX_BATCH_CAPS)
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
