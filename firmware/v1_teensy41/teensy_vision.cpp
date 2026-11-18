// =========================================================================
// Teensy 4.1 - Embedded Vision & Plan Generator (V1)
// Hardware: ARM Cortex-M7 @ 600 MHz | RAM: 1024 KB
// Wiring:
//   Serial2 (Pin 7 RX2, Pin 8 TX2) <-> ESP32-CAM UART2 (921,600 baud, 3.3V)
//   Serial1 (Pin 0 RX1, Pin 1 TX1) <-> 3.3V/5V Shifter <-> Mega Serial1 (115,200)
// =========================================================================
#include <Arduino.h>
#include <math.h>
#include "../common/packet_protocol.h"

#define FRAME_WIDTH       320
#define FRAME_HEIGHT      240
#define FRAME_PIXELS      (FRAME_WIDTH * FRAME_HEIGHT)
#define FRAME_BYTES       (FRAME_PIXELS * 2) // 153,600 bytes RGB565
#define BFS_QUEUE_SIZE    4096

// Allocate large buffers in Teensy 4.1 DMAMEM (512 KB RAM2)
DMAMEM uint8_t  imageBuffer[FRAME_BYTES];
DMAMEM uint8_t  colorMask[FRAME_PIXELS];
DMAMEM uint32_t bfsQueue[BFS_QUEUE_SIZE];

CapTarget detectedCaps[100];
uint8_t   capCount = 0;
uint16_t  currentBatchId = 1;

// Planar Homography 3x3 Calibration Matrix (Image px -> Board mm)
const float H[9] = {
  0.9854f, -0.0120f, -12.45f,
  0.0105f,  0.9782f, -14.10f,
  0.00004f, 0.00003f, 1.0000f
};

// External Sorting Bin Coordinates (mm)
const float BIN_X_MM[5] = { 0.0f, 355.0f, 355.0f, 355.0f, 355.0f };
const float BIN_Y_MM[5] = { 0.0f,  55.0f, 130.0f, 205.0f, 280.0f };

bool readBytesWithTimeout(HardwareSerial &port, uint8_t *dest, uint32_t len, uint32_t timeoutMs) {
  uint32_t startMs = millis();
  uint32_t idx = 0;
  while (idx < len) {
    if (port.available() > 0) {
      dest[idx++] = (uint8_t)port.read();
    } else if (millis() - startMs > timeoutMs) {
      return false;
    }
  }
  return true;
}

bool receiveFrame(uint8_t *buf, uint32_t expectedBytes) {
  // Wait for sync header [0x55, 0xAA]
  uint32_t startMs = millis();
  uint8_t prev = 0;
  bool synced = false;
  while (millis() - startMs < 2500) {
    if (Serial2.available() > 0) {
      uint8_t b = (uint8_t)Serial2.read();
      if (prev == FRAME_SYNC_BYTE_1 && b == FRAME_SYNC_BYTE_2) {
        synced = true;
        break;
      }
      prev = b;
    }
  }
  if (!synced) return false;

  // Read 4-byte uint32_t frame length
  uint8_t lenBytes[4];
  if (!readBytesWithTimeout(Serial2, lenBytes, 4, 500)) return false;
  uint32_t frameLen = ((uint32_t)lenBytes[0] << 24) |
                      ((uint32_t)lenBytes[1] << 16) |
                      ((uint32_t)lenBytes[2] << 8)  |
                      ((uint32_t)lenBytes[3]);
  if (frameLen != expectedBytes) return false;

  // Read full RGB565 payload (153,600 bytes @ 921,600 baud ~ 1.67s)
  if (!readBytesWithTimeout(Serial2, buf, expectedBytes, 3000)) return false;

  // Read 2-byte CRC16-CCITT and verify
  uint8_t crcBytes[2];
  if (!readBytesWithTimeout(Serial2, crcBytes, 2, 500)) return false;
  uint16_t rxCrc = ((uint16_t)crcBytes[0] << 8) | crcBytes[1];
  return (computeCRC16(buf, expectedBytes) == rxCrc);
}

uint8_t classifyRgb565Pixel(uint16_t pixel) {
  uint8_t r = (uint8_t)(((pixel >> 11) & 0x1F) * 255 / 31);
  uint8_t g = (uint8_t)(((pixel >> 5)  & 0x3F) * 255 / 63);
  uint8_t b = (uint8_t)((pixel & 0x1F) * 255 / 31);

  uint8_t cmax = max(r, max(g, b));
  uint8_t cmin = min(r, min(g, b));
  uint8_t delta = cmax - cmin;

  if (cmax < 60 || delta == 0) return COLOR_NONE;
  uint8_t s = (uint8_t)(((uint16_t)delta * 255) / cmax);
  if (s < 70) return COLOR_NONE;

  // Compute Hue in [0..180] matching OpenCV convention
  int16_t h = 0;
  if (cmax == r) {
    h = (30 * (int16_t)(g - b)) / delta;
  } else if (cmax == g) {
    h = 60 + (30 * (int16_t)(b - r)) / delta;
  } else {
    h = 120 + (30 * (int16_t)(r - g)) / delta;
  }
  if (h < 0) h += 180;

  if ((h < 10 || h > 168) && s > 95)  return COLOR_RED;
  if (h >= 18 && h <= 35  && s > 95)  return COLOR_YELLOW;
  if (h >= 38 && h <= 85  && s > 75)  return COLOR_GREEN;
  if (h >= 98 && h <= 132 && s > 85)  return COLOR_BLUE;
  return COLOR_NONE;
}

void transformToMillimeters(CapTarget *cap) {
  float u = (float)cap->pixelX;
  float v = (float)cap->pixelY;
  float w = H[6] * u + H[7] * v + H[8];
  cap->boardX_mm = (H[0] * u + H[1] * v + H[2]) / w;
  cap->boardY_mm = (H[3] * u + H[4] * v + H[5]) / w;
}

void segmentCapsHSV(const uint8_t *frameBuf, CapTarget *outCaps, uint8_t *outCount) {
  *outCount = 0;
  // 1. Classify every pixel into colorMask
  for (uint32_t i = 0; i < FRAME_PIXELS; i++) {
    uint16_t px = ((uint16_t)frameBuf[i * 2] << 8) | frameBuf[i * 2 + 1];
    colorMask[i] = classifyRgb565Pixel(px);
  }

  // 2. Iterative Connected-Components BFS with Circularity Gating
  const int8_t dx[4] = { 1, -1, 0, 0 };
  const int8_t dy[4] = { 0, 0, 1, -1 };

  for (uint16_t y = 2; y < FRAME_HEIGHT - 2; y++) {
    for (uint16_t x = 2; x < FRAME_WIDTH - 2; x++) {
      uint32_t seedIdx = (uint32_t)y * FRAME_WIDTH + x;
      uint8_t targetColor = colorMask[seedIdx];
      if (targetColor == COLOR_NONE || (targetColor & 0x80)) continue;

      uint32_t area = 0, perimeter = 0, sumX = 0, sumY = 0;
      uint16_t qHead = 0, qTail = 0;

      colorMask[seedIdx] |= 0x80; // Mark visited
      bfsQueue[qTail++] = seedIdx;

      while (qHead != qTail) {
        uint32_t curr = bfsQueue[qHead++];
        qHead &= (BFS_QUEUE_SIZE - 1);
        uint16_t cx = (uint16_t)(curr % FRAME_WIDTH);
        uint16_t cy = (uint16_t)(curr / FRAME_WIDTH);

        area++;
        sumX += cx;
        sumY += cy;
        bool isBoundary = false;

        for (uint8_t d = 0; d < 4; d++) {
          int16_t nx = (int16_t)cx + dx[d];
          int16_t ny = (int16_t)cy + dy[d];
          if (nx < 0 || nx >= FRAME_WIDTH || ny < 0 || ny >= FRAME_HEIGHT) {
            isBoundary = true;
            continue;
          }
          uint32_t nIdx = (uint32_t)ny * FRAME_WIDTH + nx;
          uint8_t nCol = colorMask[nIdx];
          if ((nCol & 0x7F) != targetColor) {
            isBoundary = true;
          } else if (!(nCol & 0x80)) {
            colorMask[nIdx] |= 0x80;
            uint16_t nextTail = (qTail + 1) & (BFS_QUEUE_SIZE - 1);
            if (nextTail != qHead) {
              bfsQueue[qTail] = nIdx;
              qTail = nextTail;
            }
          }
        }
        if (isBoundary) perimeter++;
      }

      if (area >= 220 && area <= 1800 && perimeter > 0 && *outCount < 100) {
        float circularity = (4.0f * 3.1415926f * (float)area) / ((float)perimeter * (float)perimeter);
        if (circularity >= 0.72f) {
          CapTarget &cap = outCaps[*outCount];
          cap.capId       = *outCount + 1;
          cap.color       = targetColor;
          cap.pixelX      = (uint16_t)(sumX / area);
          cap.pixelY      = (uint16_t)(sumY / area);
          cap.targetBinId = targetColor;
          transformToMillimeters(&cap);

          if (cap.boardX_mm >= 0.0f && cap.boardX_mm <= 300.0f &&
              cap.boardY_mm >= 0.0f && cap.boardY_mm <= 300.0f) {
            (*outCount)++;
          }
        }
      }
    }
  }
}

void optimizeSortingPath(CapTarget *caps, uint8_t count) {
  float curX = 0.0f, curY = 0.0f;
  for (uint8_t i = 0; i < count; i++) {
    uint8_t bestIdx = i;
    float bestDist = 1e9f;
    for (uint8_t j = i; j < count; j++) {
      float dist = hypotf(caps[j].boardX_mm - curX, caps[j].boardY_mm - curY);
      if (dist < bestDist) {
        bestDist = dist;
        bestIdx = j;
      }
    }
    if (bestIdx != i) {
      CapTarget tmp = caps[i];
      caps[i] = caps[bestIdx];
      caps[bestIdx] = tmp;
    }
    caps[i].capId = i + 1;
    uint8_t binId = caps[i].targetBinId;
    curX = BIN_X_MM[binId];
    curY = BIN_Y_MM[binId];
  }
}

bool sendPlanBatchToMega(const CapTarget *caps, uint8_t count) {
  if (count == 0) return false;
  uint32_t payloadBytes = (uint32_t)count * sizeof(CapTarget);

  BatchHeader hdr;
  hdr.magic     = PROTOCOL_MAGIC_BYTE;
  hdr.batchId   = currentBatchId++;
  hdr.totalCaps = count;
  hdr.crc16     = computeCRC16((const uint8_t*)caps, payloadBytes);

  Serial1.write((const uint8_t*)&hdr, sizeof(BatchHeader));
  Serial1.write((const uint8_t*)caps, payloadBytes);
  Serial1.flush();

  // Wait for CMD_BATCH_ACK from Mega
  uint32_t waitStart = millis();
  while (millis() - waitStart < 1000) {
    if (Serial1.available() > 0) {
      uint8_t resp = (uint8_t)Serial1.read();
      if (resp == CMD_BATCH_ACK) return true;
      if (resp == CMD_BATCH_NACK) return false;
    }
  }
  return false;
}

void superviseMegaExecutionWithHeartbeat() {
  // Send 250ms keep-alive heartbeat while Mega moves gantry across workspace
  uint32_t lastPingMs = millis();
  uint32_t batchStartMs = millis();
  while (millis() - batchStartMs < 120000) {
    if (millis() - lastPingMs >= 250) {
      Serial1.write(CMD_HEARTBEAT_PING);
      lastPingMs = millis();
    }
    if (Serial1.available() > 0) {
      uint8_t status = (uint8_t)Serial1.read();
      if (status == CMD_BATCH_COMPLETE || status == CMD_RECOVERY_HALT) {
        return; // Gantry parked at (0,0) clear of camera FOV!
      }
    }
  }
}

void setup() {
  Serial.begin(115200);   // USB Debug
  Serial1.begin(115200);  // Arduino Mega Link (3.3V <-> 5V Level Shifter)
  Serial2.begin(921600);  // ESP32-CAM High-Speed Wired Link
  pinMode(LED_BUILTIN, OUTPUT);
  delay(1500);            // Allow Mega to complete initial homing
}

void loop() {
  // 1. Trigger ESP32-CAM capture only when gantry is parked at (0,0)
  while (Serial2.available() > 0) Serial2.read();
  Serial2.write(CMD_CAPTURE_FRAME);

  // 2. Read full QVGA RGB565 frame with 32-bit length & CRC16 verification
  if (receiveFrame(imageBuffer, FRAME_BYTES)) {
    digitalWrite(LED_BUILTIN, HIGH);
    segmentCapsHSV(imageBuffer, detectedCaps, &capCount);

    if (capCount > 0) {
      optimizeSortingPath(detectedCaps, capCount);
      if (sendPlanBatchToMega(detectedCaps, capCount)) {
        // Maintain 250ms heartbeat until Mega completes batch & parks at (0,0)
        superviseMegaExecutionWithHeartbeat();
      }
    }
    digitalWrite(LED_BUILTIN, LOW);
  }
  delay(500);
}
