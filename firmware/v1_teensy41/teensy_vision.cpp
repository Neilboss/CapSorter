// =========================================================================
// Teensy 4.1 - Embedded Vision & Plan Generator (V1)
// Status: Pre-Validation Engineering Firmware (requires bench calibration)
// Hardware: ARM Cortex-M7 @ 600 MHz | RAM: 1024 KB | 3.3V ONLY (NOT 5V tolerant)
// Wiring:
//   Serial2 (Pin 7 RX2, Pin 8 TX2) <-> ESP32-CAM UART2 (921,600 baud, 3.3V)
//   Serial1 (Pin 0 RX1, Pin 1 TX1) <-> Bidirectional 3.3V/5V Shifter <-> Mega Serial1 (115,200)
// =========================================================================
#include <Arduino.h>
#include <math.h>
#include <string.h>
#include "../common/packet_protocol.h"

#define FRAME_WIDTH        320
#define FRAME_HEIGHT       240
#define FRAME_PIXELS       (FRAME_WIDTH * FRAME_HEIGHT)
#define FRAME_BYTES        (FRAME_PIXELS * 2) // 153,600 bytes RGB565
#define BFS_QUEUE_SIZE     4096
#define MIN_CAP_SEP_MM     24.0f              // Spatial NMS radius (caps are 28-32 mm)

// Supervision & Handshake Timeouts
#define READY_POLL_INTERVAL_MS   500UL
#define HEARTBEAT_INTERVAL_MS    250UL
#define MAX_SINGLE_CAP_MS      25000UL  // Per-cap stall watchdog (reset by CMD_CAP_PROGRESS)
#define PER_CAP_BUDGET_MS      18000UL  // Worst-case cap cycle + margin for dynamic total ceiling
#define BATCH_OVERHEAD_MS      30000UL  // Homing/parking & handshake margin

// FAIL-CLOSED CALIBRATION GUARD:
// Measure the 4 board corner fiducials (Top-Left, Top-Right, Bottom-Right, Bottom-Left)
// in pixels at the installed camera height, update BOARD_FIDUCIALS_PX below, verify mapped
// test-grid accuracy across board edges, and ONLY THEN set V1_CALIBRATION_VERIFIED to 1.
#define V1_CALIBRATION_VERIFIED  0

const float BOARD_FIDUCIALS_MM[4][2] = {
  {   0.0f,   0.0f },  // Top-Left (Origin)
  { 300.0f,   0.0f },  // Top-Right
  { 300.0f, 300.0f },  // Bottom-Right
  {   0.0f, 300.0f }   // Bottom-Left
};

// Placeholder fiducial pixel coordinates — MUST be replaced with bench measurements
const float BOARD_FIDUCIALS_PX[4][2] = {
  {  16.0f,  16.0f },  // Top-Left fiducial (u, v)
  { 304.0f,  16.0f },  // Top-Right fiducial (u, v)
  { 304.0f, 224.0f },  // Bottom-Right fiducial (u, v)
  {  16.0f, 224.0f }   // Bottom-Left fiducial (u, v)
};

// Allocate large buffers in Teensy 4.1 DMAMEM (512 KB RAM2)
DMAMEM uint8_t  serial2RxBuf[16384]; // 16 KB DMA RX buffer prevents 921,600 baud drops
DMAMEM uint8_t  imageBuffer[FRAME_BYTES];
DMAMEM uint8_t  rawMask[FRAME_PIXELS];
DMAMEM uint8_t  colorMask[FRAME_PIXELS];
DMAMEM uint32_t bfsQueue[BFS_QUEUE_SIZE];

CapTarget detectedCaps[MAX_BATCH_CAPS];
uint32_t  detectedAreas[MAX_BATCH_CAPS];
uint8_t   capCount       = 0;
uint16_t  currentBatchId = 1;
float     H[9]           = {0};
bool      homographyOk   = false;
bool      megaFaulted    = false;

// External Sorting Bin Coordinates (mm)
const float BIN_X_MM[5] = { 0.0f, 355.0f, 355.0f, 355.0f, 355.0f };
const float BIN_Y_MM[5] = { 0.0f,  55.0f, 130.0f, 205.0f, 280.0f };

enum BatchExecResult : uint8_t {
  EXEC_COMPLETE,
  EXEC_FAULT_HALT,
  EXEC_TIMEOUT
};

// Solve 3x3 planar homography H (h8 = 1) from 4 fiducial point pairs using
// 8x8 Gaussian elimination with partial pivoting (returns false if singular).
bool computeHomography4Pt(const float srcPx[4][2], const float dstMm[4][2], float outH[9]) {
  double A[8][9];
  for (int i = 0; i < 4; i++) {
    double u = srcPx[i][0], v = srcPx[i][1];
    double x = dstMm[i][0], y = dstMm[i][1];

    A[2 * i][0] = u;  A[2 * i][1] = v;  A[2 * i][2] = 1.0;
    A[2 * i][3] = 0;  A[2 * i][4] = 0;  A[2 * i][5] = 0;
    A[2 * i][6] = -u * x; A[2 * i][7] = -v * x; A[2 * i][8] = x;

    A[2 * i + 1][0] = 0;  A[2 * i + 1][1] = 0;  A[2 * i + 1][2] = 0;
    A[2 * i + 1][3] = u;  A[2 * i + 1][4] = v;  A[2 * i + 1][5] = 1.0;
    A[2 * i + 1][6] = -u * y; A[2 * i + 1][7] = -v * y; A[2 * i + 1][8] = y;
  }

  for (int col = 0; col < 8; col++) {
    int pivot = col;
    double maxVal = fabs(A[col][col]);
    for (int row = col + 1; row < 8; row++) {
      double v = fabs(A[row][col]);
      if (v > maxVal) {
        maxVal = v;
        pivot = row;
      }
    }
    if (maxVal < 1e-10) return false;
    if (pivot != col) {
      for (int c = col; c < 9; c++) {
        double tmp = A[col][c];
        A[col][c] = A[pivot][c];
        A[pivot][c] = tmp;
      }
    }
    double div = A[col][col];
    for (int c = col; c < 9; c++) A[col][c] /= div;
    for (int row = 0; row < 8; row++) {
      if (row == col) continue;
      double factor = A[row][col];
      for (int c = col; c < 9; c++) A[row][c] -= factor * A[col][c];
    }
  }

  for (int i = 0; i < 8; i++) {
    if (!isfinite(A[i][8])) return false;
    outH[i] = (float)A[i][8];
  }
  outH[8] = 1.0f;
  return true;
}

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

bool waitForMegaReady(uint32_t timeoutMs) {
  uint32_t startMs = millis();
  uint32_t lastPollMs = 0;
  while (timeoutMs == 0 || (millis() - startMs < timeoutMs)) {
    if (lastPollMs == 0 || (millis() - lastPollMs >= READY_POLL_INTERVAL_MS)) {
      Serial1.write(CMD_QUERY_STATUS);
      Serial1.flush();
      lastPollMs = millis();
    }
    while (Serial1.available() > 0) {
      uint8_t resp = (uint8_t)Serial1.read();
      if (resp == CMD_STATUS_READY) {
        megaFaulted = false;
        return true;
      }
      if (resp == CMD_STATUS_FAULT || resp == CMD_HARD_FAULT || resp == CMD_RECOVERY_HALT) {
        megaFaulted = true;
        Serial.println(F("[TEENSY] Mega reports FAULT/HALT; motion inhibited until operator re-arm."));
        return false;
      }
    }
    delay(10);
  }
  return false;
}

bool receiveFrame(uint8_t *buf, uint32_t expectedBytes) {
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

  uint8_t lenBytes[4];
  if (!readBytesWithTimeout(Serial2, lenBytes, 4, 500)) return false;
  uint32_t frameLen = ((uint32_t)lenBytes[0] << 24) |
                      ((uint32_t)lenBytes[1] << 16) |
                      ((uint32_t)lenBytes[2] << 8)  |
                      ((uint32_t)lenBytes[3]);
  if (frameLen != expectedBytes) return false;

  // Read full RGB565 payload (153,600 bytes @ 921,600 baud ~ 1.67 s)
  if (!readBytesWithTimeout(Serial2, buf, expectedBytes, 3000)) return false;

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

void cleanColorMask3x3() {
  memset(colorMask, COLOR_NONE, FRAME_PIXELS);
  for (uint16_t y = 1; y < FRAME_HEIGHT - 1; y++) {
    for (uint16_t x = 1; x < FRAME_WIDTH - 1; x++) {
      uint32_t idx = (uint32_t)y * FRAME_WIDTH + x;
      uint8_t center = rawMask[idx];
      uint8_t counts[5] = {0, 0, 0, 0, 0};
      for (int8_t dy = -1; dy <= 1; dy++) {
        for (int8_t dx = -1; dx <= 1; dx++) {
          uint8_t c = rawMask[(uint32_t)(y + dy) * FRAME_WIDTH + (x + dx)];
          if (c <= 4) counts[c]++;
        }
      }
      if (center != COLOR_NONE && counts[center] >= 5) {
        colorMask[idx] = center;
      } else if (center == COLOR_NONE) {
        for (uint8_t col = 1; col <= 4; col++) {
          if (counts[col] >= 6) {
            colorMask[idx] = col;
            break;
          }
        }
      }
    }
  }
}

bool transformToMillimeters(CapTarget *cap) {
  float u = (float)cap->pixelX;
  float v = (float)cap->pixelY;
  float w = H[6] * u + H[7] * v + H[8];
  if (fabsf(w) < 1e-6f) return false;
  cap->boardX_mm = (H[0] * u + H[1] * v + H[2]) / w;
  cap->boardY_mm = (H[3] * u + H[4] * v + H[5]) / w;
  return isfinite(cap->boardX_mm) && isfinite(cap->boardY_mm);
}

// Greedy spatial NMS sorted strictly by descending contour area so metadata & areas
// stay synchronized and a smaller overlapping blob can never overwrite a larger one.
void deduplicateTargetsNMS(CapTarget *caps, uint32_t *areas, uint8_t *count) {
  uint8_t n = *count;
  if (n <= 1) return;

  // 1. Insertion-sort caps[] and areas[] together by area descending
  for (uint8_t i = 1; i < n; i++) {
    CapTarget keyCap  = caps[i];
    uint32_t  keyArea = areas[i];
    int16_t j = (int16_t)i - 1;
    while (j >= 0 && areas[j] < keyArea) {
      caps[j + 1]  = caps[j];
      areas[j + 1] = areas[j];
      j--;
    }
    caps[j + 1]  = keyCap;
    areas[j + 1] = keyArea;
  }

  // 2. Greedily retain highest-area candidates separated by >= MIN_CAP_SEP_MM
  uint8_t kept = 0;
  for (uint8_t i = 0; i < n; i++) {
    bool overlap = false;
    for (uint8_t k = 0; k < kept; k++) {
      float dist = hypotf(caps[i].boardX_mm - caps[k].boardX_mm,
                          caps[i].boardY_mm - caps[k].boardY_mm);
      if (dist < MIN_CAP_SEP_MM) {
        overlap = true;
        break;
      }
    }
    if (!overlap) {
      caps[kept]       = caps[i];
      areas[kept]      = areas[i];
      caps[kept].capId = kept + 1;
      kept++;
    }
  }
  *count = kept;
}

void segmentCapsHSV(const uint8_t *frameBuf, CapTarget *outCaps, uint8_t *outCount) {
  *outCount = 0;
  if (!homographyOk) return;

  for (uint32_t i = 0; i < FRAME_PIXELS; i++) {
    uint16_t px = ((uint16_t)frameBuf[i * 2] << 8) | frameBuf[i * 2 + 1];
    rawMask[i] = classifyRgb565Pixel(px);
  }

  cleanColorMask3x3();

  const int8_t dx[4] = { 1, -1, 0, 0 };
  const int8_t dy[4] = { 0, 0, 1, -1 };

  for (uint16_t y = 2; y < FRAME_HEIGHT - 2; y++) {
    for (uint16_t x = 2; x < FRAME_WIDTH - 2; x++) {
      uint32_t seedIdx = (uint32_t)y * FRAME_WIDTH + x;
      uint8_t targetColor = colorMask[seedIdx];
      if (targetColor == COLOR_NONE || (targetColor & 0x80)) continue;

      uint32_t area = 0, perimeter = 0, sumX = 0, sumY = 0;
      uint16_t qHead = 0, qTail = 0;

      colorMask[seedIdx] |= 0x80;
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

      if (area >= 220 && area <= 1800 && perimeter > 0 && *outCount < MAX_BATCH_CAPS) {
        float circularity = (4.0f * 3.1415926f * (float)area) / ((float)perimeter * (float)perimeter);
        if (circularity >= 0.72f) {
          CapTarget &cap = outCaps[*outCount];
          cap.capId       = *outCount + 1;
          cap.color       = targetColor;
          cap.pixelX      = (uint16_t)(sumX / area);
          cap.pixelY      = (uint16_t)(sumY / area);
          cap.targetBinId = targetColor;

          if (transformToMillimeters(&cap) &&
              cap.boardX_mm >= 0.0f && cap.boardX_mm <= 300.0f &&
              cap.boardY_mm >= 0.0f && cap.boardY_mm <= 300.0f) {
            detectedAreas[*outCount] = area;
            (*outCount)++;
          }
        }
      }
    }
  }

  deduplicateTargetsNMS(outCaps, detectedAreas, outCount);
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

void writePacedSerial1(const uint8_t *data, uint32_t len) {
  // Pace transmissions in 32-byte chunks with 2 ms pause so Mega's 64B RX FIFO never overflows
  uint32_t offset = 0;
  while (offset < len) {
    uint32_t chunk = min((uint32_t)32, len - offset);
    Serial1.write(data + offset, chunk);
    Serial1.flush();
    offset += chunk;
    if (offset < len) delay(2);
  }
}

bool sendPlanBatchToMega(const CapTarget *caps, uint8_t count) {
  if (count == 0 || count > MAX_BATCH_CAPS) return false;
  uint32_t payloadBytes = (uint32_t)count * sizeof(CapTarget);

  BatchHeader hdr;
  hdr.magic     = PROTOCOL_MAGIC_BYTE;
  hdr.batchId   = currentBatchId++;
  hdr.totalCaps = count;
  hdr.crc16     = computeCRC16((const uint8_t*)caps, payloadBytes);

  for (uint8_t attempt = 0; attempt < 3; attempt++) {
    while (Serial1.available() > 0) Serial1.read();
    Serial1.write((const uint8_t*)&hdr, sizeof(BatchHeader));
    Serial1.flush();
    delay(2);
    writePacedSerial1((const uint8_t*)caps, payloadBytes);

    uint32_t waitStart = millis();
    while (millis() - waitStart < 1000) {
      if (Serial1.available() > 0) {
        uint8_t resp = (uint8_t)Serial1.read();
        if (resp == CMD_BATCH_ACK) return true;
        if (resp == CMD_BATCH_NACK) break;
        if (resp == CMD_HARD_FAULT || resp == CMD_RECOVERY_HALT || resp == CMD_STATUS_FAULT) {
          megaFaulted = true;
          return false;
        }
      }
    }
    delay(50);
  }
  return false;
}

BatchExecResult superviseMegaExecutionWithHeartbeat(uint8_t count) {
  // Maintain 250 ms heartbeat for the full batch duration, bounded by per-cap progress
  // and dynamic worst-case total duration (never a fixed 180 s cutoff).
  uint32_t batchStartMs   = millis();
  uint32_t lastPingMs     = batchStartMs;
  uint32_t lastProgressMs = batchStartMs;
  uint32_t maxTotalMs     = (uint32_t)count * PER_CAP_BUDGET_MS + BATCH_OVERHEAD_MS;

  while (true) {
    uint32_t now = millis();
    if (now - lastPingMs >= HEARTBEAT_INTERVAL_MS) {
      Serial1.write(CMD_HEARTBEAT_PING);
      lastPingMs = now;
    }

    while (Serial1.available() > 0) {
      uint8_t status = (uint8_t)Serial1.read();
      if (status == CMD_CAP_PROGRESS) {
        lastProgressMs = millis();
      } else if (status == CMD_BATCH_COMPLETE) {
        return EXEC_COMPLETE;
      } else if (status == CMD_RECOVERY_HALT || status == CMD_HARD_FAULT || status == CMD_STATUS_FAULT) {
        megaFaulted = true;
        return EXEC_FAULT_HALT;
      }
    }

    if ((now - lastProgressMs > MAX_SINGLE_CAP_MS) || (now - batchStartMs > maxTotalMs)) {
      Serial.println(F("[TEENSY] Batch supervision stall/timeout detected."));
      megaFaulted = true;
      return EXEC_TIMEOUT;
    }
    delay(5);
  }
}

void setup() {
  Serial.begin(115200);   // USB Debug
  Serial1.begin(115200);  // Arduino Mega Link (Bidirectional 3.3V <-> 5V Level Shifter)
  Serial2.addMemoryForRead(serial2RxBuf, sizeof(serial2RxBuf));
  Serial2.begin(921600);  // ESP32-CAM High-Speed Wired Link
  pinMode(LED_BUILTIN, OUTPUT);

  homographyOk = computeHomography4Pt(BOARD_FIDUCIALS_PX, BOARD_FIDUCIALS_MM, H);
  if (!homographyOk) {
    Serial.println(F("[TEENSY ERROR] Fiducial homography matrix is singular; motion inhibited."));
  }
#if !V1_CALIBRATION_VERIFIED
  Serial.println(F("[TEENSY FAIL-CLOSED] V1_CALIBRATION_VERIFIED == 0."));
  Serial.println(F("Measure 4 board fiducial corners, update BOARD_FIDUCIALS_PX, and set V1_CALIBRATION_VERIFIED 1."));
#endif

  Serial.println(F("[TEENSY] Waiting for Mega CMD_STATUS_READY handshake..."));
  waitForMegaReady(0); // Wait indefinitely for Mega homing completion or fault report
}

void loop() {
#if !V1_CALIBRATION_VERIFIED
  delay(1000);
  return;
#endif

  if (!homographyOk || megaFaulted) {
    // Poll Mega status passively; operator must explicitly re-arm Mega before resuming
    if (waitForMegaReady(1000)) {
      Serial.println(F("[TEENSY] Mega re-armed and READY."));
    }
    delay(500);
    return;
  }

  // Confirm Mega is READY & parked at (0,0) before triggering camera capture
  if (!waitForMegaReady(2000)) {
    delay(500);
    return;
  }

  while (Serial2.available() > 0) Serial2.read();
  Serial2.write(CMD_CAPTURE_FRAME);

  if (receiveFrame(imageBuffer, FRAME_BYTES)) {
    digitalWrite(LED_BUILTIN, HIGH);
    segmentCapsHSV(imageBuffer, detectedCaps, &capCount);

    if (capCount > 0) {
      optimizeSortingPath(detectedCaps, capCount);
      if (sendPlanBatchToMega(detectedCaps, capCount)) {
        BatchExecResult res = superviseMegaExecutionWithHeartbeat(capCount);
        if (res != EXEC_COMPLETE) {
          Serial.println(F("[TEENSY] Halting cycle until Mega is re-armed and READY."));
        }
      }
    }
    digitalWrite(LED_BUILTIN, LOW);
  }
  delay(500);
}
