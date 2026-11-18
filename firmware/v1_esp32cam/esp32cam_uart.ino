// =========================================================================
// ESP32-CAM (AI-Thinker OV2640) - High-Speed Frame Streamer (V1)
// Pure Image Acquisition (Zero Network Overhead, 32-Bit Length + CRC16)
// Wiring: GPIO13 (TX2) -> Teensy Pin 7 (RX2) | GPIO14 (RX2) <- Teensy Pin 8 (TX2)
// =========================================================================
#include "esp_camera.h"
#include <HardwareSerial.h>
#include "../common/packet_protocol.h"

HardwareSerial TeensySerial(2); // UART2 (RX=GPIO14, TX=GPIO13)

void setup() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = 5;  config.pin_d1 = 18; config.pin_d2 = 19; config.pin_d3 = 21;
  config.pin_d4       = 36; config.pin_d5 = 39; config.pin_d6 = 34; config.pin_d7 = 35;
  config.pin_xclk     = 0;  config.pin_pclk = 22; config.pin_vsync = 25; config.pin_href = 23;
  config.pin_sscb_sda = 26; config.pin_sscb_scl = 27;
  config.pin_pwdn     = 32; config.pin_reset = -1;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_RGB565;   // Raw 16-bit color (320x240x2 = 153,600 bytes)
  config.frame_size   = FRAMESIZE_QVGA;     // 320x240
  config.fb_count     = 2;
  config.grab_mode    = CAMERA_GRAB_LATEST; // Always capture freshest frame on trigger

  if (esp_camera_init(&config) != ESP_OK) {
    while (true) { delay(1000); }
  }

  // 1. Allow AWB, AEC, and AGC to converge under diffused LED light plate for 800 ms
  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_exposure_ctrl(s, 1);
    s->set_gain_ctrl(s, 1);
    delay(800);

    // Flush 2 warm-up frames
    for (int i = 0; i < 2; i++) {
      camera_fb_t *warmFb = esp_camera_fb_get();
      if (warmFb) esp_camera_fb_return(warmFb);
      delay(50);
    }

    // 2. Lock AWB, Exposure, and Gain so colored caps never shift hue balance!
    s->set_whitebal(s, 0);
    s->set_awb_gain(s, 0);
    s->set_exposure_ctrl(s, 0);
    s->set_gain_ctrl(s, 0);
  }

  TeensySerial.begin(921600, SERIAL_8N1, 14, 13);
}

void loop() {
  if (TeensySerial.available() > 0) {
    uint8_t cmd = TeensySerial.read();
    if (cmd == CMD_CAPTURE_FRAME) {
      camera_fb_t *fb = esp_camera_fb_get();
      if (fb) {
        uint32_t frameLen = (uint32_t)fb->len; // 153,600 bytes (requires 32-bit length!)
        uint16_t crc16    = computeCRC16(fb->buf, frameLen);

        // 1. Send 6-byte Frame Header: [0x55, 0xAA, Len3, Len2, Len1, Len0]
        uint8_t header[6] = {
          FRAME_SYNC_BYTE_1,
          FRAME_SYNC_BYTE_2,
          (uint8_t)((frameLen >> 24) & 0xFF),
          (uint8_t)((frameLen >> 16) & 0xFF),
          (uint8_t)((frameLen >> 8)  & 0xFF),
          (uint8_t)(frameLen & 0xFF)
        };
        TeensySerial.write(header, 6);

        // 2. Stream raw RGB565 framebuffer bytes (153,600 bytes)
        TeensySerial.write(fb->buf, frameLen);

        // 3. Send 2-byte CRC16-CCITT trailer [CrcHigh, CrcLow]
        TeensySerial.write((uint8_t)((crc16 >> 8) & 0xFF));
        TeensySerial.write((uint8_t)(crc16 & 0xFF));
        TeensySerial.flush();

        esp_camera_fb_return(fb);
      }
    }
  }
}
