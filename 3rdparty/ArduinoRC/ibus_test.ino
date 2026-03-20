// FlySky iBus Live Monitor (Mega2560)

#include <Arduino.h>

#define IBUS_SERIAL Serial2

static const uint8_t IBUS_FRAME_LEN = 32;
uint8_t ibus_buf[IBUS_FRAME_LEN];
uint8_t ibus_idx = 0;
uint16_t ibus_ch[14] = {1500};
unsigned long lastFrame = 0;

bool readIbusFrame() {
  while (IBUS_SERIAL.available()) {
    uint8_t b = IBUS_SERIAL.read();

    if (ibus_idx == 0 && b != 0x20) continue;
    if (ibus_idx == 1 && b != 0x40) { ibus_idx = 0; continue; }

    ibus_buf[ibus_idx++] = b;

    if (ibus_idx == IBUS_FRAME_LEN) {
      uint16_t sum = 0xFFFF;
      for (int i = 0; i < IBUS_FRAME_LEN - 2; i++) sum -= ibus_buf[i];

      uint16_t rxsum = ibus_buf[30] | (ibus_buf[31] << 8);
      ibus_idx = 0;

      if (sum != rxsum) return false;

      for (int ch = 0; ch < 14; ch++) {
        ibus_ch[ch] = ibus_buf[2 + ch * 2] | (ibus_buf[3 + ch * 2] << 8);
      }

      lastFrame = millis();
      return true;
    }
  }
  return false;
}

void setup() {
  Serial.begin(115200);     // USB monitor
  IBUS_SERIAL.begin(115200); // FlySky iBus

  Serial.println("iBus Monitor Starting...");
}

void loop() {
  if (readIbusFrame()) {

    Serial.print("CH1: "); Serial.print(ibus_ch[0]);
    Serial.print("  CH2: "); Serial.print(ibus_ch[1]);
    Serial.print("  CH3: "); Serial.print(ibus_ch[2]);
    Serial.print("  CH4: "); Serial.print(ibus_ch[3]);
    Serial.print("  CH5: "); Serial.print(ibus_ch[4]);
    Serial.print("  CH6: "); Serial.print(ibus_ch[5]);

    Serial.println();
  }

  // signal loss warning
  if (millis() - lastFrame > 500) {
    Serial.println("⚠️ NO SIGNAL");
    delay(500);
  }
}