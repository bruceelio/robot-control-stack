void setup() {
  Serial.begin(115200);
  Serial1.begin(38400);
  delay(1000);
  Serial.println("Starting RoboClaw test");
}

void sendPacket(uint8_t addr, uint8_t cmd, uint8_t value) {
  uint16_t crc = addr + cmd + value;
  Serial1.write(addr);
  Serial1.write(cmd);
  Serial1.write(value);
  Serial1.write((crc >> 8) & 0xFF);
  Serial1.write(crc & 0xFF);
}

void loop() {
  Serial.println("M1 forward");
  sendPacket(0x80, 0x00, 32);   // M1 forward, low speed
  delay(2000);

  Serial.println("M1 stop");
  sendPacket(0x80, 0x00, 0);    // M1 forward with zero
  delay(1000);

  Serial.println("M1 backward");
  sendPacket(0x80, 0x01, 32);   // M1 backward, low speed
  delay(2000);

  Serial.println("M1 stop");
  sendPacket(0x80, 0x01, 0);    // M1 backward with zero
  delay(2000);
}