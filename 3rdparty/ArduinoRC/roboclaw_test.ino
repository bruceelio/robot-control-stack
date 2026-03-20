    void setup() {
      Serial1.begin(38400);
      delay(500);
    }

    void loop() {
      uint8_t addr = 0x80;
      uint8_t cmd  = 0x00;   // M1 forward
      uint8_t val  = 32;     // low speed
      uint16_t crc = addr + cmd + val;

      Serial1.write(addr);
      Serial1.write(cmd);
      Serial1.write(val);
      Serial1.write((crc >> 8) & 0xFF);
      Serial1.write(crc & 0xFF);

      delay(1000);

      cmd = 0x01;            // M1 backward
      crc = addr + cmd + val;

      Serial1.write(addr);
      Serial1.write(cmd);
      Serial1.write(val);
      Serial1.write((crc >> 8) & 0xFF);
      Serial1.write(crc & 0xFF);

      delay(1000);

      cmd = 0x00;            // M1 forward with 0 speed = stop on some tests
      val = 0;
      crc = addr + cmd + val;

      Serial1.write(addr);
      Serial1.write(cmd);
      Serial1.write(val);
      Serial1.write((crc >> 8) & 0xFF);
      Serial1.write(crc & 0xFF);

      delay(1000);
    }