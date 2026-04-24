// 3rdparty/DFPlayer/Arduino_DFPlayer.ino

/*
  DFPlayer ADKEY bring-up sketch
  Arduino Mega
  ADKEY control: D6 via 4.7k + 100nF RC filter
  BUSY feedback: D2 optional

  Open Serial Monitor at 115200 baud.
*/

const int DF_ADKEY_PIN = 6;
const int DF_BUSY_PIN  = 2;

// Try these first. These are calibration values, not guaranteed final.
int testLevels[] = {
  20, 40, 60, 80, 100, 120, 140, 160, 190, 220
};

const int NUM_LEVELS = sizeof(testLevels) / sizeof(testLevels[0]);

int holdMs = 300;

void setup() {
  Serial.begin(115200);

  pinMode(DF_BUSY_PIN, INPUT_PULLUP);

  releaseAdkey();

  Serial.println("DFPlayer ADKEY test ready.");
  Serial.println("Commands:");
  Serial.println("  0-9 = send test ADKEY level");
  Serial.println("  s   = sweep all levels");
  Serial.println("  b   = read BUSY pin");
  Serial.println("  +   = increase hold time");
  Serial.println("  -   = decrease hold time");
  Serial.println();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();

    if (c >= '0' && c <= '9') {
      int index = c - '0';

      if (index < NUM_LEVELS) {
        Serial.print("Sending level ");
        Serial.print(index);
        Serial.print(" PWM=");
        Serial.println(testLevels[index]);

        sendAdkeyLevel(testLevels[index]);
        printBusy();
      }
    }

    else if (c == 's') {
      sweepLevels();
    }

    else if (c == 'b') {
      printBusy();
    }

    else if (c == '+') {
      holdMs += 50;
      Serial.print("Hold time now ");
      Serial.print(holdMs);
      Serial.println(" ms");
    }

    else if (c == '-') {
      holdMs -= 50;
      if (holdMs < 50) holdMs = 50;

      Serial.print("Hold time now ");
      Serial.print(holdMs);
      Serial.println(" ms");
    }
  }
}

void sendAdkeyLevel(int pwmValue) {
  pwmValue = constrain(pwmValue, 0, 255);

  pinMode(DF_ADKEY_PIN, OUTPUT);
  analogWrite(DF_ADKEY_PIN, pwmValue);

  delay(holdMs);

  releaseAdkey();

  delay(300);
}

void releaseAdkey() {
  analogWrite(DF_ADKEY_PIN, 0);
  pinMode(DF_ADKEY_PIN, INPUT);   // high impedance release
}

void sweepLevels() {
  Serial.println("Sweeping ADKEY levels...");

  for (int i = 0; i < NUM_LEVELS; i++) {
    Serial.print("Level ");
    Serial.print(i);
    Serial.print(" PWM=");
    Serial.println(testLevels[i]);

    sendAdkeyLevel(testLevels[i]);
    delay(1000);
  }

  Serial.println("Sweep complete.");
}

void printBusy() {
  int busy = digitalRead(DF_BUSY_PIN);

  Serial.print("BUSY pin = ");
  Serial.print(busy);

  if (busy == LOW) {
    Serial.println("  playing");
  } else {
    Serial.println("  idle");
  }
}