#include <Arduino.h>
#include "RoboClaw.h"

// ---------------------------------------------------------------------
// RoboClaw wiring for Mega 2560
//
// Mega TX1 (pin 18) -> RoboClaw S1 signal
// Mega RX1 (pin 19) <- RoboClaw S2 signal   (optional but recommended)
// Mega GND          -> RoboClaw GND
//
// RoboClaw configured for:
// - Packet Serial
// - Baud rate 38400
// - Address 0x80
// ---------------------------------------------------------------------

// Create RoboClaw object on Serial1
RoboClaw roboclaw(&Serial1, 10000);

// Default RoboClaw address
static const uint8_t ROBOCLAW_ADDR = 0x80;

// Test speed: keep low for safety
static const uint8_t TEST_SPEED = 20;

// Delay between actions
static const unsigned long STEP_MS = 2000;

void stopAll() {
  roboclaw.ForwardM1(ROBOCLAW_ADDR, 0);
  roboclaw.ForwardM2(ROBOCLAW_ADDR, 0);
}

void testM1() {
  Serial.println("M1 forward");
  roboclaw.ForwardM1(ROBOCLAW_ADDR, TEST_SPEED);
  delay(STEP_MS);

  Serial.println("M1 stop");
  roboclaw.ForwardM1(ROBOCLAW_ADDR, 0);
  delay(1000);

  Serial.println("M1 backward");
  roboclaw.BackwardM1(ROBOCLAW_ADDR, TEST_SPEED);
  delay(STEP_MS);

  Serial.println("M1 stop");
  roboclaw.BackwardM1(ROBOCLAW_ADDR, 0);
  delay(1000);
}

void testM2() {
  Serial.println("M2 forward");
  roboclaw.ForwardM2(ROBOCLAW_ADDR, TEST_SPEED);
  delay(STEP_MS);

  Serial.println("M2 stop");
  roboclaw.ForwardM2(ROBOCLAW_ADDR, 0);
  delay(1000);

  Serial.println("M2 backward");
  roboclaw.BackwardM2(ROBOCLAW_ADDR, TEST_SPEED);
  delay(STEP_MS);

  Serial.println("M2 stop");
  roboclaw.BackwardM2(ROBOCLAW_ADDR, 0);
  delay(1000);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for USB serial on some boards
  }

  Serial.println("Starting RoboClaw library test...");

  Serial1.begin(38400);
  roboclaw.begin(38400);

  stopAll();
  delay(1000);

  Serial.println("Setup complete.");
  Serial.println("Wheels should be off the ground.");
}

void loop() {
  testM1();
  delay(1000);

  testM2();
  delay(2000);
}