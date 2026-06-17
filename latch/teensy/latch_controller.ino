// Latch controller for Arduino Nano 33 BLE Rev 2 (nRF52840)
//
// Wiring:
//   Servo signal  -> Pin D9  (PWM, 3.3 V logic — most servos accept this)
//   Reed switch   -> Pin D10 (INPUT_PULLUP; other leg to GND)
//
// Serial protocol (115200 baud over USB):
//   Receive from Jetson:  "<angle_deg>\n"   e.g. "90.0\n"
//   Send to Jetson:       "REED <0|1>\n"    sent on change + every 500 ms

#include <Servo.h>

static const int SERVO_PIN = 9;
static const int REED_PIN  = 10;

// Reed switch publish interval (ms) — guarantees state even if no change
static const unsigned long REED_INTERVAL_MS = 500;

Servo latchServo;

int           lastReedState  = -1;
unsigned long lastReedSendMs = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);  // wait for USB serial on Nano 33 BLE

  latchServo.attach(SERVO_PIN);
  latchServo.write(90);  // neutral on boot

  pinMode(REED_PIN, INPUT_PULLUP);
}

void loop() {
  // --- Receive servo angle command ---
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      float angle = line.toFloat();
      angle = constrain(angle, 0.0f, 180.0f);
      latchServo.write((int)angle);
    }
  }

  // --- Read reed switch (LOW = closed, because INPUT_PULLUP) ---
  int reedState = (digitalRead(REED_PIN) == LOW) ? 1 : 0;
  unsigned long now = millis();

  bool stateChanged    = (reedState != lastReedState);
  bool intervalElapsed = (now - lastReedSendMs >= REED_INTERVAL_MS);

  if (stateChanged || intervalElapsed) {
    Serial.print("REED ");
    Serial.println(reedState);
    lastReedState  = reedState;
    lastReedSendMs = now;
  }
}
