#include <Arduino.h>
#include <ESP32Servo.h>

// D0 = pin 5 (pan,  SG90  blue)
// D1 = pin 6 (tilt, MG90S pink)
static const int PAN_PIN  = 5;
static const int TILT_PIN = 6;

static const int PAN_MIN  = 0,  PAN_MAX  = 180;
static const int TILT_MIN = 20, TILT_MAX = 160;

Servo panServo;
Servo tiltServo;

String rxBuf;

static int clampi(int v, int lo, int hi) {
  return v < lo ? lo : v > hi ? hi : v;
}

void applyCommand(int pan, int tilt) {
  pan  = clampi(pan,  PAN_MIN,  PAN_MAX);
  tilt = clampi(tilt, TILT_MIN, TILT_MAX);
  panServo.write(pan);
  tiltServo.write(tilt);
  Serial.print("pan="); Serial.print(pan);
  Serial.print(" tilt="); Serial.println(tilt);
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  panServo.setPeriodHertz(50);
  tiltServo.setPeriodHertz(50);
  // Full 500-2500 µs range → write(90) = true mechanical center (1500 µs)
  panServo.attach(PAN_PIN,   500, 2500);
  tiltServo.attach(TILT_PIN, 500, 2500);

  // Limit sweep so you can see the full physical range
  Serial.println(">> MIN (0 deg)");
  panServo.write(0);   tiltServo.write(0);   delay(1500);
  Serial.println(">> MAX (180 deg)");
  panServo.write(180); tiltServo.write(180); delay(1500);
  Serial.println(">> CENTER (90 deg = 1500 us)");

  applyCommand(100, 71);
  Serial.println("GUARD.IAM PTZ ready. Send: pan,tilt");
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxBuf.length() > 0) {
        int comma = rxBuf.indexOf(',');
        if (comma > 0) {
          int pan  = rxBuf.substring(0, comma).toInt();
          int tilt = rxBuf.substring(comma + 1).toInt();
          applyCommand(pan, tilt);
        }
        rxBuf = "";
      }
    } else {
      rxBuf += c;
      if (rxBuf.length() > 32) rxBuf = "";
    }
  }
}
