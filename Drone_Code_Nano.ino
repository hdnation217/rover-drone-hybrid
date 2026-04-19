#include <Servo.h>

Servo esc2, esc3, esc4;

//const int ESC1_PIN = 3;
const int ESC2_PIN = 5;
const int ESC3_PIN = 6;
const int ESC4_PIN = 9;

const int ESC_MIN = 1000;
const int ESC_MAX = 2000;
const int ESC_IDLE = 1100;

// Toggle this to true ONLY when you want to calibrate
bool CALIBRATE = false;

void writeAllESCs(int pwm) {
  //esc1.writeMicroseconds(pwm);
  esc2.writeMicroseconds(pwm);
  esc3.writeMicroseconds(pwm);
  esc4.writeMicroseconds(pwm);
}

void setup() {
  Serial.begin(115200);

  //esc1.attach(ESC1_PIN, 1000, 2000);
  esc2.attach(ESC2_PIN, 1000, 2000);
  esc3.attach(ESC3_PIN, 1000, 2000);
  esc4.attach(ESC4_PIN, 1000, 2000);

  if (CALIBRATE) {
    Serial.println("=== CALIBRATION MODE ===");

    // Step 1: Send MAX before power
    writeAllESCs(ESC_MAX);
    Serial.println("Send MAX throttle to all ESCs. Now plug in ESC power.");

    delay(8000); // time for ESCs to detect max

    // Step 2: Send MIN
    writeAllESCs(ESC_MIN);
    Serial.println("Switching all ESCs to MIN throttle");

    delay(8000);

    Serial.println("Calibration complete.");
  } else {
    Serial.println("=== NORMAL MODE ===");

    // Arm all ESCs
    writeAllESCs(ESC_MIN);
    Serial.println("Arming all ESCs...");
    delay(8000);

    Serial.println("All ESCs armed.");
  }
}

void loop() {
  // Step 1: idle spin
  Serial.println("Idle spin");
  writeAllESCs(ESC_IDLE);
  delay(5000);

  // Step 2: increase throttle
  Serial.println("Increasing throttle");
  for (int pwm = 1150; pwm <= 2000; pwm += 10) {
    writeAllESCs(pwm);
    delay(300);
  }
  delay(5000);
  // Step 3: decrease throttle
  Serial.println("Decreasing throttle");
  for (int pwm = 2000; pwm >= 1100; pwm -= 10) {
    writeAllESCs(pwm);
    delay(300);
  }

  // Step 4: stop
  Serial.println("Stopping all motors");
  writeAllESCs(ESC_MIN);
  delay(5000);
}