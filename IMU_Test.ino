#include <Wire.h>
#include "ICM_20948.h"
#include <MadgwickAHRS.h>

ICM_20948_I2C imu;
Madgwick filter;

#define AD0_VAL_69 1   // I2C address 0x69
#define AD0_VAL_68 0   // I2C address 0x68

bool imuFound = false;
const float sampleRateHz = 50.0f;
const unsigned long samplePeriodMs = 20; // ~50 Hz
unsigned long lastSampleMs = 0;

bool beginIMU() {
  Serial.println("Trying IMU at 0x69...");
  imu.begin(Wire, AD0_VAL_69);
  if (imu.status == ICM_20948_Stat_Ok) {
    Serial.println("Sensor found at 0x69");
    return true;
  }

  Serial.println("0x69 failed. Trying 0x68...");
  imu.begin(Wire, AD0_VAL_68);
  if (imu.status == ICM_20948_Stat_Ok) {
    Serial.println("Sensor found at 0x68");
    return true;
  }

  return false;
}

void setup() {
  Serial.begin(115200);

  unsigned long startWait = millis();
  while (!Serial && (millis() - startWait < 5000)) {
    delay(10);
  }

  delay(1000);
  Serial.println();
  Serial.println("Starting ICM-20948 AHRS fusion...");
  Serial.println("Initializing I2C...");

  Wire.begin();
  Wire.setClock(400000); // optional but helpful if supported
  delay(100);

  imuFound = beginIMU();

  if (!imuFound) {
    Serial.println("IMU not detected!");
    Serial.println("Check VIN, GND, SDA, SCL and address.");
    while (1) {
      delay(1000);
      Serial.println("Still waiting: IMU not detected");
    }
  }

  filter.begin(sampleRateHz);

  Serial.println("CSV:");
  Serial.println("ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,mx_uT,my_uT,mz_uT,roll_deg,pitch_deg,yaw_deg");
}

void loop() {
  if (millis() - lastSampleMs < samplePeriodMs) {
    return;
  }
  lastSampleMs = millis();

  if (!imu.dataReady()) {
    return;
  }

  imu.getAGMT();

  // SparkFun library returns:
  // acc in mg, gyro in dps, mag in uT
  float ax = imu.accX() / 1000.0f; // convert mg -> g
  float ay = imu.accY() / 1000.0f;
  float az = imu.accZ() / 1000.0f;

  float gx = imu.gyrX(); // degrees/sec
  float gy = imu.gyrY();
  float gz = imu.gyrZ();

  float mx = imu.magX(); // microtesla
  float my = imu.magY();
  float mz = imu.magZ();

  // Quaternion-based fusion internally; Euler only for output
  filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);

  float roll  = filter.getRoll();
  float pitch = filter.getPitch();
  float yaw   = filter.getYaw();

  // Optional: shift yaw from [0, 360) style to [-180, 180)
  if (yaw > 180.0f) yaw -= 360.0f;

  Serial.print(millis());
  Serial.print(",");

  Serial.print(ax, 4); Serial.print(",");
  Serial.print(ay, 4); Serial.print(",");
  Serial.print(az, 4); Serial.print(",");

  Serial.print(gx, 3); Serial.print(",");
  Serial.print(gy, 3); Serial.print(",");
  Serial.print(gz, 3); Serial.print(",");

  Serial.print(mx, 3); Serial.print(",");
  Serial.print(my, 3); Serial.print(",");
  Serial.print(mz, 3); Serial.print(",");

  Serial.print(roll, 2); Serial.print(",");
  Serial.print(pitch, 2); Serial.print(",");
  Serial.println(yaw, 2);
}