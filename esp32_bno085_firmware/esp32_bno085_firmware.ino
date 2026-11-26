// ESP32 + BNO085 (I2C) -> USB serial streamer for the ROS 2 bno085_driver package.
//
// Wiring: BNO085 SDA -> GPIO21, SCL -> GPIO22, VIN -> 3V3, GND -> GND.
// Library: "Adafruit BNO08x" (Library Manager; also installs Adafruit BusIO / Unified Sensor).
// Board:   any "ESP32 Dev Module". Serial monitor is NOT needed; the ROS node reads the port.
//
// Output line (ASCII, 921600 baud), one per rotation-vector report:
//   $IMU,<ms>,<qx>,<qy>,<qz>,<qw>,<gx>,<gy>,<gz>,<ax>,<ay>,<az>,<mx>,<my>,<mz>*<XOR checksum hex>
// quaternion unitless, gyro rad/s, accel m/s^2 (includes gravity), as in sensor_msgs/Imu;
// mag in uT (calibrated, hard-iron compensated by the BNO085).

#include <Wire.h>
#include <Adafruit_BNO08x.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define BNO_ADDR 0x4A          // 0x4B if the DI/address pin is pulled high
#define REPORT_INTERVAL_US 10000  // 100 Hz
#define BAUD 921600
#define I2C_RUN_HZ 400000     // drop to 100000 if you see I2C errors/garbled data

#define RESET_PIN -1           // set to a GPIO (e.g. 4) if BNO085 RST is wired, else -1
Adafruit_BNO08x bno(RESET_PIN);
sh2_SensorValue_t val;

float q[4] = {0, 0, 0, 1};     // x y z w
float gyro[3] = {0, 0, 0};
float acc[3] = {0, 0, 0};
float mag[3] = {0, 0, 0};

bool enableReports() {
  bool ok = bno.enableReport(SH2_ROTATION_VECTOR, REPORT_INTERVAL_US);
  ok &= bno.enableReport(SH2_GYROSCOPE_CALIBRATED, REPORT_INTERVAL_US);
  ok &= bno.enableReport(SH2_ACCELEROMETER, REPORT_INTERVAL_US);
  ok &= bno.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, REPORT_INTERVAL_US);
  return ok;
}

bool acks(uint8_t a) {
  Wire.beginTransmission(a);
  return Wire.endTransmission() == 0;
}

// Lists every I2C device that ACKs, to help diagnose wiring/address problems.
void scanI2C() {
  char buf[120] = "#BNO085 not found. I2C devices:";
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      snprintf(buf + strlen(buf), sizeof(buf) - strlen(buf), " 0x%02X", a);
      n++;
    }
  }
  if (n == 0) strncat(buf, " none (check SDA/SCL/power/pull-ups)", sizeof(buf) - strlen(buf) - 1);
  Serial.println(buf);
}

void setup() {
  Serial.begin(BAUD);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);  // slow clock for initialisation
  Wire.setTimeOut(100);

  delay(1000);  // let the BNO085 finish booting

  bool found = false;
  while (!found) {
    // Only try addresses that actually ACK, so the library's own error text stays quiet.
    for (uint8_t a : {(uint8_t)0x4A, (uint8_t)0x4B}) {
      if (!acks(a)) continue;
      if (bno.begin_I2C(a, &Wire)) {
        found = true;
        Wire.setClock(I2C_RUN_HZ);  // init at 100 kHz, run fast for 100 Hz throughput
        Serial.printf("#BNO085 found at 0x%02X\n", a);
        break;
      }
      Serial.printf("#device ACKs at 0x%02X but BNO08x init failed (try power-cycling / RST wire)\n", a);
    }
    if (!found) {
      Wire.setClock(100000);
      scanI2C();
      delay(1000);
    }
  }
  while (!enableReports()) {
    Serial.println("#enableReport failed, retrying");
    delay(500);
  }
  Serial.println("#BNO085 ready");
}

void sendLine() {
  char body[200];
  snprintf(body, sizeof(body), "IMU,%lu,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f",
           millis(), q[0], q[1], q[2], q[3], gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2],
           mag[0], mag[1], mag[2]);
  uint8_t cs = 0;
  for (const char *p = body; *p; ++p) cs ^= (uint8_t)*p;
  Serial.printf("$%s*%02X\n", body, cs);
}

void loop() {
  if (bno.wasReset()) {
    enableReports();
  }
  // Drain all pending events; emit one line per rotation vector update.
  while (bno.getSensorEvent(&val)) {
    switch (val.sensorId) {
      case SH2_ROTATION_VECTOR:
        q[0] = val.un.rotationVector.i;
        q[1] = val.un.rotationVector.j;
        q[2] = val.un.rotationVector.k;
        q[3] = val.un.rotationVector.real;
        sendLine();
        break;
      case SH2_GYROSCOPE_CALIBRATED:
        gyro[0] = val.un.gyroscope.x;
        gyro[1] = val.un.gyroscope.y;
        gyro[2] = val.un.gyroscope.z;
        break;
      case SH2_ACCELEROMETER:
        acc[0] = val.un.accelerometer.x;
        acc[1] = val.un.accelerometer.y;
        acc[2] = val.un.accelerometer.z;
        break;
      case SH2_MAGNETIC_FIELD_CALIBRATED:
        mag[0] = val.un.magneticField.x;
        mag[1] = val.un.magneticField.y;
        mag[2] = val.un.magneticField.z;
        break;
    }
  }
}
