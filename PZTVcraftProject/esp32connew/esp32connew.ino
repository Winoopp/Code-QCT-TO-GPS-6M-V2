#include <LoRa.h>
#define RXp2 16
#define TXp2 17
#include <Wire.h>
#include <TinyGPS++.h>

TinyGPSPlus gps;

int countstop = 0;

// QMC5883P address
const int QMC5883P_ADDR = 0x2C;
const int MODE_REG = 0x0A;
const int CONFIG_REG = 0x0B;
const int X_LSB_REG = 0x01;
const int X_MSB_REG = 0x02;
const int Y_LSB_REG = 0x03;
const int Y_MSB_REG = 0x04;
const int Z_LSB_REG = 0x05;
const int Z_MSB_REG = 0x06;

int16_t xMin = 32767, xMax = -32768;
int16_t yMin = 32767, yMax = -32768;
float xOffset = 0, yOffset = 0;
float xScale = 1, yScale = 1;

double targetLat = 17.358237;
double targetLon = 101.255716;

//___________________ FUNCTION __________________ //
void initQMC5883P() {
  Wire.begin();
  Wire.beginTransmission(QMC5883P_ADDR);
  Wire.write(MODE_REG);
  Wire.write(0xCF); // continuous mode, 200Hz
  Wire.endTransmission();

  Wire.beginTransmission(QMC5883P_ADDR);
  Wire.write(CONFIG_REG);
  Wire.write(0x08); // ±8G
  Wire.endTransmission();
}

void readQMC5883PData(int16_t &x, int16_t &y, int16_t &z) {
  Wire.beginTransmission(QMC5883P_ADDR);
  Wire.write(X_LSB_REG);
  Wire.endTransmission(false);
  Wire.requestFrom(QMC5883P_ADDR, 6);
  if (Wire.available() == 6) {
    byte x_lsb = Wire.read();
    byte x_msb = Wire.read();
    byte y_lsb = Wire.read();
    byte y_msb = Wire.read();
    byte z_lsb = Wire.read();
    byte z_msb = Wire.read();
    x = (x_msb << 8) | x_lsb;
    y = (y_msb << 8) | y_lsb;
    z = (z_msb << 8) | z_lsb;
  }
}

void updateCalibration(int16_t x, int16_t y) {
  if (x < xMin) xMin = x;
  if (x > xMax) xMax = x;
  if (y < yMin) yMin = y;
  if (y > yMax) yMax = y;
  xOffset = (xMax + xMin) / 2.0;
  yOffset = (yMax + yMin) / 2.0;
  float xRange = (xMax - xMin) / 2.0;
  float yRange = (yMax - yMin) / 2.0;
  float avgRange = (xRange + yRange) / 2.0;
  xScale = avgRange / xRange;
  yScale = avgRange / yRange;
}

double normalizeAngle(double angle) {
  while (angle > 180) angle -= 360;
  while (angle < -180) angle += 360;
  return angle;
}

float calculateHeading(int16_t x, int16_t y) {
  float xCal = (x - xOffset) * xScale;
  float yCal = (y - yOffset) * yScale;
  float heading = atan2(yCal, xCal) * 180.0 / PI;
  if (heading < 0) heading += 360;
  return heading;
}

//__________________ MAIN________________//

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, 4, -1); // RX GPS → D4
  Serial2.begin(9600, SERIAL_8N1, RXp2, TXp2);
  initQMC5883P();
  Serial.println("QMC5883P ready");
  LoRa.begin(433E6);
}

void loop() {
  // อ่าน GPS
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }

  // อ่าน compass
  int16_t x, y, z;
  readQMC5883PData(x, y, z);
  updateCalibration(x, y);
  double heading = calculateHeading(x, y);

  if (gps.location.isUpdated()) {
    double currLat = gps.location.lat();
    double currLon = gps.location.lng();

    double distance = TinyGPSPlus::distanceBetween(currLat, currLon, targetLat, targetLon);
    double bearing = TinyGPSPlus::courseTo(currLat, currLon, targetLat, targetLon);
    double diff = normalizeAngle(bearing - heading);

    Serial.println("-----------------------------");
    Serial.printf("Current : %.6f, %.6f\n", currLat, currLon);
    Serial.printf("Target  : %.6f, %.6f\n", targetLat, targetLon);
    Serial.printf("Distance: %.2f m\n", distance);
    Serial.printf("Bearing : %.2f°  |  Heading : %.2f°  |  Diff : %.2f°\n", bearing, heading, diff);

    // deadzone ปรับ ±25° → ลดการอ้อม
    const double DEADZONE = 25.0;
    const double STOP_DIST = 0.5; // หยุดเมื่อใกล้ waypoint ประมาณ 0.5m

    if (distance < STOP_DIST) {
      Serial2.println("ST");
      Serial.println("ST");
    } else if (diff > DEADZONE) {
      Serial2.println("TR");
      Serial.println("TR");
    } else if (diff < -DEADZONE) {
      Serial2.println("TL");
      Serial.println("TL");
    } else {
      Serial2.println("TW");
      Serial.println("TW");
    }

    countstop = 0;
  } else {
    // หาก GPS ขาดสัญญาณ
    countstop++;
    if (countstop > 70) { // timeout
      Serial2.println("ST");
      Serial.println("ST");
    }
    delay(100);
  }
  delay(50);
}
