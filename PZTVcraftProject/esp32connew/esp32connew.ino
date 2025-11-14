#include <LoRa.h>
#include <SPI.h>
#define RXp2 16
#define TXp2 17
#include <Wire.h>
#include <TinyGPS++.h>
#define MAX_WAYPOINTS 4000

const int pwmPin18 = 18;
const int pwmPin19 = 19;
int duty_1ms = (int)(65535 * (1.0 / 20.0));
int duty_1_5ms = (int)(65535 * (1.5 / 20.0));
int duty_2ms = (int)(65535 * (2.0 / 20.0));

int currentWaypoint = 0; // เริ่มจาก waypoint แรก

struct Waypoint {
  int id;
  float lat;
  float lon;
};

Waypoint waypoints[MAX_WAYPOINTS];
int waypointCount = 0;


TinyGPSPlus gps;

#define SS 5 
#define RST 14 
#define DIO0 26

unsigned long lastSend = 0;
const unsigned long sendInterval = 100; // 100ms

int countstop = 0;
String msg = "";

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

bool gpsmode = false;
double targetLat = waypoints[currentWaypoint].lat;
double targetLon = waypoints[currentWaypoint].lon;


//___________________ FUNCTION __________________ //


int lastWaypoint = 0; // เริ่มต้น -1 เพื่อให้ waypoint 0 ผ่านทันที

void storeWaypoint(String msg) {
  if (waypointCount >= MAX_WAYPOINTS) {
    Serial.println("❌ เต็มแล้ว! ไม่สามารถเก็บ waypoint เพิ่มได้");
    return;
  }

  int i1 = msg.indexOf(',');
  int i2 = msg.indexOf(',', i1 + 1);
  if (i1 < 0 || i2 < 0) {
    Serial.println("⚠️ ข้อมูลไม่ถูกต้อง");
    return;
  }

  waypoints[waypointCount].id  = msg.substring(0, i1).toInt();
  waypoints[waypointCount].lat = msg.substring(i1 + 1, i2).toFloat();
  waypoints[waypointCount].lon = msg.substring(i2 + 1).toFloat();
  waypointCount++;

  Serial.print("✅ เก็บ Waypoint #");
  Serial.print(waypoints[waypointCount - 1].id);
  Serial.print(" : ");
  Serial.print(waypoints[waypointCount - 1].lat, 6);
  Serial.print(", ");
  Serial.println(waypoints[waypointCount - 1].lon, 6);
}
//____________________________________________________________________________________//
void receivePacket() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String packet = "";
    while (LoRa.available()) {
      packet += (char)LoRa.read();
    }

    Serial.print("Received packet: ");
    Serial.println(packet);

    // แยกค่าด้วย comma
    int firstComma = packet.indexOf(',');
    int secondComma = packet.indexOf(',', firstComma + 1);
    int thirdComma = packet.indexOf(',', secondComma + 1);

    if (firstComma == -1 || secondComma == -1 || thirdComma == -1) {
      Serial.println("Invalid packet format!");
      return;
    }

    int waypoint = packet.substring(0, firstComma).toInt();
    float lat = packet.substring(firstComma + 1, secondComma).toFloat();
    float lon = packet.substring(secondComma + 1, thirdComma).toFloat();
    float extra = packet.substring(thirdComma + 1).toFloat();

    bool packetOK = true;


    // ตรวจสอบ lat/lon เป็น 0
    if ((abs(lat) < 0.00001 && abs(lon) < 0.00001)) {
      Serial.println("Invalid coordinates! Request data again...");
      LoRa.beginPacket();
      LoRa.print("REQ,");
      LoRa.print(lastWaypoint + 1);
      LoRa.endPacket();
      packetOK = false;
    }

    // ตรวจสอบ waypoint ข้ามเลข ยกเว้น waypoint 0
    if (waypoint != 0 && waypoint != lastWaypoint + 1) {
      Serial.println("Waypoint skipped! Request previous data...");
      LoRa.beginPacket();
      LoRa.print("REQ,");
      LoRa.print(lastWaypoint + 1);
      LoRa.endPacket();
      packetOK = false;
    }

    // ถ้า packet ถูกต้อง ให้ตอบกลับ "waynext"
    if (packetOK) {
      Serial.println("Packet OK, sending 'waynext'");
      LoRa.beginPacket();
      LoRa.print("waynext");
      LoRa.endPacket();
      lastWaypoint = waypoint; // อัปเดต waypoint ล่าสุด
    }

    Serial.printf("Waypoint: %d, Lat: %.7f, Lon: %.7f, Extra: %.2f\n", waypoint, lat, lon, extra);
  }
}

//_______________________________________________________________________________________________________//

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

//__________________________________________________//
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
//__________________________________________________//
double normalizeAngle(double angle) {
  while (angle > 180) angle -= 360;
  while (angle < -180) angle += 360;
  return angle;
}
//__________________________________________________//
float calculateHeading(int16_t x, int16_t y) {
  float xCal = (x - xOffset) * xScale;
  float yCal = (y - yOffset) * yScale;
  float heading = atan2(yCal, xCal) * 180.0 / PI;
  if (heading < 0) heading += 360;
  return heading;
}
//__________________________________________________//
// ส่งเฉพาะ heading เป็น integer
void sendHeading(int heading) {
    LoRa.beginPacket();       // เริ่ม packet ใหม่
    LoRa.print(heading);      // ส่ง heading
    LoRa.endPacket();         // ปิด packet ส่งออก
}

//__________________________________________________//



//__________________ MAIN________________//

void setup() {
  Serial.println("Starting setup...");
  ledcAttach(pwmPin18, 50, 16);
  ledcAttach(pwmPin19, 50, 16);
  // กำหนดขา SPI ให้ตรงกับ ESP32
  SPI.begin(18, 19, 23, SS);  // SCK=18, MISO=19, MOSI=23, SS=5
  LoRa.setPins(SS, RST, DIO0);

  // หน่วงนิดหนึ่งก่อนเริ่ม LoRa
  delay(500);
  LoRa.begin(433E6);

  LoRa.setSpreadingFactor(11);   // ค่า 6-12 (ค่าเยอะ = ไกลขึ้นแต่ช้าลง)
  LoRa.setSignalBandwidth(125E3); // 125 kHz (เสถียรสุด)
  LoRa.setCodingRate4(5);        // 4/5 (ทนสัญญาณรบกวน)
  LoRa.setTxPower(20);           // ส่งแรงสุด (2–20 dBm)
  LoRa.disableCrc();             // ปิด CRC ถ้าส่งข้อมูลสั้น ๆ

  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, 4, -1); // RX GPS → D4
  Serial2.begin(9600, SERIAL_8N1, RXp2, TXp2);
  initQMC5883P();
  Serial.println("QMC5883P ready");

  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  Serial.println("LoRa Transmitter ready!");

}
//__________________________LOOP_______________________///

void loop() {
 
  receivePacket();


  int16_t x, y, z;
  readQMC5883PData(x, y, z);
  updateCalibration(x, y);

  int heading = calculateHeading(x, y);

  if(gpsmode){
    Serial2.println("gpson");
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
        // ไป waypoint ถัดไป
        if (currentWaypoint < waypointCount - 1) {
          currentWaypoint++;
          targetLat = waypoints[currentWaypoint].lat;
          targetLon = waypoints[currentWaypoint].lon;
        } else {
         Serial.println("✅ ถึง waypoint สุดท้าย");
        }
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
      if (countstop > 100) { // timeout
        ledcWrite(pwmPin18, duty_1_5ms);
        ledcWrite(pwmPin19, duty_1_5ms);
        Serial2.println("ST");
      }
      delay(50);
    }
  }else{
    Serial2.println("gpsoff");
  }
  delay(100);
}
