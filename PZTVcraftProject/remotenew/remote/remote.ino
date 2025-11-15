#include <HardwareSerial.h>
#include <SPI.h>
#include <LoRa.h>
#include <TinyGPS++.h>
#include <mavlink.h>
#include <Wire.h>
#include <math.h>
#include <Preferences.h>

Preferences prefs;

#define RXp2 17
#define TXp2 16
#define RMONPIN 33
#define Zonarpin 32

const int H1pin    = 18;
const int H2pin    = 19;
const int actuator = 23;
const int relaypin = 25;

int duty_1ms   = (int)(65535 * (1.0  / 20.0));
int duty_1_5ms = (int)(65535 * (1.5  / 20.0));
int duty_2ms   = (int)(65535 * (2.0  / 20.0));

TinyGPSPlus gps;

// ===== UART BT =====
HardwareSerial SerialBT(2);   // HC-06 Bluetooth
#define BT_BAUD 9600

// ===== Waypoint =====
struct Waypoint {
    int32_t lat_e7;   // lat * 1e7
    int32_t lon_e7;   // lon * 1e7
    int32_t alt_cm;   // alt * 100
};

Waypoint wp[2000];
int wpCount    = 0;
bool uploaded  = false;
bool missionDone = false;

// ===== System ID =====
const uint8_t SYS_ID  = 1;
const uint8_t COMP_ID = 1;

// ===== ตัวแปร =====
unsigned long lastHeartbeat = 0;
uint16_t mission_total = 0;

// QMC5883P address
const int QMC5883P_ADDR = 0x2C;
const int MODE_REG      = 0x0A;
const int CONFIG_REG    = 0x0B;
const int X_LSB_REG     = 0x01;
const int X_MSB_REG     = 0x02;
const int Y_LSB_REG     = 0x03;
const int Y_MSB_REG     = 0x04;
const int Z_LSB_REG     = 0x05;
const int Z_MSB_REG     = 0x06;

int16_t xMin = 32767, xMax = -32768;
int16_t yMin = 32767, yMax = -32768;
float xOffset = 0, yOffset = 0;
float xScale  = 1, yScale  = 1;

int   currentWp    = -1;      // -1 = ยังไม่มี
double targetLatDeg = 0;
double targetLonDeg = 0;

int countstop = 0;            // GPS timeout counter

//____________________________________________________FUNCTION_________________________________________________________________//

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

const int CALIB_BUTTON_PIN = 27;          // เลือกขาว่างสักขา
bool calibrating = false;
unsigned long calibStart = 0;
const unsigned long CALIB_DURATION_MS = 15000; // 15 วิ


void updateCalibration(int16_t x, int16_t y) {
  if (x < xMin) xMin = x;
  if (x > xMax) xMax = x;
  if (y < yMin) yMin = y;
  if (y > yMax) yMax = y;

  float xRange = (xMax - xMin) / 2.0;
  float yRange = (yMax - yMin) / 2.0;

  // ถ้ายังหมุนไม่พอ (ค่าเท่าเดิม) ไม่ต้องอัปเดต offset/scale
  if (xRange == 0 || yRange == 0) return;

  xOffset = (xMax + xMin) / 2.0;
  yOffset = (yMax + yMin) / 2.0;

  float avgRange = (xRange + yRange) / 2.0;
  xScale = avgRange / xRange;
  yScale = avgRange / yRange;
}


void saveCompassCalibration() {
  prefs.begin("compass", false);  // namespace "compass"
  prefs.putBool("hasCal", true);
  prefs.putInt("xMin", xMin);
  prefs.putInt("xMax", xMax);
  prefs.putInt("yMin", yMin);
  prefs.putInt("yMax", yMax);
  prefs.putFloat("xOffset", xOffset);
  prefs.putFloat("yOffset", yOffset);
  prefs.putFloat("xScale",  xScale);
  prefs.putFloat("yScale",  yScale);
  prefs.end();

  Serial.println("📥 Saved compass calibration to NVS");
}

bool loadCompassCalibration() {
  prefs.begin("compass", true);   // read-only
  bool hasCal = prefs.getBool("hasCal", false);
  if (hasCal) {
    xMin    = prefs.getInt("xMin");
    xMax    = prefs.getInt("xMax");
    yMin    = prefs.getInt("yMin");
    yMax    = prefs.getInt("yMax");
    xOffset = prefs.getFloat("xOffset");
    yOffset = prefs.getFloat("yOffset");
    xScale  = prefs.getFloat("xScale");
    yScale  = prefs.getFloat("yScale");
    prefs.end();

    Serial.println("📤 Loaded compass calibration from NVS");
    Serial.printf("xMin=%d xMax=%d yMin=%d yMax=%d\n", xMin, xMax, yMin, yMax);
    Serial.printf("xOffset=%.2f yOffset=%.2f xScale=%.4f yScale=%.4f\n",
                  xOffset, yOffset, xScale, yScale);
    return true;
  }
  prefs.end();
  Serial.println("⚠️ No saved compass calibration");
  return false;
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

bool isValidCoord(float lat, float lon) {
    if (isnan(lat) || isnan(lon)) return false;
    if (isinf(lat) || isinf(lon)) return false;

    // กันจุด (0,0) กับ noise เล็ก ๆ
    if (fabs(lat) < 0.000001 || fabs(lon) < 0.000001) return false;

    // ขอบเขตโลก
    if (lat < -90 || lat > 90) return false;
    if (lon < -180 || lon > 180) return false;

    return true;
}

void addWaypoint(float lat, float lon, float alt) {

    if (!isValidCoord(lat, lon)) {
        Serial.println("Invalid WP, skipping...");
        return;
    }

    if (wpCount >= 2000) {
        Serial.println("WP buffer full, skipping...");
        return;
    }

    wp[wpCount].lat_e7 = (int32_t)(lat * 1e7);
    wp[wpCount].lon_e7 = (int32_t)(lon * 1e7);
    wp[wpCount].alt_cm = (int32_t)(alt * 100);

    Serial.printf("Added WP %d  Lat:%.6f  Lon:%.6f Alt:%.1f\n",
                  wpCount, lat, lon, alt);

    // ถ้าเป็นจุดแรก → ตั้งเป็น target เริ่มต้นเลย
    if (wpCount == 0) {
        currentWp    = 0;
        targetLatDeg = lat;
        targetLonDeg = lon;
        missionDone  = false;
    }

    wpCount++;
}

void printWaypoints() {
  Serial.println("====== Waypoints Uploaded ======");
  for (int i = 0; i < wpCount; i++) {
    double lat = wp[i].lat_e7 / 1e7;
    double lon = wp[i].lon_e7 / 1e7;
    double alt = wp[i].alt_cm / 100.0;

    Serial.printf("#%d  Lat: %.7f  Lon: %.7f  Alt: %.2f m\n",
                  i, lat, lon, alt);
  }
  Serial.println("================================");
}

// ===== ฟังก์ชันส่ง Heartbeat =====
void send_heartbeat() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_heartbeat_pack(
    SYS_ID, COMP_ID, &msg,
    MAV_TYPE_GROUND_ROVER,
    MAV_AUTOPILOT_ARDUPILOTMEGA,
    MAV_MODE_GUIDED_ARMED,
    0,
    MAV_STATE_ACTIVE
  );
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialBT.write(buf, len);
}

// ===== ฟังก์ชันส่งพารามิเตอร์ =====
void send_param_value(const char* name, float value, uint8_t type, uint16_t index, uint16_t count) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_param_value_pack(SYS_ID, COMP_ID, &msg, name, value, type, count, index);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialBT.write(buf, len);
}

// ===== ฟังก์ชันส่งจำนวน mission =====
void send_mission_count(uint8_t target_sys, uint8_t target_comp, uint16_t count, uint8_t mission_type) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_mission_count_pack(
    SYS_ID, COMP_ID, &msg,
    target_sys, target_comp,
    count,
    mission_type,
    0
  );
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialBT.write(buf, len);
}


// ===== ฟังก์ชันตอบ ACK =====
void send_mission_ack(uint8_t target_sys, uint8_t target_comp, uint8_t result, uint8_t mission_type = MAV_MISSION_TYPE_MISSION) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_mission_ack_pack(
    SYS_ID, COMP_ID, &msg,
    target_sys, target_comp,
    result,
    mission_type,
    0
  );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialBT.write(buf, len);
  Serial.printf("✅ ส่ง MISSION_ACK (type=%d)\n", mission_type);

  delay(100);
}

void gotoNextWaypoint() {
  if (missionDone || wpCount == 0) return;

  currentWp++;
  if (currentWp >= wpCount) {
    missionDone = true;
    Serial.println("Mission complete, no more waypoints.");
    ledcWrite(H1pin, duty_1_5ms);
    ledcWrite(H2pin, duty_1_5ms);
    return;
  }

  targetLatDeg = wp[currentWp].lat_e7 / 1e7;
  targetLonDeg = wp[currentWp].lon_e7 / 1e7;

  Serial.print("Next WP: ");
  Serial.print(currentWp);
  Serial.print("  Lat: ");
  Serial.print(targetLatDeg, 6);
  Serial.print("  Lon: ");
  Serial.println(targetLonDeg, 6);
}

//____________________________________________________SETUP / LOOP___________________________________________________________//

void setup() {
  

  Serial.begin(115200);

  // คุณใช้ ledcAttach() ได้อยู่แล้ว → ใช้ต่อไป
  ledcAttach(H1pin, 50, 16);
  ledcAttach(H2pin, 50, 16);
  ledcAttach(actuator, 50, 16);

  pinMode(relaypin, OUTPUT);
  digitalWrite(relaypin, true);

  pinMode(RMONPIN, INPUT);
  pinMode(Zonarpin, INPUT);
    pinMode(CALIB_BUTTON_PIN, INPUT_PULLUP); // ปุ่มต่อ GND → LOW = กด

  SerialBT.begin(BT_BAUD, SERIAL_8N1, RXp2, TXp2);
  delay(1000);
  Serial1.begin(9600, SERIAL_8N1, 4, -1); // RX GPS → D4

  Serial.println("✅ เริ่มระบบ ESP32 MAVLink + GPS จริง (NEO-8M)");
  Serial.println("✅ เริ่มระบบ ESP32 MAVLink ผ่าน HC-06 แล้ว");
  
  initQMC5883P();
  loadCompassCalibration();
  Serial.println("QMC5883P ready");
  
  send_heartbeat();
}

void loop() {

  if (millis() - lastHeartbeat > 1000) {
    send_heartbeat();
    lastHeartbeat = millis();
  }

  // ---------- 1) เช็คปุ่ม เริ่มโหมดคาลิเบรต ----------
  static bool lastBtn = HIGH;
  bool btn = digitalRead(CALIB_BUTTON_PIN);

  if (!calibrating && lastBtn == HIGH && btn == LOW) {
    // กดจากไม่กด -> กด
    calibrating = true;
    calibStart = millis();

    // รีเซ็ต min/max ใหม่เพื่อเก็บจากศูนย์
    xMin = yMin =  32767;
    xMax = yMax = -32768;

    Serial.println("=== START COMPASS CALIBRATION ===");
    Serial.println("หมุนรถ/กล่องรอบตัวเองช้า ๆ ประมาณ 10–15 วิ");
  }
  lastBtn = btn;

  // ---------- 2) ถ้าอยู่ในโหมดคาลิเบรต ----------
  if (calibrating) {
    int16_t x, y, z;
    readQMC5883PData(x, y, z);
    updateCalibration(x, y);  // ใช้ฟังก์ชันเดิมของคุณ

    // กัน divide-by-zero ถ้ายังหมุนไม่พอ
    // (optional safety – ใส่ใน updateCalibration() ก็ได้)
    // float xRange = (xMax - xMin) / 2.0;
    // float yRange = (yMax - yMin) / 2.0;
    // if (xRange == 0 || yRange == 0) return;

    if (millis() - calibStart > CALIB_DURATION_MS) {
      calibrating = false;

      Serial.println("=== CALIBRATION DONE ===");
      Serial.printf("xMin=%d xMax=%d yMin=%d yMax=%d\n", xMin, xMax, yMin, yMax);
      Serial.printf("xOffset=%.2f yOffset=%.2f xScale=%.4f yScale=%.4f\n",
                    xOffset, yOffset, xScale, yScale);

      saveCompassCalibration();  // 💾 เก็บลง flash
      Serial.println("ค่าคาลิเบรตถูกบันทึกแล้ว ใช้ต่อได้จนกว่าจะกดคาลิเบรตใหม่");
    }

    // ระหว่างคาลิเบรต: หยุดรถ / ปลดรีเลย์
    ledcWrite(H1pin, duty_1_5ms);
    ledcWrite(H2pin, duty_1_5ms);
    digitalWrite(relaypin, true);

    delay(50);
    return;    // ⛔ ออกจาก loop เลย รอบนี้ไม่ต้องนำทาง / ไม่ต้องอ่าน MAVLink
  }
    // ===== รับ MAVLink จาก QGC =====
   while (SerialBT.available()) {
    uint8_t c = SerialBT.read();
    static mavlink_message_t msg;
    static mavlink_status_t status;
  
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      Serial.printf("📨 ได้รับ msgid=%d\n", msg.msgid);
      if (msg.msgid == MAVLINK_MSG_ID_PARAM_REQUEST_LIST) {
        Serial.println("🧭 QGC ขอ PARAM → ส่งกลับ...");
        uint16_t count = 10;
        send_param_value("SYSID_THISMAV", SYS_ID, MAV_PARAM_TYPE_UINT8, 0, count);
        send_param_value("MAV_TYPE",       MAV_TYPE_GROUND_ROVER,      MAV_PARAM_TYPE_UINT8, 1, count);
        send_param_value("MAV_AUTOPILOT",  MAV_AUTOPILOT_ARDUPILOTMEGA,MAV_PARAM_TYPE_UINT8, 2, count);
        send_param_value("SYS_AUTOSTART",  1,                          MAV_PARAM_TYPE_INT32, 3, count);
        send_param_value("SYSID_MYGCS",    255,                        MAV_PARAM_TYPE_UINT8, 4, count);
        send_param_value("GPS_TYPE",       1,                          MAV_PARAM_TYPE_UINT8, 5, count);
        send_param_value("RTL_ALT",        10,                         MAV_PARAM_TYPE_INT32, 6, count);
        send_param_value("ARMING_CHECK",   0,                          MAV_PARAM_TYPE_UINT8, 7, count);
        send_param_value("FS_BATT_ENABLE", 0,                          MAV_PARAM_TYPE_UINT8, 8, count);
        send_param_value("FRAME_CLASS",    1,                          MAV_PARAM_TYPE_UINT8, 9, count);
        Serial.println("✅ ส่ง PARAM_VALUE ครบแล้ว");
      }
      // MISSION_REQUEST_LIST (QGC ขอ mission ที่มีอยู่) → บอกว่า 0
      else if (msg.msgid == MAVLINK_MSG_ID_MISSION_REQUEST_LIST) {
        mavlink_mission_request_list_t req;
        mavlink_msg_mission_request_list_decode(&msg, &req);

        Serial.printf("🗺️ MISSION_REQUEST_LIST type=%d\n", req.mission_type);

        // บอกว่า "ไม่มี mission ของ type นี้" อย่างถูกต้อง
        send_mission_count(req.target_system,
                          req.target_component,
                          0,                  // ไม่มีภารกิจเก่า
                          req.mission_type);  // ใช้ type ตามที่เค้าถาม
      }

      else if (msg.msgid == MAVLINK_MSG_ID_MISSION_COUNT) {
        mavlink_mission_count_t missionCount;
        mavlink_msg_mission_count_decode(&msg, &missionCount);

        Serial.printf("🗺️ QGC จะส่งภารกิจ %d จุด (type=%d)\n",
                      missionCount.count, missionCount.mission_type);

        if (missionCount.mission_type == MAV_MISSION_TYPE_MISSION) {
          // ✅ รีเซ็ตเฉพาะ mission ปกติ
          mission_total = missionCount.count;

          wpCount     = 0;
          currentWp   = -1;
          missionDone = false;
          uploaded    = false;

          // ขอ waypoint แรก
          mavlink_message_t req;
          uint8_t buf[MAVLINK_MAX_PACKET_LEN];
          mavlink_msg_mission_request_int_pack(
              SYS_ID, COMP_ID, &req,
              missionCount.target_system,
              missionCount.target_component,
              0,  // ขอ seq 0
              MAV_MISSION_TYPE_MISSION
          );
          uint16_t len = mavlink_msg_to_send_buffer(buf, &req);
          SerialBT.write(buf, len);
          Serial.println("📤 ขอ waypoint #0");
        }
        else if (missionCount.mission_type == MAV_MISSION_TYPE_FENCE ||
                missionCount.mission_type == MAV_MISSION_TYPE_RALLY) {
          // ❌ Fence / Rally → แค่ตอบว่าไม่รองรับ
          Serial.printf("⚠️ Mission type %d not supported (Fence/Rally)\n",
                        missionCount.mission_type);

          send_mission_ack(
              missionCount.target_system,
              missionCount.target_component,
              MAV_MISSION_UNSUPPORTED,
              missionCount.mission_type
          );

          // ⛔️ ไม่ต้องแตะ wpCount / uploaded / missionDone เลย
        }
        else {
          // type แปลก ๆ: ปัดตกแบบสุภาพ
          Serial.printf("⚠️ Unknown mission_type=%d\n", missionCount.mission_type);
          send_mission_ack(
              missionCount.target_system,
              missionCount.target_component,
              MAV_MISSION_UNSUPPORTED,
              missionCount.mission_type
          );
        }
      }

      // MISSION_ITEM_INT (QGC ส่ง waypoint ทีละจุด)
      else if (msg.msgid == MAVLINK_MSG_ID_MISSION_ITEM_INT) {
        mavlink_mission_item_int_t item;
        mavlink_msg_mission_item_int_decode(&msg, &item);

        float lat = item.x / 1e7;
        float lon = item.y / 1e7;
        float alt = item.z;

        Serial.printf("📍 WP #%d cmd=%d lat=%.7f lon=%.7f alt=%.1f\n",
                      item.seq, item.command, lat, lon, alt);

        // เก็บเป็น waypoint นำทาง (จะฟิลเตอร์ cmd เพิ่มก็ได้)
        addWaypoint(lat, lon, alt);

        if (item.seq < mission_total - 1) {
          mavlink_message_t reqNext;
          uint8_t buf2[MAVLINK_MAX_PACKET_LEN];
          mavlink_msg_mission_request_int_pack(
              SYS_ID, COMP_ID, &reqNext,
              item.target_system,
              item.target_component,
              item.seq + 1,
              MAV_MISSION_TYPE_MISSION
          );
          uint16_t len2 = mavlink_msg_to_send_buffer(buf2, &reqNext);
          SerialBT.write(buf2, len2);
          Serial.printf("📤 ขอ waypoint #%d ถัดไป\n", item.seq + 1);
        } else {
          send_mission_ack(item.target_system,
                           item.target_component,
                           MAV_MISSION_ACCEPTED,
                           MAV_MISSION_TYPE_MISSION);
          Serial.println("✅ MISSION_ACK (ครบทุกจุด)");
          uploaded = (wpCount > 0);
          if (uploaded) {
            Serial.printf("✅ Mission uploaded, total WP = %d\n", wpCount);
            printWaypoints();
          } else {
            Serial.println("⚠️ Mission uploaded but no valid waypoint");
          }
        }
      }
    }
  }

  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }

  unsigned long remoteon = pulseIn(RMONPIN, HIGH, 25000);
  int Zonar = analogRead(Zonarpin);
  // เข็มทิศ
  int16_t x, y, z;
  readQMC5883PData(x, y, z);
  float heading = calculateHeading(x, y);
    Serial.println("_____________________");
    Serial.print("heading : ");
    Serial.println(heading);
    Serial.println("_____________________");

  // โหมดอัตโนมัติ: รีโมท < 1200, mission upload แล้ว, ยังไม่จบ
    // โหมดอัตโนมัติ: รีโมท < 1200, mission upload แล้ว, ยังไม่จบ
  Serial.printf("[STATE] uploaded=%d missionDone=%d wpCount=%d currentWp=%d\n", uploaded, missionDone, wpCount, currentWp);

  if (remoteon < 1300 && uploaded && !missionDone && wpCount > 0) {
    digitalWrite(relaypin, false);
    if (gps.location.isUpdated()) {
      countstop = 0;  // รีเซ็ต timeout

      double currLat = gps.location.lat();
      double currLon = gps.location.lng();

      double distance = TinyGPSPlus::distanceBetween(currLat, currLon, targetLatDeg, targetLonDeg);
      double bearing  = TinyGPSPlus::courseTo(currLat, currLon, targetLatDeg, targetLonDeg);
      double diff     = normalizeAngle(bearing - heading);

      Serial.println("-----------------------------");
      Serial.printf("Current : %.6f, %.6f\n", currLat, currLon);
      Serial.printf("Target  : %.6f, %.6f\n", targetLatDeg, targetLonDeg);
      Serial.printf("Distance: %.2f m\n", distance);
      Serial.printf("Bearing : %.2f°  |  Heading : %.2f°  |  Diff : %.2f°\n", bearing, heading, diff);

      const double DEADZONE  = 25.0;
      const double STOP_DIST = 1.2; // 0.5m รอบจุด

      if (Zonar > 2700) {
        if (distance < STOP_DIST) {
          Serial.println("Arrived WP, switching to next...");
          gotoNextWaypoint();
          return;  // ออก loop รอบนี้ รอรอบหน้าใช้เป้าใหม่
        }

        // ยังไม่ถึง → คุมทิศ / วิ่งต่อ
        if (diff > DEADZONE) {
          ledcWrite(H1pin, duty_1_5ms);
          ledcWrite(H2pin, duty_2ms);
          Serial.println("TR");
        } else if (diff < -DEADZONE) {
          ledcWrite(H1pin, duty_1_5ms);
          ledcWrite(H2pin, duty_1ms);
          Serial.println("TL");
        } else {
          ledcWrite(H1pin, duty_2ms);
          ledcWrite(H2pin, duty_1_5ms);
          Serial.println("TW");
        }
      } else {
        // Zonar block → หยุด
        ledcWrite(H1pin, duty_1_5ms);
        ledcWrite(H2pin, duty_1_5ms);
        Serial.println("Zonar Block");
      }

    } else {
      // GPS ไม่อัปเดต
      countstop++;
      if (countstop > 100) { // timeout
        ledcWrite(H1pin, duty_1_5ms);
        ledcWrite(H2pin, duty_1_5ms);
        Serial.println("GPS_Time_Out");
      }
      delay(50);
    }

  } else {
    // manual / ยังไม่ upload mission / missionDone
    ledcWrite(H1pin, duty_1_5ms);
    ledcWrite(H2pin, duty_1_5ms);
    digitalWrite(relaypin, true);
  }

  delay(70);
}