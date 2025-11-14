#include <HardwareSerial.h>
#include <SPI.h>
#include <LoRa.h>
#include <TinyGPS++.h>
#include <mavlink.h>
#include <Wire.h>
#define RXp2 16
#define TXp2 17
#define RMONPIN 33
#define Zonarpin 32

const int H1pin = 18;
const int H2pin = 19;
const int actuator = 23;
const int relaypin = 25;

int duty_1ms = (int)(65535 * (1.0 / 20.0));
int duty_1_5ms = (int)(65535 * (1.5 / 20.0));
int duty_2ms = (int)(65535 * (2.0 / 20.0));

TinyGPSPlus gps;

// ===== UART =====
HardwareSerial SerialBT(1);   // HC-06 Bluetooth
#define BT_TX 17
#define BT_RX 16
#define BT_BAUD 9600

struct Waypoint {
    int32_t lat;
    int32_t lon;
    int32_t alt_cm;
};

Waypoint wp[2000];
int wpCount = 0;
bool uploaded = false;


// ===== System ID =====
const uint8_t SYS_ID = 1;
const uint8_t COMP_ID = 1;

// ===== ตัวแปร =====
unsigned long lastHeartbeat = 0;
unsigned long lastGpsSend = 0;
uint16_t mission_total = 0;

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

bool isValidCoord(float lat, float lon) {
    if (isnan(lat) || isnan(lon)) return false;
    if (isinf(lat) || isinf(lon)) return false;

    // ป้องกันจุดกลางโลก / ค่าจาก KML เพี้ยน
    if (fabs(lat) < 0.000001 || fabs(lon) < 0.000001) return false;

    // ป้องกัน lat/lon เกินขอบเขตโลก
    if (lat < -90 || lat > 90) return false;
    if (lon < -180 || lon > 180) return false;

    return true;
}

void addWaypoint(float lat, float lon, float alt) {

    if (!isValidCoord(lat, lon)) {
        Serial.println("Invalid WP, skipping...");
        return;
    }

    wp[wpCount].lat = (int32_t)(lat * 1e7);
    wp[wpCount].lon = (int32_t)(lon * 1e7);
    wp[wpCount].alt_cm = (int32_t)(alt * 100);

    Serial.printf("Added WP %d  Lat:%.6f  Lon:%.6f\n", wpCount, lat, lon);

    wpCount++;
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
void send_mission_count(uint8_t target_sys, uint8_t target_comp, uint16_t count) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_mission_count_pack(
    SYS_ID, COMP_ID, &msg,
    target_sys, target_comp,
    count,
    MAV_MISSION_TYPE_MISSION,
    0
  );
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialBT.write(buf, len);
}

// ===== ฟังก์ชันตอบ ACK =====

void send_mission_ack(uint8_t target_sys, uint8_t target_comp, uint8_t result, uint8_t mission_type = MAV_MISSION_TYPE_MISSION) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  // ✅ ใช้ mission_type ตามจริง (Mission / Fence / Rally)
  mavlink_msg_mission_ack_pack(
    SYS_ID, COMP_ID, &msg,
    target_sys, target_comp,
    result,
    mission_type,  // <-- ปรับให้ยืดหยุ่น
    0
  );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialBT.write(buf, len);
  Serial.printf("✅ ส่ง MISSION_ACK (type=%d)\n", mission_type);

  delay(100); // ✅ ให้ QGC มีเวลารับก่อนเปลี่ยน phase
}

const int WP_COUNT = sizeof(wp) / sizeof(wp[0]);
int currentWp = 0;
double targetLat  = wp[0].lat;
double targetLon  = wp[0].lon;
bool missionDone  = false;

void gotoNextWaypoint() {
  currentWp++;
  if (currentWp >= WP_COUNT) {
    missionDone = true;
    Serial.println("Mission complete, no more waypoints.");
    ledcWrite(H1pin, duty_1_5ms);
    ledcWrite(H2pin, duty_1_5ms);
    return;
  }

  targetLat = wp[currentWp].lat;
  targetLon = wp[currentWp].lon;

  Serial.print("Next WP: ");
  Serial.print(currentWp);
  Serial.print("  Lat: ");
  Serial.print(targetLat, 6);
  Serial.print("  Lon: ");
  Serial.println(targetLon, 6);
}



void setup() {

  Serial.begin(115200);

  ledcAttach(H1pin, 50, 16);
  ledcAttach(H2pin, 50, 16);
  ledcAttach(actuator, 50, 16);
  pinMode(relaypin, OUTPUT);
  digitalWrite(relaypin, true);

  pinMode(RMONPIN, INPUT);
  pinMode(Zonarpin, INPUT);
  SerialBT.begin(BT_BAUD, SERIAL_8N1, RXp2, TXp2);
  delay(1000);
  Serial1.begin(9600, SERIAL_8N1, 4, -1); // RX GPS → D4
  Serial.println("✅ เริ่มระบบ ESP32 MAVLink + GPS จริง (NEO-8M)");
  Serial.println("✅ เริ่มระบบ ESP32 MAVLink ผ่าน HC-06 แล้ว");
  initQMC5883P();
  Serial.println("QMC5883P ready");
  
  send_heartbeat();

}

void loop() {
  
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
    delay(1);
  }
  unsigned long remoteon = pulseIn(RMONPIN, HIGH, 25000);
  int Zonar = analogRead(Zonarpin);

  Serial.print("zonar : ");
  Serial.println(Zonar);
  Serial.print("remoteon : ");
  Serial.println(remoteon);

  int16_t x, y, z;
  readQMC5883PData(x, y, z);
  updateCalibration(x, y);
  int heading = calculateHeading(x, y);

    if (remoteon < 1200 && uploaded) {
      digitalWrite(relaypin, false);
      if (gps.location.isUpdated()){
        
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
          if(Zonar > 2700) {
            if (distance < STOP_DIST) {
            Serial.println("Arrived WP, switching to next...");
            gotoNextWaypoint();   // <<< เปลี่ยนเป้าไปจุดถัดไป
            return;               // ออกก่อน รอบหน้า loop ค่อยคิดใหม่
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
          }
        } else {
          // หาก GPS ขาดสัญญาณ
          countstop++;
          if (countstop > 100) { // timeout
          ledcWrite(H1pin, duty_1_5ms);
          ledcWrite(H2pin, duty_1_5ms);
          Serial.println("GPS_Time_Out");
          }
          delay(50);
        }
        digitalWrite(relaypin, false);
      }

  while (SerialBT.available()) {
    uint8_t c = SerialBT.read();
    static mavlink_message_t msg;
    static mavlink_status_t status;
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
        Serial.printf("📨 ได้รับ msgid=%d\n", msg.msgid);

        // ===== PARAM_REQUEST_LIST =====
        if (msg.msgid == MAVLINK_MSG_ID_PARAM_REQUEST_LIST) {
            Serial.println("🧭 QGC ขอพารามิเตอร์ทั้งหมด -> ส่งกลับให้...");
            uint16_t count = 10; // จำนวนพารามิเตอร์ทั้งหมด
            send_param_value("SYSID_THISMAV", SYS_ID, MAV_PARAM_TYPE_UINT8, 0, count);
            send_param_value("MAV_TYPE", MAV_TYPE_GROUND_ROVER, MAV_PARAM_TYPE_UINT8, 1, count);
            send_param_value("MAV_AUTOPILOT", MAV_AUTOPILOT_ARDUPILOTMEGA, MAV_PARAM_TYPE_UINT8, 2, count);
            send_param_value("SYS_AUTOSTART", 1, MAV_PARAM_TYPE_INT32, 3, count);
            send_param_value("SYSID_MYGCS", 255, MAV_PARAM_TYPE_UINT8, 4, count);
            send_param_value("GPS_TYPE", 1, MAV_PARAM_TYPE_UINT8, 5, count);
            send_param_value("RTL_ALT", 10, MAV_PARAM_TYPE_INT32, 6, count);
            send_param_value("ARMING_CHECK", 0, MAV_PARAM_TYPE_UINT8, 7, count);
            send_param_value("FS_BATT_ENABLE", 0, MAV_PARAM_TYPE_UINT8, 8, count);
            send_param_value("FRAME_CLASS", 1, MAV_PARAM_TYPE_UINT8, 9, count);
            Serial.println("✅ ส่ง PARAM_VALUE ครบทั้งหมดแล้ว");
        }
        // ===== MISSION_REQUEST_LIST =====
        else if (msg.msgid == MAVLINK_MSG_ID_MISSION_REQUEST_LIST) {
            mavlink_mission_request_list_t req;
            mavlink_msg_mission_request_list_decode(&msg, &req);

            if (req.mission_type == MAV_MISSION_TYPE_MISSION) {
                Serial.println("🗺️ QGC ขอ Mission List -> ตอบกลับว่าไม่มี (0)");
                send_mission_count(req.target_system, req.target_component, 0);
            } else if (req.mission_type == MAV_MISSION_TYPE_FENCE) {
                Serial.println("🚧 QGC ขอ GeoFence -> ตอบกลับว่าไม่มี (0)");
                send_mission_count(req.target_system, req.target_component, 0);
            } else if (req.mission_type == MAV_MISSION_TYPE_RALLY) {
                Serial.println("🏁 QGC ขอ Rally Points -> ตอบกลับว่าไม่มี (0)");
                send_mission_count(req.target_system, req.target_component, 0);
            } else {
                Serial.printf("⚠️ QGC ขอ mission_type ไม่รู้จัก: %d\n", req.mission_type);
                send_mission_count(req.target_system, req.target_component, 0);
            }
        }
        // ===== MISSION_COUNT =====
        else if (msg.msgid == MAVLINK_MSG_ID_MISSION_COUNT) {
            mavlink_mission_count_t missionCount;
            mavlink_msg_mission_count_decode(&msg, &missionCount);
            mission_total = missionCount.count;

            Serial.printf("🗺️ QGC จะส่งภารกิจ %d จุด (type=%d)\n",
                          mission_total, missionCount.mission_type);

            if (missionCount.mission_type == MAV_MISSION_TYPE_RALLY) {
                Serial.println("🏁 QGC ส่ง Rally Point -> ตอบกลับว่ารับแล้ว (ไม่รองรับ)");
                send_mission_ack(missionCount.target_system, missionCount.target_component, MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_RALLY);
                Serial.println("✅ ส่ง ACK Rally กลับแล้ว");
            } else if (missionCount.mission_type == MAV_MISSION_TYPE_FENCE) {
                Serial.println("🚧 QGC ส่ง Fence -> ตอบกลับว่ารับแล้ว (ไม่รองรับ)");
                send_mission_ack(missionCount.target_system, missionCount.target_component, MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_FENCE);
                Serial.println("✅ ส่ง ACK Fence กลับแล้ว");
            } else if (missionCount.mission_type == MAV_MISSION_TYPE_MISSION) {
                Serial.println("🗺️ QGC ส่ง Mission ปกติ -> ขอ waypoint แรก");
                mavlink_message_t req;
                uint8_t buf[MAVLINK_MAX_PACKET_LEN];
                mavlink_msg_mission_request_int_pack(
                    SYS_ID, COMP_ID, &req,
                    missionCount.target_system,
                    missionCount.target_component,
                    0,
                    MAV_MISSION_TYPE_MISSION
                );
                uint16_t len = mavlink_msg_to_send_buffer(buf, &req);
                SerialBT.write(buf, len);
                Serial.println("📤 ขอ waypoint #0");
            }
        }
        // ===== MISSION_ITEM_INT =====
        else if (msg.msgid == MAVLINK_MSG_ID_MISSION_ITEM_INT) {
            mavlink_mission_item_int_t item;
            delay(200);
            mavlink_msg_mission_item_int_decode(&msg, &item);
            delay(200);

            float lat = item.x / 1e7;
            float lon = item.y / 1e7;
            float alt = item.z;

            Serial.printf("📍 Waypoint #%d lat=%.7f lon=%.7f alt=%.1f\n",
                          item.seq, lat, lon, alt);

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
                delay(400);
            } else {
                send_mission_ack(item.target_system, item.target_component, MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION);
                Serial.println("✅ ส่ง MISSION_ACK (ครบทุกจุด)");
                uploaded = true;
            }
        }
    }
  }
    // ส่ง Heartbeat ทุก 1 วิ
  if (millis() - lastHeartbeat > 700) {
    send_heartbeat();
    lastHeartbeat = millis();
  }
  delay(150);
}