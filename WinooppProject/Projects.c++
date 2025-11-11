#include <HardwareSerial.h>
#include <TinyGPS++.h>
extern "C" {
  #include "common/mavlink.h"
}

// ===== UART =====
HardwareSerial SerialBT(1);   // HC-06 Bluetooth
#define BT_TX 26
#define BT_RX 27
#define BT_BAUD 9600

HardwareSerial SerialGPS(2);  // GPS UART
#define GPS_TX 18
#define GPS_RX 19
#define GPS_BAUD 9600

TinyGPSPlus gps;

// ===== System ID =====
const uint8_t SYS_ID = 1;
const uint8_t COMP_ID = 1;

// ===== ตัวแปร =====
unsigned long lastHeartbeat = 0;
unsigned long lastGpsSend = 0;
uint16_t mission_total = 0;

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


// ===== ฟังก์ชันส่ง GPS จริง =====
void send_gps_raw(double lat, double lon, double alt) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_gps_raw_int_pack(
    SYS_ID, COMP_ID, &msg,
    micros(),
    3,
    (int32_t)(lat * 1e7),
    (int32_t)(lon * 1e7),
    (int32_t)(alt * 1000),
    100, 100, 0, 0, 10,
    (int32_t)(alt * 1000),
    500, 500, 0, 0, 0
  );
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
void send_mission_ack(uint8_t target_sys, uint8_t target_comp, uint8_t result) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_mission_ack_pack(
    SYS_ID, COMP_ID, &msg,
    target_sys, target_comp,
    result,
    MAV_MISSION_TYPE_MISSION,
    0
  );
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialBT.write(buf, len);
  Serial.println("✅ ส่ง MISSION_ACK (QGC จะเลิก error แล้ว)");
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin(BT_BAUD, SERIAL_8N1, BT_RX, BT_TX);
  delay(1000);
  SerialGPS.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("✅ เริ่มระบบ ESP32 MAVLink + GPS จริง (NEO-6M)");
  Serial.println("✅ เริ่มระบบ ESP32 MAVLink ผ่าน HC-06 แล้ว");
  
  send_heartbeat();
}

void loop() {
  // ส่ง Heartbeat ทุก 1 วิ
  if (millis() - lastHeartbeat > 1000) {
    send_heartbeat();
    lastHeartbeat = millis();
  }
    // อ่านข้อมูลจาก GPS
  while (SerialGPS.available()) {
    gps.encode(SerialGPS.read());
  }

  // ส่ง GPS ทุก 2 วิ
  if (millis() - lastGpsSend > 1) {
    if (gps.location.isValid()) {
      double lat = gps.location.lat();
      double lon = gps.location.lng();
      double alt = gps.altitude.meters();
      Serial.printf("📡 GPS จริง lat=%.6f lon=%.6f alt=%.1f\n", lat, lon, alt);
      send_gps_raw(lat, lon, alt);
    } else {
      Serial.println("❌ ยังไม่เจอสัญญาณ GPS...");
    }
    lastGpsSend = millis();
  }


  // อ่านข้อมูลจาก QGC
  while (SerialBT.available()) {
    uint8_t c = SerialBT.read();
    static mavlink_message_t msg;
    static mavlink_status_t status;

    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      Serial.printf("📨 ได้รับ msgid=%d\n", msg.msgid);

  switch (msg.msgid) {

      // ✅ ตอบกลับ PARAM_REQUEST_LIST
  case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
    Serial.println("🧭 QGC ขอพารามิเตอร์ทั้งหมด -> ส่งกลับให้...");
    uint16_t count = 10;  // จำนวนพารามิเตอร์ทั้งหมดที่เราจะส่งกลับ

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
    break;
  }
  // ============ QGC ขอ Mission List ===============
  case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: {
  mavlink_mission_request_list_t req;
  mavlink_msg_mission_request_list_decode(&msg, &req);

  // ตรวจประเภทของ mission
  switch (req.mission_type) {

    // ปกติ (Mission waypoints)
    case MAV_MISSION_TYPE_MISSION:
      Serial.println("🗺️ QGC ขอ Mission List -> ตอบกลับว่าไม่มี (0)");
      send_mission_count(req.target_system, req.target_component, 0);
      break;

    // กำแพงจำกัดพื้นที่ (GeoFence)
    case MAV_MISSION_TYPE_FENCE:
      Serial.println("🚧 QGC ขอ GeoFence -> ตอบกลับว่าไม่มี (0)");
      {
        mavlink_message_t msgCount;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        mavlink_msg_mission_count_pack(
          SYS_ID, COMP_ID, &msgCount,
          req.target_system,
          req.target_component,
          0,  // จำนวน = 0
          MAV_MISSION_TYPE_FENCE,
          0
        );
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msgCount);
        SerialBT.write(buf, len);
      }
      break;

    // จุดกลับฉุกเฉิน (Rally Point)
    case MAV_MISSION_TYPE_RALLY:
      Serial.println("🏁 QGC ขอ Rally Points -> ตอบกลับว่าไม่มี (0)");
      {
        mavlink_message_t msgCount;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        mavlink_msg_mission_count_pack(
          SYS_ID, COMP_ID, &msgCount,
          req.target_system,
          req.target_component,
          0,  // จำนวน = 0
          MAV_MISSION_TYPE_RALLY,
          0
        );
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msgCount);
        SerialBT.write(buf, len);
      }
      break;

    default:
      Serial.printf("⚠️ QGC ขอ mission_type ไม่รู้จัก: %d\n", req.mission_type);
      send_mission_count(req.target_system, req.target_component, 0);
      break;
  }

  break;
}


  // ============ QGC แจ้งว่าจะส่ง Mission (รวม Rally/Fence) ============
  case MAVLINK_MSG_ID_MISSION_COUNT: {
    mavlink_mission_count_t missionCount;
    mavlink_msg_mission_count_decode(&msg, &missionCount);
    mission_total = missionCount.count;
    Serial.printf("🗺️ QGC จะส่งภารกิจ %d จุด (type=%d)\n",
                  mission_total, missionCount.mission_type);

    // Rally Point
    if (missionCount.mission_type == MAV_MISSION_TYPE_RALLY) {
      Serial.println("🏁 QGC ส่ง Rally Point -> ตอบกลับว่ารับแล้ว (ไม่รองรับ)");
      mavlink_message_t ack;
      uint8_t buf[MAVLINK_MAX_PACKET_LEN];
      mavlink_msg_mission_ack_pack(
        SYS_ID, COMP_ID, &ack,
        missionCount.target_system,
        missionCount.target_component,
        MAV_MISSION_ACCEPTED,
        MAV_MISSION_TYPE_RALLY,
        0
      );
      uint16_t len = mavlink_msg_to_send_buffer(buf, &ack);
      SerialBT.write(buf, len);
      Serial.println("✅ ส่ง ACK Rally กลับแล้ว");
      break;
    }

    // Fence
    if (missionCount.mission_type == MAV_MISSION_TYPE_FENCE) {
      Serial.println("🚧 QGC ส่ง Fence -> ตอบกลับว่ารับแล้ว (ไม่รองรับ)");
      mavlink_message_t ack;
      uint8_t buf[MAVLINK_MAX_PACKET_LEN];
      mavlink_msg_mission_ack_pack(
        SYS_ID, COMP_ID, &ack,
        missionCount.target_system,
        missionCount.target_component,
        MAV_MISSION_ACCEPTED,
        MAV_MISSION_TYPE_FENCE,
        0
      );
      uint16_t len = mavlink_msg_to_send_buffer(buf, &ack);
      SerialBT.write(buf, len);
      Serial.println("✅ ส่ง ACK Fence กลับแล้ว");
      break;
    }

    // Mission ปกติ
    if (missionCount.mission_type == MAV_MISSION_TYPE_MISSION) {
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
      break;
    }

    break; // จบ case MISSION_COUNT
  }

  // ============ QGC ส่ง Waypoint แต่ละจุด ============
  case MAVLINK_MSG_ID_MISSION_ITEM_INT: {
    mavlink_mission_item_int_t item;
    mavlink_msg_mission_item_int_decode(&msg, &item);
    Serial.printf("📍 Waypoint #%d lat=%.7f lon=%.7f alt=%.1f\n",
                  item.seq, item.x / 1e7, item.y / 1e7, item.z);

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
      // ✅ เมื่อได้รับครบทุกจุดแล้ว
      send_mission_ack(item.target_system, item.target_component, MAV_MISSION_ACCEPTED);
      Serial.println("✅ ส่ง MISSION_ACK (ครบทุกจุด)");
    }
    break;
  }

  default:
    break;
}


    }
  }
}
