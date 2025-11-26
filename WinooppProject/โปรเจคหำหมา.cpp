#include <HardwareSerial.h>
#include <SPI.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <math.h>
#include <Preferences.h>
#include <vector>
#include <TinyGPS++.h>


extern "C" {
  #include "common/mavlink.h"
}

// ===============================
//  I2C ADDRESS
// ===============================
#define ICM_ADDR 0x69
#define MAG_ADDR 0x0C

// ICM20600 REG
#define ACCEL_XOUT_H   0x3B
#define PWR_MGMT_1     0x6B
#define ACCEL_CONFIG   0x1C
#define GYRO_CONFIG    0x1B

// MAG REG (AK09918)
#define MAG_ST1        0x10
#define MAG_HXL        0x11
#define MAG_ST2        0x18
#define MAG_CNTL2      0x31

// ปุ่มคาลิเบรต
#define CAL_BUTTON 27

TinyGPSPlus gps;

#define RMONPIN 33
#define Zonarpin 32

const int actuator = 23;
const int relaypin = 25;

// UART ไปบอร์ดมอเตอร์
const int MotorRxPin = 18;
const int MotorTxPin = 19;

const uint8_t SYS_ID  = 1;
const uint8_t COMP_ID = 1;

uint16_t mission_expected = 0;
uint16_t mission_received = 0;
bool mission_active = false;


// ========= Bluetooth Serial =========
HardwareSerial SerialBT(2);
#define BT_RX 17
#define BT_TX 16
#define BT_BAUD 9600

unsigned long lastHeartbeat = 0;

/////////////////////////////////////////////////////// แอปQCT ////////////////////////////////////
// ====== เก็บ WP ทั้งหมดที่เป็นพิกัดจริง ======
struct Point {
  float lat;
  float lon;
  float alt;
};

Point all_wp[100];
int all_count = 0;

// ====== มุมจริง 4 มุม ======
Point corners[4];
int corner_count = 0;

// ระยะทางแต่ละด้าน
double distAB = 0;
double distBC = 0;
double distCD = 0;
double distDA = 0;

// ทิศทางแต่ละด้าน
float dirAB_global = 0;
float dirBC_global = 0;
float dirCD_global = 0;
float dirDA_global = 0;

// flag บอกว่า “คำนวณครบแล้ว”
bool cornerReady = false;





// ========= CROSS PRODUCT (Convex Hull) =========
double cross(const Point& O, const Point& A, const Point& B) {
    return (A.lon - O.lon) * (B.lat - O.lat)
         - (A.lat - O.lat) * (B.lon - O.lon);

}

// ===== คำนวณ Bearing ระหว่าง A → B =====
// ผลลัพธ์เป็นองศา (0–360)
// 0 = North, 90 = East, 180 = South, 270 = West
float bearingTo(float lat1, float lon1, float lat2, float lon2) {

    float lat1Rad = lat1 * M_PI / 180.0;
    float lat2Rad = lat2 * M_PI / 180.0;
    float dLon    = (lon2 - lon1) * M_PI / 180.0;

    float y = sin(dLon) * cos(lat2Rad);
    float x = cos(lat1Rad)*sin(lat2Rad) -
              sin(lat1Rad)*cos(lat2Rad)*cos(dLon);

    float brng = atan2(y, x) * 180.0 / M_PI;

    // ทำให้เป็น 0–360
    if (brng < 0) brng += 360.0;

    return brng;
}

// ===== คำนวณระยะทางระหว่าง 2 จุดบนโลก (หน่วย: เมตร) =====
double distanceM(float lat1, float lon1, float lat2, float lon2) {
    const double R = 6371000.0; // รัศมีโลก (เมตร)

    double lat1_rad = lat1 * M_PI / 180.0;
    double lat2_rad = lat2 * M_PI / 180.0;
    double dLat     = (lat2 - lat1) * M_PI / 180.0;
    double dLon     = (lon2 - lon1) * M_PI / 180.0;

    double a = sin(dLat/2.0) * sin(dLat/2.0) +
               cos(lat1_rad) * cos(lat2_rad) *
               sin(dLon/2.0) * sin(dLon/2.0);

    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    return R * c;  // ระยะทางเป็นเมตร
}


void send_mission_ack(uint8_t target_sys, uint8_t target_comp, uint8_t result, uint8_t mission_type) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_mission_ack_pack(
        SYS_ID, COMP_ID,
        &msg,
        target_sys,
        target_comp,
        result,
        mission_type,
        0  // unused
    );

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    SerialBT.write(buf, len);

    Serial.println("✔ ส่ง Mission ACK");
}


void send_statustext() {
    mavlink_message_t msg;
    uint8_t buf[100];

    mavlink_msg_statustext_pack(
        SYS_ID,
        COMP_ID,
        &msg,
        MAV_SEVERITY_INFO,     // severity
        "ESP32 Connected",     // text
        0,                     // id (0 = default)
        0                      // chunk_seq (0 = first chunk)
    );

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    SerialBT.write(buf, len);

    
}

// ========= หา 4 มุมจริง (Convex Hull) =========
void computeCorners() {

    if (all_count < 4) {
        Serial.println("❌ waypoint ไม่พอที่จะหามุม");
        return;
    }

    // sort ตาม lon, แล้ว lat
    std::sort(all_wp, all_wp + all_count, [](Point a, Point b){
        return a.lon < b.lon || (a.lon == b.lon && a.lat < b.lat);
    });

    // ทำ Hull
    std::vector<Point> H;

    // Lower hull
    for (int i = 0; i < all_count; i++) {
        while (H.size() >= 2 && cross(H[H.size()-2], H.back(), all_wp[i]) <= 0)
            H.pop_back();
        H.push_back(all_wp[i]);
    }

    // Upper hull
    int t = H.size()+1;
    for (int i = all_count - 1; i >= 0; i--) {
        while (H.size() >= t && cross(H[H.size()-2], H.back(), all_wp[i]) <= 0)
            H.pop_back();
        H.push_back(all_wp[i]);
    }

       corner_count = 0;
    Serial.println("========== มุมจริงของพื้นที่ ==========");
    for (int i = 0; i < (int)H.size() - 1 && corner_count < 4; i++) {
        corners[corner_count] = H[i];
        Serial.printf("📌 มุม[%d] = %.7f , %.7f\n", corner_count+1, H[i].lat, H[i].lon);
        corner_count++;
    }
    Serial.println("=======================================");

    // ถ้าได้ครบ 4 มุมแล้ว คำนวณระยะทาง 4 ด้าน
    if (corner_count == 4) {
        double BC = distanceM(corners[0].lat, corners[0].lon,
                              corners[1].lat, corners[1].lon);
        double CD = distanceM(corners[1].lat, corners[1].lon,
                              corners[2].lat, corners[2].lon);
        double DA = distanceM(corners[2].lat, corners[2].lon,
                              corners[3].lat, corners[3].lon);
        double AB = distanceM(corners[3].lat, corners[3].lon,
                              corners[0].lat, corners[0].lon);

        Serial.println("===== ระยะทางแต่ละด้าน (เมตร) =====");
        Serial.printf("📏 ด้าน AB = %.2f m\n", AB);
        Serial.printf("📏 ด้าน BC = %.2f m\n", BC);
        Serial.printf("📏 ด้าน CD = %.2f m\n", CD);
        Serial.printf("📏 ด้าน DA = %.2f m\n", DA);
        Serial.println("===================================");

               // ===================== คำนวณทิศทางของแต่ละด้าน =====================
        float dirBC = bearingTo(corners[0].lat, corners[0].lon,
                                corners[1].lat, corners[1].lon);

        float dirCD = bearingTo(corners[1].lat, corners[1].lon,
                                corners[2].lat, corners[2].lon);

        float dirDA = bearingTo(corners[2].lat, corners[2].lon,
                                corners[3].lat, corners[3].lon);

        float dirAB = bearingTo(corners[3].lat, corners[3].lon,
                                corners[0].lat, corners[0].lon);

        Serial.println("===== ทิศทาง (องศา 0-360) =====");
        Serial.printf("🧭 A → B = %.2f°\n", dirAB);
        Serial.printf("🧭 B → C = %.2f°\n", dirBC);
        Serial.printf("🧭 C → D = %.2f°\n", dirCD);
        Serial.printf("🧭 D → A = %.2f°\n", dirDA);
        Serial.println("================================");
            // === COPY ค่าออกไป Global ===

        distAB = AB;
        distBC = BC;
        distCD = CD;
        distDA = DA;

        dirAB_global = dirAB;
        dirBC_global = dirBC;
        dirCD_global = dirCD;
        dirDA_global = dirDA;

// แจ้งว่าใช้ได้แล้ว
cornerReady = true;

        
    } else {
        Serial.printf("⚠ ได้มุมจริงมา %d จุด ยังไม่ครบ 4 มุม\n", corner_count);
    }

Serial.printf("DEBUG distAB=%.2f distBC=%.2f distCD=%.2f distDA=%.2f\n",
    distAB, distBC, distCD, distDA);

}



// ========= HEARTBEAT =========
void send_heartbeat() {
  mavlink_message_t msg;
  uint8_t buf[50];

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


///////////////////////////////////////////////////////// App QCT /////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////// เข็มทิศ + ความเร่ง ////////////////////////////////////////////////////////////
// ===== STATE =====
float gyro_bias = 0;
float gyro_heading = 0;
float heading_filtered = 0;

float mag_offset_x = 0, mag_offset_y = 0, mag_offset_z = 0;
float mag_scale_x = 1, mag_scale_y = 1, mag_scale_z = 1;

unsigned long last_t;

// ===============================
// RAW I2C
// ===============================
int16_t read16(uint8_t addr, uint8_t reg){
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)addr,(uint8_t)2);
  return (Wire.read()<<8)|Wire.read();
}

uint8_t read8(uint8_t addr,uint8_t reg){
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)addr,(uint8_t)1);
  return Wire.read();
}

void write8(uint8_t addr,uint8_t reg,uint8_t val){
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

// ===============================
// GYRO CALIBRATION
// ===============================
void calibrateGyro(){
  Serial.println("Calibrating Gyro...");
  long sum = 0;
  for(int i=0;i<500;i++){
    int16_t gz = read16(ICM_ADDR, ACCEL_XOUT_H + 12);
    sum += gz;
    delay(3);
  }
  gyro_bias = (sum/500.0) / 65.5;
  Serial.print("Gyro bias = "); Serial.println(gyro_bias);
}

// ===============================
// MAG CALIBRATION
// ===============================
void calibrateMag(){
  Serial.println("Rotate sensor 8 seconds...");

  long t0 = millis();
  long dur = 1000;

  int32_t min_x =  1000000, max_x = -1000000;
  int32_t min_y =  1000000, max_y = -1000000;
  int32_t min_z =  1000000, max_z = -1000000;

  while(millis()-t0 < dur){
    if(read8(MAG_ADDR, MAG_ST1) & 0x01){
      int16_t mx = read16(MAG_ADDR,MAG_HXL);
      int16_t my = read16(MAG_ADDR,MAG_HXL+2);
      int16_t mz = read16(MAG_ADDR,MAG_HXL+4);
      read8(MAG_ADDR, MAG_ST2);

      if(mx < min_x) min_x = mx;
      if(mx > max_x) max_x = mx;

      if(my < min_y) min_y = my;
      if(my > max_y) max_y = my;

      if(mz < min_z) min_z = mz;
      if(mz > max_z) max_z = mz;

      Serial.print(".");
    }
    delay(5);
  }

  mag_offset_x = (max_x + min_x)/2.0;
  mag_offset_y = (max_y + min_y)/2.0;
  mag_offset_z = (max_z + min_z)/2.0;

  float rx = (max_x-min_x)/2.0;
  float ry = (max_y-min_y)/2.0;
  float rz = (max_z-min_z)/2.0;
  float avg=(rx+ry+rz)/3.0;

  mag_scale_x = avg/rx;
  mag_scale_y = avg/ry;
  mag_scale_z = avg/rz;

  Serial.println("\nMAG calibrated!");
}

 // =============================================
// ACCEL CONVERT — แปลงเป็น g-force
// =============================================
float accel_g_x, accel_g_y, accel_g_z;
float accel_total_g = 0;

// sensitivity ±4g = 8192 LSB/g
#define ACCEL_SENS 8192.0

void processAccel(int16_t ax, int16_t ay, int16_t az) {
    accel_g_x = ax / ACCEL_SENS;
    accel_g_y = ay / ACCEL_SENS;
    accel_g_z = az / ACCEL_SENS;

    // magnitude รวม (กี่ g)
    accel_total_g = sqrt(
        accel_g_x * accel_g_x +
        accel_g_y * accel_g_y +
        accel_g_z * accel_g_z
    );
}

// ตรวจว่ารถอืด / หยุดไหม
bool isSlow(){
    // ปกติยืนเฉยๆ = ประมาณ 1.00 g
    // ถ้าเร่ง = ≈1.05–1.25 g
    // ถ้าอืด/ติด = กลับมาใกล้ 1.00 g
    if(accel_total_g > 0.98 && accel_total_g < 1.02){
        return true;    // รถอืด / หยุด
    }
    return false;
}


///////////////////////////////////////////เข็มทิศ + ความเร่ง /////////////////////////////////////////////////////////////



/////////////////////////////////////////
/////// AUTO RUN 4 SIDES (S/F/L/R) //////
/////////////////////////////////////////

// ====== สำหรับระบบลดกรอบ (Side-Shrink) ======
float shrinkStep = 0.4;       // ลดทีละ 0.4 m เฉพาะ BC/DA
float BC_current = 0;
float DA_current = 0;
int shrinkRound = 0;
bool shrinkMode = false;      // เริ่มลดกรอบหลังจากครบ 4 ด้าน

// ======= ตัวแปรหลักของระบบ =======
double distArr[4];
float headingArr[4];
unsigned long runDuration[4];



bool legsSetup = false;

enum PHASE {
    PHASE_ROTATE,
    PHASE_MOVE,
    PHASE_DONE
};

PHASE phase = PHASE_ROTATE;
int legIndex = 0;
unsigned long moveStart = 0;

const float timePerMeter = 0.8; // วินาที/เมตร
const float headingTol = 3;     // ±3° หันตรงพอแล้ว
unsigned long pausedTotal = 0;
unsigned long pauseStart = 0;
bool isPaused = false;

// ===== ตัวแปรควบคุม Auto Pause =====
bool autoSuspended = false;
unsigned long autoSuspendStart = 0;
unsigned long autoSuspendedTotal = 0;

// เตรียมระยะในแต่ละรอบของ Side-Shrink
void prepareLegs() {

    // AB / CD คงเดิม
    distArr[0] = distAB;  // AB
    distArr[2] = distCD;  // CD

    // BC / DA ลดลงตามจำนวนรอบ
    distArr[1] = BC_current;
    distArr[3] = DA_current;

    // แปลงระยะเป็นเวลา
    for (int i = 0; i < 4; i++) {
        runDuration[i] = (unsigned long)(distArr[i] * timePerMeter * 1000);
    }

    Serial.printf("\n======= SHRINK ROUND %d =======\n", shrinkRound);
    Serial.printf("AB=%.2f   BC=%.2f   CD=%.2f   DA=%.2f\n",
        distArr[0], distArr[1], distArr[2], distArr[3]);
}


// ส่งคำสั่งมอเตอร์แบบตัวอักษร
void motorStop()   { Serial1.println("S"); Serial.println("S"); }
void motorForward(){ Serial1.println("F"); Serial.println("F"); }
void motorLeft()   { Serial1.println("L"); Serial.println("L"); }
void motorRight()  { Serial1.println("R"); Serial.println("R");}

// === ฟังก์ชันจัดมุม ===
float norm360(float a){
    while (a < 0) a += 360;
    while (a >= 360) a -= 360;
    return a;
}

float angleDiff(float target, float current){
    float d = norm360(target) - norm360(current);
    if (d > 180) d -= 360;
    if (d < -180) d += 360;
    return d;  // -180..+180
}





// =========================================================================
// ============================ SETUP ======================================
// =========================================================================

void setup() {
  Serial.begin(115200);
  Wire.begin(21,22);

  pinMode(relaypin, OUTPUT);
  digitalWrite(relaypin, HIGH);
  pinMode(RMONPIN, INPUT);
  pinMode(Zonarpin, INPUT);
  Serial1.begin(250000, SERIAL_8N1, 4, 18);

  SerialBT.begin(BT_BAUD, SERIAL_8N1, BT_RX, BT_TX);
  pinMode(CAL_BUTTON, INPUT_PULLUP);

  write8(ICM_ADDR,PWR_MGMT_1,0x01);
  write8(ICM_ADDR,ACCEL_CONFIG,0x08);
  write8(ICM_ADDR,GYRO_CONFIG,0x08);
  write8(MAG_ADDR,MAG_CNTL2,0x08);

  delay(20);

  calibrateGyro();

  // ลบส่วนตั้งค่า heading จาก mag ครั้งแรกทิ้ง
  gyro_heading = 0;
  heading_filtered = 0;

  last_t = millis();
  Serial.println("System Ready");

  Serial.println("Bluetooth Ready");
}

// =========================================================================
// ============================ LOOP ======================================
// =========================================================================

void loop() {

////////////////////////////QCT///////////////////////////////////
  if (millis() - lastHeartbeat > 1000) {
    lastHeartbeat = millis();
    send_heartbeat();
    send_statustext();
  }

  while (SerialBT.available()) {
    uint8_t c = SerialBT.read();

    static mavlink_message_t msg;
    static mavlink_status_t status;

    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {

      switch (msg.msgid) {

// ============= รับจำนวน Mission =====================
case MAVLINK_MSG_ID_MISSION_COUNT: {
    mavlink_mission_count_t m;
    mavlink_msg_mission_count_decode(&msg, &m);

    if (m.mission_type != MAV_MISSION_TYPE_MISSION) {
        Serial.println("⚠️ QGC ส่งไม่ใช่ MISSION");
        break;
    }

    mission_expected = m.count;
    mission_received = 0;
    mission_active = true;
    all_count = 0;

    Serial.printf("🗺️ เตรียมรับ Mission %d จุด\n", mission_expected);

    mavlink_message_t req;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_mission_request_int_pack(
        SYS_ID, COMP_ID, &req,
        m.target_system,
        m.target_component,
        0,
        MAV_MISSION_TYPE_MISSION
    );

    uint16_t len = mavlink_msg_to_send_buffer(buf, &req);
    SerialBT.write(buf, len);
    Serial.println("📤 ขอ WP #0");
}
break;


// ============= รับ Waypoint ทีละตัว =====================
case MAVLINK_MSG_ID_MISSION_ITEM_INT: {

    if (!mission_active) break;

    mavlink_mission_item_int_t item;
    mavlink_msg_mission_item_int_decode(&msg, &item);

    float lat = item.x / 1e7;
    float lon = item.y / 1e7;
    float alt = item.z;

    Serial.printf("📍 WP %d/%d → %.7f , %.7f , alt=%.1f\n",
                  item.seq, mission_expected, lat, lon, alt);

    mission_received++;

    // กรองค่าหลอก 0,0
    if (lat != 0 && lon != 0 && fabs(lat) > 0.000001) {
        if (all_count < 100) {
            all_wp[all_count].lat = lat;
            all_wp[all_count].lon = lon;
            all_wp[all_count].alt = alt;
            all_count++;
        }
    }

    // ขอ WP ต่อ
    if (item.seq + 1 < mission_expected) {
        mavlink_message_t req;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];

        mavlink_msg_mission_request_int_pack(
            SYS_ID, COMP_ID, &req,
            item.target_system,
            item.target_component,
            item.seq + 1,
            MAV_MISSION_TYPE_MISSION
        );

        uint16_t len = mavlink_msg_to_send_buffer(buf, &req);
        SerialBT.write(buf, len);

        Serial.printf("📤 ขอ WP #%d\n", item.seq + 1);
    }
    else {
        // Mission ครบแล้ว
        send_mission_ack(item.target_system, item.target_component,
                         MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION);

        Serial.println("🎉 รับ Mission ครบแล้ว!");
        mission_active = false;

        // หา 4 มุมจริง
        computeCorners();
    }

  }
  break;

      }
    }
  }
//////////////////////////////QCT////////////////////

/////////////////////////////เข็มทิศ//////////////////
// ===== ปุ่มคาลิเบรต =====
  if(digitalRead(CAL_BUTTON) == LOW){
    calibrateGyro();
    calibrateMag();

    gyro_heading = 0;
    heading_filtered = 0;

    Serial.println("Re-centered heading.");
    delay(500);
  }
  // ===== Accel =====
int16_t ax = read16(ICM_ADDR, ACCEL_XOUT_H);
int16_t ay = read16(ICM_ADDR, ACCEL_XOUT_H+2);
int16_t az = read16(ICM_ADDR, ACCEL_XOUT_H+4);

processAccel(ax, ay, az);

  // ===== Gyro =====
  int16_t gz_raw = read16(ICM_ADDR, ACCEL_XOUT_H+12);
  float gz = gz_raw/65.5 - gyro_bias;

  float dt = (millis() - last_t)/1000.0;
  last_t = millis();

  // ===== กลับทิศของ Gyro ให้ตรงมือถือ =====
  gyro_heading -= gz * dt;

  if(gyro_heading < 0) gyro_heading += 360;
  if(gyro_heading >= 360) gyro_heading -= 360;

  float mag_heading = heading_filtered;

  // ===== MAG read =====
  if(read8(MAG_ADDR, MAG_ST1) & 0x01){
    float mx = (read16(MAG_ADDR,MAG_HXL)     - mag_offset_x)*mag_scale_x;
    float my = (read16(MAG_ADDR,MAG_HXL+2)   - mag_offset_y)*mag_scale_y;
    float mz = (read16(MAG_ADDR,MAG_HXL+4)   - mag_offset_z)*mag_scale_z;

    read8(MAG_ADDR, MAG_ST2);

    float roll  = atan2(ay, az);
    float pitch = atan2(-ax, sqrt(ay*ay + az*az));

    float Xh = mx*cos(pitch) + mz*sin(pitch);
    float Yh = mx*sin(roll)*sin(pitch) + my*cos(roll) - mz*sin(roll)*cos(pitch);

    float h = atan2(-Xh, Yh) * 180.0/PI;
    if(h < 0) h += 360;

    // ===== กลับทิศให้ตรงมือถือ =====
    h = 360 - h;
    if(h >= 360) h -= 360;

    mag_heading = h;
  }

  // ===== Complementary Filter =====
  float alpha = 0.95;
  heading_filtered = gyro_heading;
  if(heading_filtered < 0) heading_filtered += 360;
  if(heading_filtered >= 360) heading_filtered -= 360;
  // ===== OUTPUT =====
  float Heding = gyro_heading;
  Serial.print("Heding = ");
  Serial.println(Heding);
 
/////////////////////////////เข็มทิศ+ความเร่ง////////////////////


/////////////////////////////คุมมอเตอร์/////////////////////////

    unsigned long remoteon = pulseIn(RMONPIN, HIGH, 25000);

    if (remoteon >= 1300) digitalWrite(relaypin, HIGH);

     // กันค่าผิด

    
// ====================================================================
// ============================== AUTO START ==========================
// ====================================================================
    if (remoteon < 1300 ){
        
            if (!legsSetup) {

                if (!cornerReady) {
                    Serial.println("❌ cornerReady = false -> ยังไม่พร้อมเริ่ม AUTO");
                    return;
                }
                digitalWrite(relaypin, LOW);

                Serial.println("=== AUTO START (OUTER SQUARE) ===");

                // ตั้งค่ารอบแรก
                BC_current = distBC;
                DA_current = distDA;
                shrinkRound = 0;

                // ตั้ง target heading
                headingArr[0] = dirAB_global;
                headingArr[1] = dirBC_global;
                headingArr[2] = dirCD_global;
                headingArr[3] = dirDA_global;

                // โหลดระยะรอบแรก
                prepareLegs();

                legIndex = 0;
                phase = PHASE_ROTATE;
                legsSetup = true;
            }



          // ====================================================================
          // ========================= AUTO CONTROL LOOP ========================
          // ====================================================================
          float hd = gyro_heading;   // อ่านเข็มทิศปัจจุบัน


          switch (phase) {

          // ====================== หมุนไปหาทิศที่ต้องไป ======================
          case PHASE_ROTATE: {

              float target = headingArr[legIndex];
              float e = angleDiff(target, hd);  // -180..+180

              if (fabs(e) < headingTol) {
                  motorStop();
                  moveStart = millis();
                  phase = PHASE_MOVE;
                  Serial.printf("ด้าน %d: หมุนเสร็จ → เริ่มเดินหน้า\n", legIndex+1);
              } else {
                  if (e > 0) motorRight();   // หมุนขวา
                  else        motorLeft();   // หมุนซ้าย
              }

          }
          break;


          // ======================= เดินหน้าตามเวลา ==========================
      case PHASE_MOVE: {

        unsigned long now = millis();

        // ======= เวลาที่ผ่านจริง = (now - moveStart - pausedTotal) =======
        unsigned long el = now - moveStart - pausedTotal;
        unsigned long total = runDuration[legIndex];

        Serial.printf("MOVE %d | time=%lu/%lu ms\n",
                      legIndex+1, el, total);

        // ====== ครบเวลาแล้ว -> ไปด้านถัดไป ======
        if (el >= total) {

            motorStop();
            Serial.printf("ด้าน %d เสร็จ\n", legIndex+1);

            legIndex++;

            isPaused = false;
            pausedTotal = 0;
       // ====================== ครบ 4 ด้านใน 1 รอบ =====================
if (legIndex >= 4) {

    motorStop();
    Serial.println("=== 1 LOOP FINISHED ===");

    shrinkRound++;

    // ============ ลดทั้ง 4 ด้านแบบกินขอบด้านใน ============
    distAB -= shrinkStep;   // ลด AB
    distBC -= shrinkStep;   // ลด BC
    distCD -= shrinkStep;   // ลด CD
    distDA -= shrinkStep;   // ลด DA

    // ป้องกันหดจนสั้นเกินไป
    if (distAB < 1.0 || distBC < 1.0 || distCD < 1.0 || distDA < 1.0) {
        Serial.println("=== SHRINK FINISHED: ANY SIDE TOO SMALL ===");
        phase = PHASE_DONE;
        return;
    }

    Serial.printf("=== START SHRINK ROUND %d ===\n", shrinkRound);
    Serial.printf("AB=%.2f   BC=%.2f   CD=%.2f   DA=%.2f\n",
                  distAB, distBC, distCD, distDA);

    // ============== update ระยะใหม่ ==============
    distArr[0] = distAB;
    distArr[1] = distBC;
    distArr[2] = distCD;
    distArr[3] = distDA;

    // ============== คำนวณเวลาของแต่ละด้านใหม่ ==============
    for (int i = 0; i < 4; i++) {
        runDuration[i] = (unsigned long)(distArr[i] * timePerMeter * 1000);
    }

    // ============== reset state ==============
    legIndex = 0;
    isPaused = false;
    pausedTotal = 0;
    phase = PHASE_ROTATE;

    return;
}
        else {
                phase = PHASE_ROTATE;
            }
            break;
        }

        // ====== คุมทิศระหว่างวิ่ง ======
        float target = headingArr[legIndex];
        float e = angleDiff(target, hd);

        //===============================
        //      ตรงดี → เดินหน้า F
        //===============================
        if (fabs(e) <= 10) {

            // ถ้าเคย pause → resume timer
            if (isPaused) {
                pausedTotal += (now - pauseStart);
                isPaused = false;
            }

            motorForward();
            Serial.println("F");
        }
        //===============================
        //      เบี่ยงขวา → R → pause
        //===============================
        else if (e > 10) {

            if (!isPaused) {          // เริ่ม pause ครั้งแรกเท่านั้น
                pauseStart = now;
                isPaused = true;
            }

            motorRight();
            Serial.println("R");
        }
        //===============================
        //      เบี่ยงซ้าย → L → pause
        //===============================
        else {

            if (!isPaused) {
                pauseStart = now;
                isPaused = true;
            }

            motorLeft();
            Serial.println("L");
        }

    }
    break;

      // ====================== ครบ 4 ด้านแล้ว =====================
      case PHASE_DONE:
          motorStop();
      break;

      }


  }


}
