#define MAX_WAYPOINTS 4000
bool gpsmode = false;
double targetLat = waypoints[currentWaypoint].lat;
double targetLon = waypoints[currentWaypoint].lon;

int currentWaypoint = 0; // เริ่มจาก waypoint แรก

int switchrelay = 14;

struct Waypoint {
  int id;
  float lat;
  float lon;
};

Waypoint waypoints[MAX_WAYPOINTS];
int waypointCount = 0;

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

void setup(){
  pinMode(switchrelay, OUTPUT);
  digitalWrite(switchrelay, TRUE);
}

void loop(){
  if(gpsmode){
    digitalWrite(switchrelay, FALSE);
    if (gps.location.isUpdated()) {

      double currLat = gps.location.lat();
      double currLon = gps.location.lng();

      LoRa.beginPacket();
      LoRa.println(String(currLat)+","+String(currLon));
      LoRa.endPacket();

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
        Serial2.println("ST");
        Serial.println("ST");
      }
      delay(50);
    }
  }else{
    digitalWrite(switchrelay, TRUE);
  }
}
