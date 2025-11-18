#include <Servo.h>

// ===== ขารับ PWM เดิม =====
const int pwmPosPin     = 9;  // รับ PWM 1–2ms สำหรับมุมเซอร์โว
const int pwmRelayPin1  = 8;  // รับ PWM 1–2ms สำหรับทริกรีเลย์
const int pwmRelayPin2  = 7;  // รับ PWM 1–2ms สำหรับทริกรีเลย์

// ===== ขารับ PWM เพิ่มเติม (ใช้กำหนดทิศทางมอเตอร์) =====
const int pwmExtraPin1  = 2;  // ใช้คู่กับ pwmExtraPin2
const int pwmExtraPin2  = 3;

// ===== เซอร์โว + รีเลย์ =====
const int servoPin      = 5;  // ขาคุมเซอร์โว
const int relayPin      = 4;  // ขาคุมรีเลย์ (Active LOW)

// ===== ขาโมดูลขับมอเตอร์ (PWM, IN1, IN2) ต่อ 1 มอเตอร์ =====
// Motor 1
const int M1_PWM = 6;   // PWM
const int M1_IN1 = 22;
const int M1_IN2 = 23;

// Motor 2
const int M2_PWM = 10;  // PWM
const int M2_IN1 = 24;
const int M2_IN2 = 25;

// Motor 3
const int M3_PWM = 11;  // PWM
const int M3_IN1 = 26;
const int M3_IN2 = 27;

// Motor 4
const int M4_PWM = 12;  // PWM
const int M4_IN1 = 28;
const int M4_IN2 = 29;

Servo myServo;

const int motorSpeed = 180;   // ความเร็วมอเตอร์ 0–255 (ปรับได้)

// ===== ฟังก์ชันช่วยสั่งมอเตอร์ 1 ตัว =====
// forward = true -> เดินหน้า, false -> ถอยหลัง
void driveMotor(int pwmPin, int in1Pin, int in2Pin, bool forward, int speed) {
  int s = constrain(speed, 0, 255);

  if (s == 0) {
    analogWrite(pwmPin, 0);
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    return;
  }

  if (forward) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
  } else {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
  }

  analogWrite(pwmPin, s);
}

// หยุดมอเตอร์ทุกตัว
void stopAllMotors() {
  driveMotor(M1_PWM, M1_IN1, M1_IN2, true, 0);
  driveMotor(M2_PWM, M2_IN1, M2_IN2, true, 0);
  driveMotor(M3_PWM, M3_IN1, M3_IN2, true, 0);
  driveMotor(M4_PWM, M4_IN1, M4_IN2, true, 0);
}

// classify pulse เป็น 1ms / 1.5ms / 2ms
// return 1, 15, 2 หรือ 0 ถ้าไม่ตรงช่วง
int classifyPulse(unsigned long p) {
  if (p >= 800 && p <= 1200)   return 1;   // ~1.0ms
  if (p >= 1300 && p <= 1700)  return 15;  // ~1.5ms
  if (p >= 1800 && p <= 2200)  return 2;   // ~2.0ms
  return 0;
}

void setup() {
  Serial.begin(9600);

  // ขารับ PWM
  pinMode(pwmPosPin,    INPUT);
  pinMode(pwmRelayPin1, INPUT);
  pinMode(pwmRelayPin2, INPUT);
  pinMode(pwmExtraPin1, INPUT);
  pinMode(pwmExtraPin2, INPUT);

  // รีเลย์
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, HIGH);   // ปิดรีเลย์ (Active LOW)

  // มอเตอร์ทุกขา
  pinMode(M1_PWM, OUTPUT);
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);

  pinMode(M2_PWM, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);

  pinMode(M3_PWM, OUTPUT);
  pinMode(M3_IN1, OUTPUT);
  pinMode(M3_IN2, OUTPUT);

  pinMode(M4_PWM, OUTPUT);
  pinMode(M4_IN1, OUTPUT);
  pinMode(M4_IN2, OUTPUT);

  stopAllMotors();

  // เซอร์โว
  myServo.attach(servoPin);
  myServo.write(0);
}

void loop() {
  // ===== 1) อ่าน PWM เซอร์โว =====
  unsigned long pulsePos = pulseIn(pwmPosPin, HIGH, 25000);
  Serial.print("pwnpos ");
  Serial.println(pulsePos);

  if (pulsePos >= 1000 && pulsePos <= 2000) {
    int angle = map(pulsePos, 1000, 2000, 0, 30);
    angle = constrain(angle, 0, 30);
    myServo.write(angle);
    Serial.print("angle ");
    Serial.println(angle);
  }

  // ===== 2) อ่าน PWM สำหรับรีเลย์เดิม (ถ้ายังใช้อยู่) =====
  unsigned long pulseRelay1 = pulseIn(pwmRelayPin1, HIGH, 25000);
  unsigned long pulseRelay2 = pulseIn(pwmRelayPin2, HIGH, 25000);
  Serial.print("pwnrelay1 ");
  Serial.println(pulseRelay1);
  Serial.print("pwnrelay2 ");
  Serial.println(pulseRelay2);

  if (pulseRelay2 > 1900) {
    digitalWrite(relayPin, LOW);   // เปิดรีเลย์ (Active LOW)
  } else {
    digitalWrite(relayPin, HIGH);  // ปิดรีเลย์
  }

  // ===== 3) อ่าน PWM จาก pwmExtraPin1 / pwmExtraPin2 สำหรับโหมด "ความถี่" =====
  unsigned long pulseExtra1 = pulseIn(pwmExtraPin1, HIGH, 25000);
  unsigned long pulseExtra2 = pulseIn(pwmExtraPin2, HIGH, 25000);

  Serial.print("extra1 ");
  Serial.println(pulseExtra1);
  Serial.print("extra2 ");
  Serial.println(pulseExtra2);

  int ex1 = classifyPulse(pulseExtra1);
  int ex2 = classifyPulse(pulseExtra2);

  bool usedPwmDirection = false;   // flag บอกว่าใช้โหมด PWM หรือยัง

  // ===== 4) เงื่อนไขทิศทางจาก "ความถี่" (เหมือนของเดิม) =====
  if (ex1 == 2 && ex2 == 15 ) {
    // Extra1 2ms, Extra2 1.5ms → ทั้ง 4 เดินหน้า
    driveMotor(M1_PWM, M1_IN1, M1_IN2, true,  motorSpeed);
    driveMotor(M2_PWM, M2_IN1, M2_IN2, true,  motorSpeed);
    driveMotor(M3_PWM, M3_IN1, M3_IN2, true,  motorSpeed);
    driveMotor(M4_PWM, M4_IN1, M4_IN2, true,  motorSpeed);
    usedPwmDirection = true;

  } else if (ex1 == 1 && ex2 == 15) {
    // Extra1 1ms, Extra2 1.5ms → ทั้ง 4 ถอยหลัง
    driveMotor(M1_PWM, M1_IN1, M1_IN2, false, motorSpeed);
    driveMotor(M2_PWM, M2_IN1, M2_IN2, false, motorSpeed);
    driveMotor(M3_PWM, M3_IN1, M3_IN2, false, motorSpeed);
    driveMotor(M4_PWM, M4_IN1, M4_IN2, false, motorSpeed);
    usedPwmDirection = true;

  } else if (ex1 == 15 && ex2 == 2) {
    // Extra1 1.5ms, Extra2 2ms → M1 M2 เดินหน้า, M3 M4 ถอย (เลี้ยวขวา)
    driveMotor(M1_PWM, M1_IN1, M1_IN2, true,  motorSpeed);
    driveMotor(M2_PWM, M2_IN1, M2_IN2, true,  motorSpeed);
    driveMotor(M3_PWM, M3_IN1, M3_IN2, false, motorSpeed);
    driveMotor(M4_PWM, M4_IN1, M4_IN2, false, motorSpeed);
    usedPwmDirection = true;

  } else if (ex1 == 15 && ex2 == 1) {
    // Extra1 1.5ms, Extra2 1ms → M3 M4 เดินหน้า, M1 M2 ถอย (เลี้ยวซ้าย)
    driveMotor(M1_PWM, M1_IN1, M1_IN2, false, motorSpeed);
    driveMotor(M2_PWM, M2_IN1, M2_IN2, false, motorSpeed);
    driveMotor(M3_PWM, M3_IN1, M3_IN2, true,  motorSpeed);
    driveMotor(M4_PWM, M4_IN1, M4_IN2, true,  motorSpeed);
    usedPwmDirection = true;
  } else if (ex1 == 15 && ex2 == 15) {
    stopAllMotors();
    usedPwmDirection = true;
  }

  // ===== 5) ถ้า "ยังไม่ได้ใช้โหมด PWM" → ใช้โหมด Digital จาก pwmRelayPin1 / pwmRelayPin2 =====
  if (!usedPwmDirection) {
    int d1 = digitalRead(pwmRelayPin1);
    int d2 = digitalRead(pwmRelayPin2);

    Serial.print("digital dir d1=");
    Serial.print(d1);
    Serial.print(" d2=");
    Serial.println(d2);

    if (d1 == LOW && d2 == LOW) {
      // 0,0 → หยุด
      stopAllMotors();

    } else if (d1 == HIGH && d2 == HIGH) {
      // 1,1 → เดินหน้า
      driveMotor(M1_PWM, M1_IN1, M1_IN2, true,  motorSpeed);
      driveMotor(M2_PWM, M2_IN1, M2_IN2, true,  motorSpeed);
      driveMotor(M3_PWM, M3_IN1, M3_IN2, true,  motorSpeed);
      driveMotor(M4_PWM, M4_IN1, M4_IN2, true,  motorSpeed);

    } else if (d1 == HIGH && d2 == LOW) {
      // 1,0 → เลี้ยวซ้าย (M3 M4 เดินหน้า, M1 M2 ถอยหลัง)
      driveMotor(M1_PWM, M1_IN1, M1_IN2, false, motorSpeed);
      driveMotor(M2_PWM, M2_IN1, M2_IN2, false, motorSpeed);
      driveMotor(M3_PWM, M3_IN1, M3_IN2, true,  motorSpeed);
      driveMotor(M4_PWM, M4_IN1, M4_IN2, true,  motorSpeed);

    } else if (d1 == LOW && d2 == HIGH) {
      // 0,1 → เลี้ยวขวา (M1 M2 เดินหน้า, M3 M4 ถอยหลัง)
      driveMotor(M1_PWM, M1_IN1, M1_IN2, true,  motorSpeed);
      driveMotor(M2_PWM, M2_IN1, M2_IN2, true,  motorSpeed);
      driveMotor(M3_PWM, M3_IN1, M3_IN2, false, motorSpeed);
      driveMotor(M4_PWM, M4_IN1, M4_IN2, false, motorSpeed);

    } else {
      // กันเหนียว ถ้ามีอะไรแปลก → หยุด
      stopAllMotors();
    }
  }

  Serial.println("----");
  delay(100);
}
