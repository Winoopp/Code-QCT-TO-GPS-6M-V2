#include <Servo.h>

const int pwmPosPin   = 9;   // รับ PWM 1–2ms สำหรับมุมเซอร์โว
const int pwmRelayPin1 = 8;   // รับ PWM 1–2ms สำหรับทริกรีเลย์
const int pwmRelayPin2 = 7;   // รับ PWM 1–2ms สำหรับทริกรีเลย์

const int servoPin    = 5;   // ขาคุมเซอร์โว
const int relayPin    = 4;   // ขาคุมรีเลย์

Servo myServo;

void setup() {
  Serial.begin(9600);
  pinMode(pwmPosPin, INPUT);
  pinMode(pwmRelayPin1, INPUT);
  pinMode(pwmRelayPin2, INPUT);
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, HIGH);

  myServo.attach(servoPin);
  myServo.write(0);            // เริ่มที่ 0 องศา
}

void loop() {
  // อ่าน PWM จาก D8 (1–2ms) แล้วแปลงเป็นมุม 0–30°
  unsigned long pulsePos = pulseIn(pwmPosPin, HIGH, 25000);  // timeout 25ms
  Serial.print("pwnpos ");
  Serial.println(pulsePos);
  if (pulsePos >= 1000 && pulsePos <= 2000) {
    // map 1000–2000 µs → 0–30°
    int angle = map(pulsePos, 1000, 2000, 0, 30);
    angle = constrain(angle, 0, 30);
    myServo.write(angle);
    Serial.print("angle ");
    Serial.println(angle);
  }

  // อ่าน PWM จาก D9 (1–2ms) ถ้ามากกว่า 1700 µs ให้ทริกรีเลย์
  unsigned long pulseRelay1 = pulseIn(pwmRelayPin1, HIGH, 25000);
  unsigned long pulseRelay2 = pulseIn(pwmRelayPin2, HIGH, 25000);
  Serial.print("pwnrelay1 ");
  Serial.println(pulseRelay1);
  Serial.print("pwnrelay2 ");
  Serial.println(pulseRelay2);

  if (pulseRelay1 > 1900 && pulseRelay2 > 1900) {
    digitalWrite(relayPin, LOW);
  } else {
    digitalWrite(relayPin, HIGH);    // ปิดรีเลย์
  }
  delay(200);
}
