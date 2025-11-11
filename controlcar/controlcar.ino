#include <AFMotor.h>
AF_DCMotor motorL(1);
AF_DCMotor motorR(2);
int SpeedTurn = 80;
int Speedy = 255;
void setup() {
  Serial.begin(9600);
}

void TurnL() {
  motorL.run(BACKWARD);
  motorL.setSpeed(SpeedTurn);
  motorR.run(FORWARD);
  motorR.setSpeed(Speedy);
}

void TurnR() {
  motorR.run(BACKWARD);
  motorR.setSpeed(SpeedTurn);
  motorL.run(FORWARD);
  motorL.setSpeed(Speedy);
}

void TurnB() {
  motorR.run(BACKWARD);
  motorR.setSpeed(Speedy);
  motorL.run(BACKWARD);
  motorL.setSpeed(Speedy);
}

void TurnW() {
  motorR.run(FORWARD);
  motorR.setSpeed(Speedy);
  motorL.run(FORWARD);
  motorL.setSpeed(Speedy);
}

void stops() {
  motorR.run(RELEASE);
  motorR.setSpeed(0);
  motorL.run(RELEASE);
  motorL.setSpeed(0);
}

void loop() {
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
     if (data = "TW"){
      TurnW();
     }
     if(data = "TL"){
      TurnL();
     }
     if(data = "TR"){
      TurnR();
     }
     if(data = "TB"){
      TurnB();
     }
     if (data = "ST"){
      stops();
     }
  }
}