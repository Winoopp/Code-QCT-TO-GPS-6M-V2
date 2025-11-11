const int pwmPin18 = 18;
const int pwmPin19 = 19;
int duty_1ms = (int)(65535 * (1.0 / 20.0));
int duty_1_5ms = (int)(65535 * (1.5 / 20.0));
int duty_2ms = (int)(65535 * (2.0 / 20.0));

void setup() {
  Serial.begin(115200);
  ledcAttach(pwmPin18, 50, 16);
  ledcAttach(pwmPin19, 50, 16);
}

void loop() {
  ledcWrite(pwmPin18, duty_1ms);
  ledcWrite(pwmPin19, duty_1ms);
}
