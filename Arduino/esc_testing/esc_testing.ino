// #include <Servo.h>
//
// constexpr int JOY_X = A4;
// constexpr int JOY_Y = A5;
//
// constexpr int JOY_MID_X = 514;
// constexpr int JOY_MID_Y = 519;
// constexpr int JOY_BUFFER = 100;
//
// constexpr int SIGNAL_PIN = 9;
// constexpr int INPUT_SIGNAL_PIN = A3;
// constexpr int MAX_SIGNAL = 180;
// constexpr int SIGNAL_DELTA = 5;
//
// int last_joy_x = 0;
// int last_joy_y = 0;
// int sig = 65;
// Servo myservo;
//
// void setup() {
//   myservo.attach(SIGNAL_PIN);
// 	Serial.begin(115200);
//   pinMode(JOY_X, INPUT);
//   pinMode(JOY_Y, INPUT);
//   pinMode(SIGNAL_PIN, OUTPUT);
// }
//
//
// // TODO: find out how to read PWM signal like servo.write sends
// void loop() {
//   int joy_x = analogRead(JOY_X);
//   int joy_y = analogRead(JOY_Y);
//   if (joy_x > JOY_MID_X + JOY_BUFFER && last_joy_x < JOY_MID_X + JOY_BUFFER) {
//     sig = (sig + SIGNAL_DELTA) % MAX_SIGNAL;
//   }
//   if (joy_y > JOY_MID_Y + JOY_BUFFER && last_joy_y < JOY_MID_Y + JOY_BUFFER) {
//     sig = (sig + 1) % MAX_SIGNAL;
//   }
//   if (joy_x < JOY_MID_X - JOY_BUFFER && last_joy_x > JOY_MID_X - JOY_BUFFER) {
//     sig -= SIGNAL_DELTA;
//     if (sig < 0) {
//       sig = MAX_SIGNAL;
//     }
//   }
//   if (joy_y < JOY_MID_Y - JOY_BUFFER && last_joy_y > JOY_MID_Y - JOY_BUFFER) {
//     sig -= 1;
//     if (sig < 0) {
//       sig = MAX_SIGNAL;
//     }
//   }
//
//   Serial.print("Joy_x: ");
//   Serial.print(joy_x);
//   Serial.print("\tJoy_y: ");
//   Serial.print(joy_y);
//   Serial.print("\tSignal: ");
//   Serial.print(sig);
//   Serial.print("\tInput signal: ");
//   Serial.println(analogRead(INPUT_SIGNAL_PIN));
//
//   myservo.write(sig);
//   // analogWrite(SIGNAL_PIN, sig);
//
//   last_joy_x = joy_x;
//   last_joy_y = joy_y;
//   // delay(10);
// }

#include <Servo.h>

constexpr int JOY_X = A4;
constexpr int JOY_Y = A5;

constexpr int JOY_MID_X = 514;
constexpr int JOY_MID_Y = 519;
constexpr int JOY_BUFFER = 100;

constexpr int MAX_SIGNAL = 180;
constexpr int SIGNAL_DELTA = 5;

constexpr byte pwmInPin = 5;
constexpr byte pwmOutPin = 9;
Servo pwmOut;

int last_joy_x = 0;
int last_joy_y = 0;
int sig = 65;

void setup() {
  Serial.begin(115200);
  pwmOut.attach(pwmOutPin);
  pinMode(pwmInPin, INPUT);
}

typedef struct PWMTimes {
  byte hightime;
  byte lowtime;
} PWMTimes;
PWMTimes GetPWM(byte pin);

constexpr int start_deg = 55;

void loop() {
  static int deg = start_deg;
  pwmOut.write(deg);
  PWMTimes PWM = GetPWM(pwmInPin);
  Serial.print("PWM High: ");
  Serial.print(PWM.hightime);
  Serial.print(" PWM Low: ");
  Serial.print(PWM.lowtime);
  Serial.print("\tDEG:");
  Serial.println(deg);
  delay(200);
  deg++;
  if (deg>160) {
    deg = start_deg;
  }
}

PWMTimes GetPWM(byte pin) {
  unsigned long highTime = pulseIn(pin, HIGH, 50000UL);  // 50 millisecond timeout
  unsigned long lowTime = pulseIn(pin, LOW, 50000UL);  // 50 millisecond timeout

  // pulseIn() returns zero on timeout
  if (highTime == 0 || lowTime == 0)
    return {.hightime=100, .lowtime=0};
    //return digitalRead(pin) ? 100 : 0;  // HIGH == 100%,  LOW = 0%

  return {.hightime=highTime, .lowtime=lowTime};
  // return (100 * highTime) / (highTime + lowTime);  // highTime as percentage of total cycle time
}
