/*
 basic servo control via USB-serial
 PWM via PCA9685 I2C
 * GND
 * 5V
 * pin 21 = SDA
 * pin 22 = SDL
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); //default address 0x40

//#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
//#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)

#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates


int minp = 70;
int maxp = 480;
int i;
int back;
int pulselen;
int serialv = 0;

int percent;

uint8_t servonum = 0;
String command;

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  delay(10);
}

void turn(uint8_t servo, int per) { // expect 0-100
  int pwmv =  ( 100 - per ) * ( maxp - minp ) / 100 + minp;
  Serial.print("percent:");
  Serial.print(per);
  Serial.print(" pwm:");
  Serial.println(pwmv);
  pwm.setPWM(servo, 0, pwmv);
}

void readserial() {
  if (Serial.available()) {
    command = Serial.readStringUntil('\n');
    command.trim();
    serialv = command.toInt();
    turn(0, serialv);
    turn(1, serialv);
  }
}

void loop() {
  readserial();
  
  delay(10);

}
