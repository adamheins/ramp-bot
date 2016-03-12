#include <Servo.h>


// Driving servos
#define FL_SERVO_PIN 9
#define FR_SERVO_PIN 10
#define BL_SERVO_PIN 11
#define BR_SERVO_PIN 12
#define PAN_SERVO_PIN 8

Servo Servo_pan;
Servo Servo_FL;
Servo Servo_FR;
Servo Servo_BL;
Servo Servo_BR;

#define FL_CORR -2
#define FR_CORR 0
#define BL_CORR -2
#define BR_CORR 0


void drive(int left_pow, int right_pow)
{
  //left side servo power
  Servo_FL.write(90 + left_pow + FL_CORR);
  Servo_BL.write(90 + left_pow + BL_CORR);
  Servo_FR.write(90 - right_pow + FR_CORR);
  Servo_BR.write(90 - right_pow + BR_CORR);
}


void setup() {
  // put your setup code here, to run once:
  Servo_FL.attach(FL_SERVO_PIN);
  Servo_FR.attach(FR_SERVO_PIN);
  Servo_BL.attach(BL_SERVO_PIN);
  Servo_BR.attach(BR_SERVO_PIN);
 Servo_pan.attach(PAN_SERVO_PIN);
}

void loop() {
  // put your main code here, to run repeatedly:
drive(20,20);
}
