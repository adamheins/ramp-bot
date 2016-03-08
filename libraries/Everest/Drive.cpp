#include <Drive.h>

#include <Pins.h>
#include <Servo.h>

DriveServo::DriveServo(int pin) {
  servo = new Servo();
  this->pin = pin;
}

DriveServo::~DriveServo() {
  delete servo;
}

void DriveServo::start() {
  servo->attach(pin);
}

void DriveServo::stop() {
  servo->detach();
}

void DriveServo::drive(int vel) {
  servo->write(SERVO_STOP + vel);
}

FWDrive::FWDrive() {
  servos[LEFT][FRONT] = new DriveServo(FL_SERVO_PIN);
  servos[LEFT][BACK] = new DriveServo(BL_SERVO_PIN);
  servos[RIGHT][FRONT] = new DriveServo(FR_SERVO_PIN);
  servos[RIGHT][BACK] = new DriveServo(BR_SERVO_PIN);
}

FWDrive::~FWDrive() {
  delete servos[LEFT][FRONT];
  delete servos[LEFT][BACK];
  delete servos[RIGHT][FRONT];
  delete servos[RIGHT][BACK];
}

void FWDrive::start() {
  servos[LEFT][FRONT]->start();
  servos[LEFT][BACK]->start();
  servos[RIGHT][FRONT]->start();
  servos[RIGHT][BACK]->start();
}

void FWDrive::stop() {
  servos[LEFT][FRONT]->stop();
  servos[LEFT][BACK]->stop();
  servos[RIGHT][FRONT]->stop();
  servos[RIGHT][BACK]->stop();
}

FWDrive *FWDrive::left(int vel) {
  servos[LEFT][FRONT]->drive(vel);
  servos[LEFT][BACK]->drive(vel);
  return this;
}

FWDrive *FWDrive::right(int vel) {
  servos[RIGHT][FRONT]->drive(-vel);
  servos[RIGHT][BACK]->drive(-vel);
  return this;
}

FWDrive *FWDrive::drive(int vel) {
  return left(vel)->right(vel);
}

FWDrive *FWDrive::pivot(int vel) {
  return left(vel)->right(-vel);
}

DriveServo *FWDrive::servo(Side side, Bumper bumper) {
  return servos[side][bumper];
}
