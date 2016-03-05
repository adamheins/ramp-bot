#pragma once

#include <Pins.h>
#include <Servo.h>

static const int SERVO_STOP = 90;

class DriveServo {
  public:
    DriveServo(int pin);
    void start();
    void stop();
    void drive(int vel);

  private:
    int pin;
    Servo *servo;
}

DriveServo::DriveServo(int pin) {
  this->pin = pin;
}

void DriveServo::start() {
  servo->attach(pin);
}

void DriveServo::stop() {
  servo->deattach(pin);
}

void DriveServo::drive(int vel) {
  servo->write(SERVO_STOP + vel);
}

// Four wheel drive system.
class FWDrive {
  public:
    FWDrive();
    ~FWDrive();
    void start();
    void stop();
    void left(int vel);
    void right(int vel);
    void drive(int vel);
    void turn(int vel);

  private:
    DriveServo servos[2][2];
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
  servos[LEFT][FRONT].start();
  servos[LEFT][BACK].start();
  servos[RIGHT][FRONT].start();
  servos[RIGHT][BACK].start();
}

void FWDrive::stop() {
  servos[LEFT][FRONT].stop();
  servos[LEFT][BACK].stop();
  servos[RIGHT][FRONT].stop();
  servos[RIGHT][BACK].stop();
}
