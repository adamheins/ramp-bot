#pragma once

#include <Pins.h>
#include <Servo.h>

static const int SERVO_STOP = 90;

typedef enum Side {
  LEFT = 0,
  RIGHT,
} Side;

typedef enum Bumper {
  FRONT = 0,
  BACK,
} Bumper;

// NOTE: servo values range from 0 - 180 where:
//  0 is full reverse
//  90 is full stop
//  180 is full forward

// Left side moves somewhat faster than the right side, so we need to manually
// slow it down.

// Layout of the robot's servos:
//
// FL--FR
// |    |
// |    |
// |    |
// BL--BR
//

class DriveServo {
  public:
    DriveServo(int pin);
    ~DriveServo();
    void start();
    void stop();
    void drive(int vel);
    Servo *servo;

  private:
    int pin;
};

// Four wheel drive system.
class FWDrive {
  public:
    FWDrive();
    ~FWDrive();
    void start();
    void stop();
    FWDrive *left(int vel);
    FWDrive *right(int vel);
    FWDrive *drive(int vel);
    FWDrive *pivot(int vel);
    DriveServo *servo(Side side, Bumper bumper);

  private:
    DriveServo *servos[2][2];
};
