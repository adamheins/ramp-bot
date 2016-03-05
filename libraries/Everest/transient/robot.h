#include <Servo.h>

#define FL_PIN 8
#define FR_PIN 9
#define BL_PIN 10
#define BR_PIN 11

typedef enum Side {
  LEFT = 0,
  RIGHT,
} Side;

typedef enum Bumper {
  FRONT = 0,
  BACK,
} Bumper;

typedef struct PanServo {
  Servo servo;
  int angle;
} PanServo;

class DriveServo {
  public:
    DriveServo(int pin);
    void on();
    void off();
    void forward(int vel);
    void reverse(int vel);

  private:
    int pin;
    int vel;
    Servo servo;
}

DriveServo::DriveServo(int pin) {
  this->pin = pin;
}

void DriveServo::on() {
  this->servo.attach(this->pin);
}

void DriveServo::off() {
  this->servo.deattach();
}

// This is our robot.
typedef struct Robot {
  DriveServo drive_servos[2][2];
  PanServo pan_servo;
} Robot;

void robot_init(Robot *robot) {
  // Initialize components of robot
}

void robot_motors_on(Robot *robot) {
  drive_servos[LEFT][FRONT].attach(FL_PIN);
  drive_servos[LEFT][BACK].attach(BL_PIN);
  drive_servos[RIGHT][FRONT].attach(FR_PIN);
  drive_servos[RIGHT][BACK].attach(BR_PIN);
}

void robot_motors_off(Robot *robot) {
  drive_servos[LEFT][FRONT].deattach();
  drive_servos[LEFT][BACK].deattach();
  drive_servos[RIGHT][FRONT].deattach();
  drive_servos[RIGHT][BACK].deattach();
}
