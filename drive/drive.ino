#include <Ultra.h>

#include <Servo.h>

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

// Delay on each iteration of the loop.
#define PERIOD 1000

// Value to write to servo to stop it.
#define SERVO_STOP 90

// Servo pins.
#define FL_PIN 8
#define FR_PIN 9
#define BL_PIN 10
#define BR_PIN 11

#define PAN_PIN 8

Servo pan_servo;

Servo SERVOS[2][2];

// This is an idea, maintain some state with the servos?
// Would probably make more sense to just rely on sensor readings, so we are more or less stateless (aside from buffered values)
typedef struct EverestServo {
  Servo servo;
  int dist;
} EverestServo;

typedef enum Side {
  LEFT = 0,
  RIGHT,
} Side;

typedef enum Bumper {
  FRONT = 0,
  BACK,
} Bumper;

// Initialize the servos.
void servo_init(void) {
  SERVOS[LEFT][FRONT].attach(FL_PIN);
  SERVOS[LEFT][BACK].attach(BL_PIN);
  SERVOS[RIGHT][FRONT].attach(FR_PIN);
  SERVOS[RIGHT][BACK].attach(BR_PIN);
}

// Drive one side of the motor.
//
// @param side The side to turn {LEFt|RIGHT}
// @param vel  Velocity at which to drive (0 is stopped).
void servo_drive_side(int side, int vel) {
  SERVOS[side][FRONT].write(SERVO_STOP + vel);
  SERVOS[side][BACK].write(SERVO_STOP + vel);
}

// Drive the entire robot in one direction.
//
// @param vel The velocity at which to drive (positive means forward, negative
// is backward).
void servo_drive(int vel) {
  servo_drive_side(LEFT, vel);
  servo_drive_side(RIGHT, -vel);
}

// Turn the robot on the spot.
//
// Theoretically, if we write out the same value to both sides, the robot
// should turn on the spot. Since each side turns at somewhat different speeds,
// this may not be the case in practice.
//
// @param vel Speed of turning. Positive values turn clockwise, negative values
// turn counter-clockwise.
void servo_point_turn(int vel) {
  servo_drive_side(LEFT, vel);
  servo_drive_side(RIGHT, vel);
}

  //////////////
 // Duration //
//////////////

typedef struct Duration {
  unsigned long start;
  unsigned long current;
} Duration;

void duration_init(struct Duration *dur) {
  dur->start = millis();
  dur->current = dur->start;
}

unsigned long duration_get_duration(struct Duration *dur) {
  return dur->current - dur->start;
}

void duration_tick(struct Duration *dur) {
  dur->current = millis();
}

// We track two durations at any one time.
// The first is a total duration, running from the start of the course.
// The second is a phase duration, which is the duration of the current phase of the course (eg. find ramp, climb ramp, find base)
struct Duration *total_duration;
struct Duration *phase_duration;

int current_phase = 0;

  /////////////
 // Utility //
/////////////

void everest_delay(unsigned long ms) {
  delay(ms);
  // Do we care that these will be *slightly* out-of-sync? Perhaps.
  duration_tick(total_duration);
  duration_tick(phase_duration);
}

  //////////
 // Main //
//////////

int ultraPin = 7;

int pan_angle = 0;
int pan_inc = 1;

// Ultrasonic sensor.
Ultra *ultra;

int sec = 0;

void setup() {
  Serial.begin(9600);  
  ultra = new Ultra(ultraPin);
  pan_servo.attach(PAN_PIN);
  pan_servo.write(pan_angle);
}

// Terminate the program by sitting in an endless loop.
void stop() {
  while(1);
}

// loop() should be the master state machine, which delegates out to sub-loops that govern specific tasks/phases.
void loop() {
  // dank panning
  pan_angle += pan_inc;
  pan_servo.write(pan_angle);
  if (pan_angle >= 180) {
    pan_inc = -1;
  } else if (pan_angle <= 0) {
    pan_inc = 1;
  }
  
  // ultrasonic readings
  int duration = ultra->ping();
  int distance = ultra->distance();
  Edge *edge = ultra->edge();

  Serial.print(distance);
  Serial.print(" ");
  if (edge->side == EdgeRight) {
    Serial.println("Right");
  } else if (edge->side == EdgeLeft) {
    Serial.println("Left");
  } else {
    Serial.println("None");
  }
  Serial.println(edge->side);
  free(edge);

  delay(50);
}
