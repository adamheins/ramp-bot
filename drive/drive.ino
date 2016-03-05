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
#define FL_PIN 4
#define FR_PIN 5
#define BL_PIN 2
#define BR_PIN 3

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

void setup() {
  Serial.begin(9600);
  duration_init(total_duration);
  duration_init(phase_duration);
  servo_init();
}

// loop() should be the master state machine, which delegates out to sub-loops that govern specific tasks/phases.
void loop() {
  unsigned long dur = duration_get_duration(total_duration);
  Serial.print(dur);
  if (dur < 3 * PERIOD) {
    servo_drive_side(LEFT, 2);
    servo_drive_side(RIGHT, -10);
  } else if (dur < 5 * PERIOD) {
     servo_point_turn(10);
  } else {
    servo_drive_side(LEFT, 2);
    servo_drive_side(RIGHT, -10);
  }

  everest_delay(PERIOD);
}

