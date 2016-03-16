#pragma once

// The competition course is split into three distinct phases:
// 1. Find the ramp.
// 2. Climp the ramp.
// 3. Find the second base.
typedef enum Phase {
  PhaseOne = 0,
  PhaseTwo,
  PhaseThree,
} Phase;

// Phase one constants
#define PHASE_ONE_DELAY 20

typedef enum SubphaseOne {
  PHASE_ONE_TO_BACK_WALL = 0,
  PHASE_ONE_WALL_FOLLOWING,
  PHASE_ONE_TURNING1, // go about 45 deg then start looking for ramp
  PHASE_ONE_TURNING2, // start looking for ramp
  PHASE_ONE_MOUNTING,
  PHASE_ONE_DONE,
} SubphaseOne;

// Phase two constants
#define PHASE_TWO_DELAY 100

#define PHASE_TWO_RAMP_UP_SPEED_LEFT 10
#define PHASE_TWO_RAMP_UP_SPEED_RIGHT 20

#define PHASE_TWO_RAMP_DOWN_SPEED_LEFT 3
#define PHASE_TWO_RAMP_DOWN_SPEED_RIGHT 13

// Used to differentially slow one side of the robot, to return it toward the
// centre of the ramp.
#define PHASE_TWO_SLOW(s) ((s) * 3 / 4)

// Phase three constants
#define PHASE_THREE_DELAY 100
