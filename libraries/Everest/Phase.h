#pragma once

// The competition course is split into three distinct phases:
// 1. Find the ramp.
// 2. Climp the ramp.
// 3. Find the second base.
typedef enum Phase {
  PHASE_ONE = 0,
  PHASE_TWO,
  PHASE_THREE,
} Phase;

/************************ Phase One constants *************************/

// Phase one constants
#define PHASE_ONE_DELAY 20

typedef enum SubphaseOne {
  PHASE_ONE_TO_BACK_WALL = 0,
  PHASE_ONE_WALL_FOLLOWING,
  PHASE_ONE_MOUNTING,
  PHASE_ONE_DONE,
} SubphaseOne;

/************************ Phase Two constants *************************/

#define PHASE_TWO_DELAY 20

typedef enum SubphaseTwo {
  PHASE_TWO_ASCEND_RAMP,
  PHASE_TWO_TOP_OF_RAMP,
  PHASE_TWO_DESCEND_RAMP,
  PHASE_TWO_DONE,
} SubphaseTwo;

/************************ Phase Three constants *************************/

// Phase three constants
#define PHASE_THREE_DELAY 20
