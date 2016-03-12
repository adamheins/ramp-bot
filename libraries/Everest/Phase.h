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
#define PHASE_ONE_DELAY 100

// Phase two constants
#define PHASE_TWO_DELAY 50xA

#define PHASE_TWO_RAMP_UP_SPEED_LEFT 10
#define PHASE_TWO_RAMP_UP_SPEED_RIGHT 20

#define PHASE_TWO_RAMP_DOWN_SPEED_LEFT 3
#define PHASE_TWO_RAMP_DOWN_SPEED_RIGHT 13

#define PHASE_TWO_SLOW(s) ((s) * 3 / 4)

// Phase three constants
#define PHASE_THREE_DELAY 100
