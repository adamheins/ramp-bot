#pragma once

#define PHASE_ONE_DELAY 100
#define PHASE_TWO_DELAY 100
#define PHASE_THREE_DELAY 100

// The competition course is split into three distinct phases:
// 1. Find the ramp.
// 2. Climp the ramp.
// 3. Find the second base.
typedef enum Phase {
  PhaseOne = 0,
  PhaseTwo,
  PhaseThree,
} Phase;
