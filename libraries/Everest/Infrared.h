#pragma once

#include <Arduino.h>
#include <Buffer.h>

// Note that it is important to keep this threshold high enough to deal with
// the bumps at the top of the ramp.
#define BOTTOM_IR_RAMP_THRESHOLD 8

#define IR_BUFFER_LENGTH 5

// Infrared distance sensor.
class Infrared {
  public:
    Infrared(int pin);
    ~Infrared();
    long ping();
    long distance();
    void flush();

  private:
    Buffer *buffer;
    int pin;
};

// IRs are non-linear, they need to be scaled to get linear distance values.

#define BOTTOM_IR_LEFT_SCALE(d) (7468.9 * pow((d), -1.374))
#define BOTTOM_IR_RIGHT_SCALE(d) (2797.1 * pow((d), -1.212))

#define FRONT_IR_LEFT_SCALE(d) (7846.4 * pow((d), -1.083))
//#define FRONT_IR_RIGHT_SCALE(d) (30928.0 * pow((d), -1.316))
#define FRONT_IR_RIGHT_SCALE(d) (7846.4 * pow((d), -1.083) + 1)
