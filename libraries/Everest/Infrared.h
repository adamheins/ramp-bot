#pragma once

#include <Arduino.h>
#include <Buffer.h>

#define BOTTOM_IR_RAMP_THRESHOLD 5
#define IR_BUFFER_LENGTH 5

// Infrared distance sensor.
class Infrared {
  public:
    Infrared(int pin);
    ~Infrared();
    long ping();
    long distance();

  private:
    Buffer *buffer;
    int pin;
};

// IRs are non-linear, they need to be scaled to get linear distance values.

#define BOTTOM_IR_LEFT_SCALE(d) (7468.9 * pow((d), -1.374))
#define BOTTOM_IR_RIGHT_SCALE(d) (2797.1 * pow((d), -1.212))

#define FRONT_IR_LEFT_SCALE(d) (7846.4 * pow((d), -1.083))
#define FRONT_IR_RIGHT_SCALE(d) (30928.0 * pow((d), -1.316))
