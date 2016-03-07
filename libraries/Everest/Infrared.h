#pragma once

#include <Arduino.h>
#include <Buffer.h>

#define BOTTOM_IR_RAMP_THRESHOLD 4.5
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
