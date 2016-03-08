#pragma once

#include <Arduino.h>
#include <Buffer.h>

#define ACCEL_BUF_LEN 15

class Accelerometer {
  public:
    Accelerometer(int xPin, int yPin, int zPin);
    ~Accelerometer();
    void ping();
    int x();
    int y();
    int z();

  private:
    int xPin, yPin, zPin;
    Buffer *xBuf, *yBuf, *zBuf;
};
