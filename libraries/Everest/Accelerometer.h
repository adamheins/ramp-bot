#pragma once

#include <Arduino.h>
#include <Buffer.h>

#define ACCEL_BUF_LEN 20

#define ACCEL_FLAT_MIN_THRES 425
#define ACCEL_SLOPE_MAX_THRES 420

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
