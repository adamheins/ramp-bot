#pragma once

#include <Arduino.h>
#include <Buffer.h>

#define ACCEL_BUF_LEN 5

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

Accelerometer::Accelerometer(int xPin, int yPin, int zPin) {
  this->xPin = xPin;
  this->yPin = yPin;
  this->zPin = zPin;

  xBuf = new Buffer(ACCEL_BUF_LEN);
  yBuf = new Buffer(ACCEL_BUF_LEN);
  zBuf = new Buffer(ACCEL_BUF_LEN);
}

Accelerometer::~Accelerometer() {
  delete xBuf;
  delete yBuf;
  delete zBuf;
}

void Accelerometer::ping() {
  xBuf->insert(analogRead(xPin));
  yBuf->insert(analogRead(yPin));
  zBuf->insert(analogRead(zPin));
}

int Accelerometer::x() {
  return xBuf->average();
}

int Accelerometer::y() {
  return yBuf->average();
}

int Accelerometer::z() {
  return zBuf->average();
}
