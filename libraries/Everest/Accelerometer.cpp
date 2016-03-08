#include <Accelerometer.h>

#include <Arduino.h>
#include <Buffer.h>

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

