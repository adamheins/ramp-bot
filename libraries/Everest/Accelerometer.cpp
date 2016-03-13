#include <Accelerometer.h>

#include <Arduino.h>
#include <Buffer.h>
#include <Counter.h>

Accelerometer::Accelerometer(int xPin, int yPin, int zPin) {
  this->xPin = xPin;
  this->yPin = yPin;
  this->zPin = zPin;

  xBuf = new Buffer(ACCEL_BUF_LEN);
  yBuf = new Buffer(ACCEL_BUF_LEN);
  zBuf = new Buffer(ACCEL_BUF_LEN);

  flatCounter = new Counter(ACCEL_BUF_LEN);
  slopeCounter = new Counter(ACCEL_BUF_LEN);
}

Accelerometer::~Accelerometer() {
  delete xBuf;
  delete yBuf;
  delete zBuf;

  delete flatCounter;
  delete slopeCounter;
}

void Accelerometer::ping() {
  xBuf->insert(analogRead(xPin));
  yBuf->insert(analogRead(yPin));
  zBuf->insert(analogRead(zPin));
}

bool Accelerometer::onSlope() {
  return slopeCounter->countIfElseReset(this->z() < ACCEL_SLOPE_MAX_THRES);
}

bool Accelerometer::onFlat() {
  return flatCounter->countIfElseReset(this->z() > ACCEL_FLAT_MIN_THRES);
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

