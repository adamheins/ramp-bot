#include <Ultra.h>

#include <Arduino.h>
#include <Buffer.h>

static long usToCm(long us) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return us / 29 / 2;
}

Ultra::Ultra(int pin) {
  this->pin = pin;
  buffer = new Buffer(ULTRA_BUF_LEN);
}

Ultra::~Ultra() {
  delete buffer;
}

long Ultra::ping() {
  // Signal a PING with a HIGH pulse >= 2 microseconds
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delayMicroseconds(2);
  digitalWrite(pin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pin, LOW);

  // Wait for response.
  // NOTE: This takes quite a while, and will slow down panning considerably.
  // ie. Don't read values while panning.
  pinMode(pin, INPUT);
  long duration = pulseIn(pin, HIGH);

  // Convert to cm and insert into the buffer.
  int cm = usToCm(duration);
  buffer->insert(cm);

  return cm;
}

// Average distance from ultrasonic buffer.
int Ultra::distance() {
  return buffer->average();
}

bool Ultra::edge() {
  return buffer->edge();
}
