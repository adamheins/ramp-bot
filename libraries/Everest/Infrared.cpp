#include <Infrared.h>

#include <Arduino.h>
#include <Buffer.h>

Infrared::Infrared(int pin) {
  this->pin = pin;
  buffer = new Buffer(IR_BUFFER_LENGTH);
}

Infrared::~Infrared() {
  delete buffer;
}

long Infrared::ping() {
  int distance = analogRead(pin);
  buffer->insert(distance);
}

long Infrared::distance() {
  return buffer->average();
}
