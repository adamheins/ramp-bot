#include <Duration.h>

#include <Arduino.h>

Duration::Duration(int stop) {
  this->stop = stop;
  start = millis();
  current = start;
}

bool Duration::tick() {
  current = millis();
  return stop != 0 && this->time() >= stop;
}

unsigned long Duration::time() {
  return current - start;
}
