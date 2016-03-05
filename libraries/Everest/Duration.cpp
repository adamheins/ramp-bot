#include <Duration.h>

#include <Arduino.h>

Duration::Duration() {
  start = millis();
  current = start;
}

void Duration::tick() {
  current = millis();
}

unsigned long Duration::time() {
  return current - start;
}
