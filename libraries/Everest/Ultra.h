#pragma once

#include <Buffer.h>

#define ULTRA_BUF_LEN 5
#define ULTRA_EDGE_THRESHOLD 5

// Ultrasonic sensor.
class Ultra {
  public:
    Ultra(int pin);
    ~Ultra();
    long ping();     // Read the ultrasonic, storing in buffer.
    int distance(); // Get the distance value from the buffer.
    Edge *edge();

  private:
    Buffer *buffer;
    int pin;
};
