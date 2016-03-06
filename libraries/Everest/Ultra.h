#pragma once

#include <Buffer.h>

#define ULTRA_BUF_LEN 10
#define ULTRA_EDGE_THRESHOLD 15

// Ultrasonic sensor.
class Ultra {
  public:
    Ultra(int pin);
    ~Ultra();
    long ping();     // Read the ultrasonic, storing in buffer.
    int distance(); // Get the distance value from the buffer.
    int old();
    int recent();
    EdgeSide edge();

  private:
    Buffer *buffer;
    int pin;
};
