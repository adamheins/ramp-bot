#pragma once

#include <Buffer.h>

#define ULTRA_BUF_LEN 10

#define ULTRA_LEFT 180
#define ULTRA_CENTRE 90
#define ULTRA_CENTER 90
#define ULTRA_RIGHT 0

// Ultrasonic sensor.
class Ultra {
  public:
    Ultra(int pin);
    ~Ultra();
    long ping();     // Read the ultrasonic, storing in buffer.
    int distance(); // Get the distance value from the buffer.
    void flush();
    bool edge();

  private:
    Buffer *buffer;
    int pin;
};
