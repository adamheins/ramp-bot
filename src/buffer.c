#include "buffer.h"

// A nice little circular buffer for holding sensor readings.
struct Buffer {
  unsigned int index;
  int buffer[BUF_SIZE];
};

void buffer_insert(Buffer *buf, int value) {
  buf->index = (buf->index + 1) % BUF_SIZE;
  buf->buffer[buf->index] = value;
}

// This can be optimized by storing and modifying a current average value.
int buffer_avg(Buffer *buf) {
  int sum = 0;
  for (int i = 0; i < BUF_SIZE; ++i) {
    sum += buf->buffer[i]; // NOTE we're making an assumption that overflow will not occur
  }
  return sum / BUF_SIZE;
}
