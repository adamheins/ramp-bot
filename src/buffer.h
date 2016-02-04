#pragma once

#define BUF_SIZE 10

// A nice little circular buffer for holding sensor readings.
typedef struct Buffer {
  int buffer[BUF_SIZE];
  unsigned int index;
  unsigned int sum; // NOTE we're making an assumption that overflow will not occur
} Buffer;

// Initialize the buffer.
//
// @param buf A pointer to the buffer to initialize.
void buffer_init(Buffer *buf);

// Insert an element into the buffer.
//
// @param buf A pointer to the buffer.
// @param value The value to insert into the buffer.
void buffer_insert(Buffer *buf, int value);

// Calculate the average value of the buffer.
//
// @param buf A pointer to the buffer.
//
// @return The average value of the buffer's contents.
int buffer_avg(Buffer *buf);

// Check if a jump has been detected in the buffer's values.
//
// @param buf A pointer to the buffer.
// @param threshold The value the buffer must have jumped to register as a jump.
//
// @return 0 if the buffer has not jumped, non-zero otherwise:
//         1 if the first half of the buffer has a lower average.
//         2 if the second half of the buffer has a lower average.
int buffer_has_jumped(Buffer *buf, int threshold);
