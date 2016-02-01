#pragma once

#define BUF_SIZE 10

// A nice little circular buffer for holding sensor readings.
struct Buffer;
typedef struct Buffer Buffer;

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
// @return True if the buffer has jumped, false otherwise.
bool buffer_has_jumped(Buffer *buf, int threshold) {
