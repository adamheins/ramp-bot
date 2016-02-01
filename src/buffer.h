#pragma once

#define BUF_SIZE 10

// A nice little circular buffer for holding sensor readings.
struct Buffer;
typedef struct Buffer Buffer;

void buffer_insert(Buffer *buf, int value);

int buffer_avg(Buffer *buf);
