#include <Buffer.h>

#include <Arduino.h>
#include <stdlib.h>

Buffer::Buffer(int len) {
  arr = (int *)calloc(sizeof(int), len);
  this->len = len;
  index = 0;
  sum = 0;
  filled = false;
}

// Reset the buffer to empty state.
void Buffer::reset() {
  for (int i = 0; i < len; ++i) {
    arr[i] = 0;
  }
  index = 0;
  sum = 0;
  filled = false;
}

Buffer::~Buffer() {
  free(arr);
}

// Insert a value into the buffer.
void Buffer::insert(int value) {
  // Update the current sum of the elements in the buffer by removing the old
  // value at this index and adding the new value
  sum += (value - arr[index]);
  arr[index] = value;

  // If we've just filled that last element in the array, we know the buffer
  // must be full.
  filled = filled || (index == len - 1);
  index = (index + 1) % len;
}

// Note the code to handle initial zero-filled buffer.
int Buffer::average() {
  if (filled) {
    return sum / len;
  } else if (index == 0) { // Handle empty case.
    return 0;
  } else {
    return sum / index;
  }
}

bool Buffer::edge() {
  // assume buffer is filled
  int i = index;
  int sum = 0;
  for (int c = 0; c < len / 2; c++) {
    sum += arr[i];
    i = (i + 1) % len;
  }
  int avg1 = sum * 2 / len;
  sum = 0;
  for (int c = 0; c < len / 2; ++c) {
    sum += arr[i];
    i = (i + 1) % len;
  }
  int avg2 = sum * 2 / len;
  return (abs(avg2 - avg1) > 30);
}
