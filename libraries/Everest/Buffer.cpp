#include <Buffer.h>

#include <stdlib.h>

Buffer::Buffer(int len) {
  arr = (int *)calloc(sizeof(int), len);
  this->len = len;
  index = 0;
}

Buffer::~Buffer() {
  free(arr);
}

// Insert a value into the buffer.
void Buffer::insert(int value) {
  // Update the current sum of the elements in the buffer by removing the old
  // value at this index and adding the new value.
  sum += (value - arr[index]);
  arr[index] = value;

  // If we've just filled that last element in the array, we know the buffer
  // must be full.
  filled = (index == len - 1);
  index = (index + 1) % len;
}

// Note the code to handle initial zero-filled buffer.
int Buffer::average() {
  if (filled) {
    return sum / len;
  } else {
    return sum / index;
  }
}
