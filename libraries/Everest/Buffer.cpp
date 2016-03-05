#include <Buffer.h>

#include <stdlib.h>

Edge::Edge(EdgeSide side, int dist) {
  this->side = side;
  this->dist = dist;
}

Buffer::Buffer(int len) {
  arr = (int *)calloc(sizeof(int), len);
  this->len = len;
}

Buffer::~Buffer() {
  free(arr);
}

void Buffer::insert(int value) {
  index = (index + 1) % len;
  arr[index] = value;
}

int Buffer::average() {
  int sum = 0;
  for (int i = 0; i < len; ++i) {
    sum += arr[i];
  }
  return sum / len;
}

Edge *Buffer::edge(int threshold) {
  int count = 0;      // Track how many elements we've processed so far.
  int half1_sum = 0;  // Sum of elements in the first half of the buffer.
  int half2_sum = 0;  // Sum of elements in the second half of the buffer.

  // Process current position to the end (less recent elements).
  for (int i = index; i < len; ++i) {
    if (count++ < len / 2) {
      half1_sum += arr[i];
    } else {
      half2_sum += arr[i];
    }
  }

  // Process start to current position (more recent elements).
  for (int i = 0; i < index; ++i) {
    if (count++ < len / 2) {
      half1_sum += arr[i];
    } else {
      half2_sum += arr[i];
    }
  }

  // Check if the difference between the two halves exceeds the threshold.
  if (abs(half1_sum - half2_sum) * 2 / len >= threshold) {
    if (half1_sum > half2_sum) {
      return new Edge(EdgeRight, half2_sum / (len / 2));
    } else {
      return new Edge(EdgeLeft, half1_sum / (len / 2));
    }
  }
  return new Edge(EdgeNone, 0);
}
