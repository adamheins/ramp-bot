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

void Buffer::insert(int value) {
  arr[index] = value;
  index = (index + 1) % len;
}

int Buffer::average() {
  int sum = 0;
  for (int i = 0; i < len; ++i) {
    sum += arr[i];
  }
  return sum / len;
}

// Average of the older half of the buffer.
int Buffer::old() {
  int sum = 0;
  int i = index;
  for (int count = 0; count < len / 2; ++count) {
    sum += arr[i];
    i = (i + 1) % len;
  }
  return sum * 2 / len;
}

// Average of the more recent half of the buffer.
int Buffer::recent() {
  int sum = 0;
  int i = (index + len / 2) % len;
  for (int count = 0; count < len / 2; ++count) {
    sum += arr[i];
    i = (i + 1) % len;
  }
  return sum * 2 / len;
}

EdgeSide Buffer::edge(int threshold) {
  int r = recent();
  int o = old();
  int diff = abs(r - o);
  if (diff > threshold) {
    if (r > o) {
      return EdgeOld;
    } else {
      return EdgeRecent;
    }
  }
  return EdgeNone;
}

/*
EdgeSide Buffer::edge(int threshold) {
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
      return EdgeRight;
    } else {
      return EdgeLeft;
    }
  }
  return EdgeNone;
}*/
