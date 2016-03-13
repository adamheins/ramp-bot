#include <Counter.h>

Counter::Counter(int max) {
  count = 0;
  this->max = max;
}

bool Counter::countIfElseReset(bool cond) {
  // Increment if condition is met, otherwise reset count to zero.
  if (cond) {
    count++;
  } else {
    count = 0;
  }

  // If we're at the max count, return true and then reset. Otherwise, return
  // false.
  if (count == max) {
    count = 0;
    return true;
  } else {
    return false;
  }
}
