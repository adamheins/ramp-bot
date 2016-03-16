#pragma once

class Counter {
  public:
    Counter(int max);
    bool countIfElseReset(bool cond);
    bool done();

  private:
    int count;
    int max;
};

Counter::Counter(int max) {
  count = 0;
  this->max = max;
}

bool Counter::countIfElseReset(bool cond) {
  if (cond) {
    count++;
  } else {
    count = 0;
  }
  return this->done();
}

bool Counter::done() {
  return count == max;
}
