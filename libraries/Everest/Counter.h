#pragma once

class Counter {
  public:
    Counter(int max);
    bool countIfElseReset(bool cond);

  private:
    int count;
    int max;
};
