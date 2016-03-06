#pragma once

class Duration {
  public:
    Duration(int stop);
    bool tick();
    unsigned long time();

  private:
    unsigned long start;
    unsigned long current;
    unsigned long stop;
};
