#pragma once

class Duration {
  public:
    Duration();
    void tick();
    unsigned long time();

  private:
    unsigned long start;
    unsigned long current;
};
