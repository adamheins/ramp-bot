#pragma once

class Buffer {
  public:
    Buffer(int len);
    ~Buffer();
    void insert(int value);
    int average();

  private:
    int *arr;
    unsigned int len;
    unsigned int index;
    long sum;
    bool filled;
};

