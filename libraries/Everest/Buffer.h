#pragma once

typedef enum EdgeSide {
  EdgeNone = 0,
  EdgeRecent,
  EdgeOld,
} EdgeSide;

class Buffer {
  public:
    Buffer(int len);
    ~Buffer();
    void insert(int value);
    int average();
    int old();
    int recent();
    EdgeSide edge(int threshold);

  private:
    int *arr;
    unsigned int len;
    unsigned int index;
};

