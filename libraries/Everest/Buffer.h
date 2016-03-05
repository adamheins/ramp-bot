#pragma once

typedef enum EdgeSide {
  EdgeNone = 0,
  EdgeRight,
  EdgeLeft,
} EdgeSide;

class Edge {
  public:
    Edge(EdgeSide side, int dist);
    EdgeSide side;
    int dist;
};

class Buffer {
  public:
    Buffer(int len);
    ~Buffer();
    void insert(int value);
    int average();
    Edge *edge(int threshold);

  private:
    int *arr;
    unsigned int len;
    unsigned int index;
};

