#include "../src/buffer.h"

#include "clar/clar.h"

static Buffer buf;

// Run before each of the unit tests.
void test_buffer_initialize(void) {
  buffer_init(&buf);
}

void test_buffer__empty(void) {
  cl_assert_(buffer_avg(&buf) == 0, "Empty buffer should have average of 0.");
}

void test_buffer__insert_one(void) {
  buffer_insert(&buf, 10);
  cl_assert_(buffer_avg(&buf) == 1, "Expected average of 1.");
}

void test_buffer__insert_all(void) {
  for (int i = 0; i < BUF_SIZE; ++i) {
    buffer_insert(&buf, 5);
  }
  cl_assert_(buffer_avg(&buf) == 5, "Expected average of 5.");
}

void test_buffer__insert_overwrite(void) {
  for (int i = 0; i < BUF_SIZE; ++i) {
    buffer_insert(&buf, 5);
  }
  for (int i = 0; i < BUF_SIZE; ++i) {
    buffer_insert(&buf, 1);
  }
  cl_assert_(buffer_avg(&buf) == 1,
             "Expected average of 1. There should be no 5's left in buffer.");
}

