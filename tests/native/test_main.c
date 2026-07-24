/* SPDX-License-Identifier: GPL-3.0-only */
#include <stdio.h>

unsigned gs_tests_run = 0;
unsigned gs_tests_failed = 0;

void gs_test_protocol(void);
void gs_test_control(void);
void gs_test_architecture(void);
void gs_test_simulation(void);
void gs_test_swd_timing(void);
void gs_test_resync(void);

int main(void) {
  gs_test_protocol();
  gs_test_control();
  gs_test_architecture();
  gs_test_simulation();
  gs_test_swd_timing();
  gs_test_resync();
  if (gs_tests_failed != 0) {
    fprintf(stderr, "%u/%u assertions failed\n", gs_tests_failed, gs_tests_run);
    return 1;
  }
  printf("native tests: %u assertions passed\n", gs_tests_run);
  return 0;
}
