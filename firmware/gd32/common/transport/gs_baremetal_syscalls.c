/* SPDX-License-Identifier: GPL-3.0-only
 * Explicit no-I/O, no-heap newlib hooks for deterministic bare-metal builds.
 */
#include <stddef.h>
#include <sys/stat.h>

int _close(int file) {
  (void)file;
  return -1;
}

int _fstat(int file, struct stat *status) {
  (void)file;
  if (status != NULL) {
    status->st_mode = S_IFCHR;
  }
  return 0;
}

int _isatty(int file) {
  (void)file;
  return 1;
}

int _lseek(int file, int offset, int origin) {
  (void)file;
  (void)offset;
  (void)origin;
  return 0;
}

int _read(int file, char *buffer, int length) {
  (void)file;
  (void)buffer;
  (void)length;
  return -1;
}

int _write(int file, const char *buffer, int length) {
  (void)file;
  (void)buffer;
  return length;
}

void *_sbrk(ptrdiff_t increment) {
  (void)increment;
  return (void *)-1;
}

int _getpid(void) { return 1; }

int _kill(int process, int signal) {
  (void)process;
  (void)signal;
  return -1;
}

__attribute__((noreturn)) void _exit(int status) {
  (void)status;
  for (;;) {
  }
}
