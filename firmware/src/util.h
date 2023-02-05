#ifndef UTIL_H
#define UTIL_H

#include <Arduino.h>

#define ARRAY_SIZE(array) (sizeof(array) / sizeof(array[0]))

void
reset_platform();

void
remove_char(char* str, char c);


int
multi_map(int val,
          int *in,
          int *out,
          uint8_t size);

extern "C" void SystemClock_Config(void);

#endif /* UTIL_H */
