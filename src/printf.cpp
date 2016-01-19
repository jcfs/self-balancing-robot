#include <stdarg.h>
#include <Arduino.h>

void prints(char *fmt, ... ){
  char buf[128] = {0}; // resulting string limited to 128 chars

  va_list args;
  va_start (args, fmt);
  uint8_t i = vsnprintf(buf, 128, fmt, args);
  buf[i]='\r';

  va_end (args);
  Serial.print(buf);
}

void printsf(const char * tag, char * fmt, ...) {
  char buf[128] = {0}; // resulting string limited to 128 chars

  uint8_t n = sprintf(buf, "[%s] ", tag);

  va_list args;
  va_start (args, fmt);
  uint8_t i = vsnprintf(buf+n, 128, fmt, args);
  buf[i+n]='\r';
  va_end (args);
  Serial.print(buf);
}
