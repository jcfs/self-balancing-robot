#include <stdarg.h>
#include <Arduino.h>

void prints(char *fmt, ... ){
  int i = 0;
  char buf[128]; // resulting string limited to 128 chars

  va_list args;
  va_start (args, fmt);
  i = vsnprintf(buf, 128, fmt, args);
  buf[i]='\r';
  buf[i+1]=0;

  va_end (args);
  Serial.print(buf);
}

void printsf(const char * tag, char * fmt, ...) {
  char buf[128];

  int n = sprintf(buf, "[%s] ", tag);

  va_list args;
  va_start (args, fmt);
  int i = vsnprintf(buf+n, 128, fmt, args);
  buf[i+n]='\r';
  buf[i+1+n]=0;
  va_end (args);
  Serial.print(buf);
}
