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
