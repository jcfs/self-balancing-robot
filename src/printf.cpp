#include <stdarg.h>
#include <Arduino.h>

void prints(char *fmt, ... ){
  noInterrupts();
  int i = 0;
  char buf[128]; // resulting string limited to 128 chars

  // is this really needed?
  for(i = 0; i < 128; i++) buf[i] = 0;

  va_list args;
  va_start (args, fmt);
  vsnprintf(buf, 128, fmt, args);
  buf[strlen(buf)]='\r';
  va_end (args);
  Serial.print(buf);
  interrupts();
}
