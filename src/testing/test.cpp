// #include <Arduino.h>
// #include <stdarg.h>

// #define MICROS_PER_SECOND 1000000

// int16_t ax, ay, az;
// int16_t gx, gy, gz;

// short degreesOfTravel = 165;
// short degreesPerSecondMax = 600;
// short degreesPerSecondLimit = 22;
// short elapsedTimeMicros = 0;
// short lastTime = -1;

// unsigned long last = micros();
// unsigned long cur = last;

// void setup()
// {
//   Serial.begin(9600);
//   Serial.println("\nTest begins\n");

// }

// void loop()
// {
//   cur = micros();
//   Serial.println(cur - last);
//   delay(1000);
//   last = cur;
// }

// void p(char *fmt, ...)
// {
//   char buf[128]; // resulting string limited to 128 chars
//   va_list args;
//   va_start(args, fmt);
//   vsnprintf(buf, 128, fmt, args);
//   va_end(args);
//   Serial.print(buf);
// }
