// Avoid IDE problems by defining struct in separate .h file.
// Pad record so size is a power of two for best write performance.
// Record size is 64 bytes.
#ifndef Logger_h
#define Logger_h
struct data_t {
  unsigned long time;
  int16_t day;
  int16_t month;
  int32_t year;
  int16_t hour;
  int16_t minute;
  int16_t second;
  int16_t Pin1;
  int16_t Pin2;
  int16_t Pin3;
  int16_t Pin4;
  int16_t Pin5;
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  float mx;
  float my;
  float mz;
};
#endif  // Logger_h
