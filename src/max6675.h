// this library is public domain. enjoy!
// www.ladyada.net/learn/sensors/thermocouple

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

class MAX6675 {
 public:
 static const uint32_t MEASURE_TIME = 750;
  MAX6675(int8_t SCLK, int8_t CS, int8_t MISO);

  float readCelsius(void);
 private:
  int8_t sclk, miso, cs;
  uint8_t spiread(void);
};
