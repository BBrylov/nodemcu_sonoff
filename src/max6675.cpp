// this library is public domain. enjoy!
// www.ladyada.net/learn/sensors/thermocouple

#ifdef __AVR
  #include <avr/pgmspace.h>
  #include <util/delay.h>
#elif defined(ESP8266)
  #include <pgmspace.h>
  //#include <delay.h>
#endif
#include <stdlib.h>
#include "max6675.h"

MAX6675::MAX6675(int8_t SCLK, int8_t CS, int8_t MISO) {
  sclk = SCLK;
  cs = CS;
  miso = MISO;

  //define pin modes
  pinMode(cs, OUTPUT);
  pinMode(sclk, OUTPUT); 
  pinMode(miso, INPUT);

  digitalWrite(cs, HIGH);
}
float MAX6675::readCelsius(void) {

  uint16_t v;

  digitalWrite(cs, LOW);
  #ifdef __AVR
  _delay_ms(1);
  #elif defined(ESP8266)
  delay(1);
  #endif
  

  v = spiread();
  v <<= 8;
  v |= spiread();

  digitalWrite(cs, HIGH);

  if (v & 0x4) {
    // uh oh, no thermocouple attached!
    return NAN; 
    //return -100;
  }

  v >>= 3;

  return v*0.25;
}


byte MAX6675::spiread(void) { 
  int i;
  byte d = 0;

  for (i=7; i>=0; i--)
  {
    digitalWrite(sclk, LOW);
	 #ifdef __AVR
  _delay_ms(1);
  #elif defined(ESP8266)
  delay(1);
  #endif
    if (digitalRead(miso)) {
      //set the bit to 0 no matter what
      d |= (1 << i);
    }

    digitalWrite(sclk, HIGH);
	 #ifdef __AVR
  _delay_ms(1);
  #elif defined(ESP8266)
  delay(1);
  #endif
  }

  return d;
}
