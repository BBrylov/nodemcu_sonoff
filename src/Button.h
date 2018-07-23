#ifndef __BUTTON_H
#define __BUTTON_H

#include <stdlib.h>
#include <inttypes.h>
#include <Ticker.h>
#include "Events.h"

class Button {
public:
  static const uint32_t TICKER_TIME = 5;
  static const uint32_t DEBOUNCE_TIME = 20; // Milliseconds for debouncing
  static const uint32_t LONG_TIME = 10000; // Milliseconds for short and long click states
  static const uint16_t DOUBLE_TIME = 500; // Milliseconds between double clicks

  enum buttonstate_t : uint8_t { BTN_RELEASED, BTN_PRESSED, BTN_CLICK, BTN_DBLCLICK, BTN_LONGPRESSED, BTN_LONGCLICK };

  Button(const Events *events) : _events((Events*)events), _ticker(Ticker()), _tickPressed(0), _tickDouble(0) {
//    pinMode(btnPin, btnLevel ? INPUT : INPUT_PULLUP);
    pinMode(btnPin, INPUT);
  }
  ~Button() {
    stop();
  }
  void start() {
    _ticker.attach_ms(TICKER_TIME, tickerProc, this);
  }
  void stop() {
    _ticker.detach();
  }
  void update();
  buttonstate_t getState();

protected:
  static const uint8_t btnPin = 0;
  static const bool btnLevel = false;

  static void tickerProc(Button *_this);

  Events *_events;
  Ticker _ticker;
  volatile uint16_t _tickPressed;
  volatile uint8_t _tickDouble;
};

#endif
