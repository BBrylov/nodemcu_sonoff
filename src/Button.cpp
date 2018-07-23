#include <Arduino.h>
#include "Button.h"

void Button::update() {
  if (digitalRead(btnPin) == btnLevel) { // Button pressed
    if (_tickPressed < 65535) {
      ++_tickPressed;
      if (_tickPressed == DEBOUNCE_TIME / TICKER_TIME)
        _events->postEvent(Events::EVT_BTNPRESSED);
      else if (_tickPressed == LONG_TIME / TICKER_TIME)
        _events->postEvent(Events::EVT_BTNLONGPRESSED);
    }
  } else { // Button released
    if (_tickDouble)
      --_tickDouble;
    if (_tickPressed) { // Was pressed
      if (_tickPressed >= LONG_TIME / TICKER_TIME) {
        _events->postEvent(Events::EVT_BTNLONGCLICK);
        _tickDouble = 0;
      } else if (_tickPressed >= DEBOUNCE_TIME / TICKER_TIME) {
        if (_tickDouble) {
          _events->postEvent(Events::EVT_BTNDBLCLICK);
          _tickDouble = 0;
        } else {
          _events->postEvent(Events::EVT_BTNCLICK);
          _tickDouble = DOUBLE_TIME / TICKER_TIME;
        }
      } else {
        _tickDouble = 0;
      }
      _tickPressed = 0;
    }
  }
}

Button::buttonstate_t Button::getState() {
  buttonstate_t result = BTN_RELEASED;

  if (digitalRead(btnPin) == btnLevel) { // Button pressed
    if (_tickPressed >= LONG_TIME / TICKER_TIME)
      result = BTN_LONGPRESSED;
    else if (_tickPressed >= DEBOUNCE_TIME / TICKER_TIME)
      result = BTN_PRESSED;
  } else { // Button released
    if (_tickPressed) {
      if (_tickPressed >= LONG_TIME / TICKER_TIME)
        result = BTN_LONGCLICK;
      else if (_tickPressed >= DEBOUNCE_TIME / TICKER_TIME) {
        if (_tickDouble)
          result = BTN_DBLCLICK;
        else
          result = BTN_CLICK;
      }
    }
  }

  return result;
}

void Button::tickerProc(Button *_this) {
  _this->update();
}
