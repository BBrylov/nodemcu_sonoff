#include <pgmspace.h>
#include "ESPWebMQTT.h"
#include "Events.h"
#include "Button.h"
#include "Date.h"
#include "Schedule.h"
#include "RTCmem.h"
#include "DS1820.h"
#include "DHT.h"
#include "max6675.h"


const int8_t maxSchedules = 10; // Количество элементов расписания

const char overSSID[] PROGMEM = "SONOFF_"; // Префикс имени точки доступа по умолчанию
const char overMQTTClient[] PROGMEM = "SONOFF_"; // Префикс имени MQTT-клиента по умолчанию

const uint8_t relayPin = 5;
const bool relayLevel = HIGH; // Уровень срабатывания реле

const bool defRelayOnBoot = false; // Состояние реле при старте модуля по умолчанию
const uint16_t defRelayAutoOff = 0; // Время в секундах до автоотключения реле по умолчанию (0 - нет автоотключения)
const uint16_t defRelayDblClkAutoOff = 60; // Время в секундах до автоотключения реле при двойном нажатии на кнопку по умолчанию
const float defTemperatureTolerance = 0.2; // Порог изменения температуры
const float defHumidityTolerance = 1.0; // Порог изменения влажности

const uint8_t climatePin = 1; // Пин, к которому подключен датчик температуры/влажности


// Пины, к котором подключен датчик max6675
const uint8_t MAX_MISO = 12;
const uint8_t  MAX_CS = 15;
const uint8_t MAXS_CLK = 14;

const char pathRelay[] PROGMEM = "/relay"; // Путь до страницы настройки параметров реле
const char pathControl[] PROGMEM = "/control"; // Путь до страницы настройки параметров кнопок/ДУ
const char pathSwitch[] PROGMEM = "/switch"; // Путь до страницы управления переключением реле
const char pathSchedules[] PROGMEM = "/schedules"; // Путь до страницы настройки параметров расписания
const char pathGetSchedule[] PROGMEM = "/getschedule"; // Путь до страницы, возвращающей JSON-пакет элемента расписания
const char pathSetSchedule[] PROGMEM = "/setschedule"; // Путь до страницы изменения элемента расписания
const char pathClimate[] PROGMEM = "/climate"; // Путь до страницы настройки параметров датчика температуры

// Имена параметров для Web-форм
const char paramRelayOnBoot[] PROGMEM = "relayonboot";
const char paramRelayAutoOff[] PROGMEM = "relayautooff";
const char paramRelayDblClkAutoOff[] PROGMEM = "relaydblclkautooff";
const char paramRelayName[] PROGMEM = "relayname";
const char paramSchedulePeriod[] PROGMEM = "period";
const char paramScheduleHour[] PROGMEM = "hour";
const char paramScheduleMinute[] PROGMEM = "minute";
const char paramScheduleSecond[] PROGMEM = "second";
const char paramScheduleWeekdays[] PROGMEM = "weekdays";
const char paramScheduleDay[] PROGMEM = "day";
const char paramScheduleMonth[] PROGMEM = "month";
const char paramScheduleYear[] PROGMEM = "year";
const char paramScheduleRelay[] PROGMEM = "relay";
const char paramScheduleTurn[] PROGMEM = "turn";
const char paramClimateSensor[] PROGMEM = "climatsensor";
const char paramClimateTempTolerance[] PROGMEM = "climatetemptolerance";
const char paramClimateHumTolerance[] PROGMEM = "climatehumtolerance";
const char paramClimateMinTemp[] PROGMEM = "climatemintemp";
const char paramClimateMaxTemp[] PROGMEM = "climatemaxtemp";
const char paramClimateMinTempTurn[] PROGMEM = "climatemintempturn";
const char paramClimateMaxTempTurn[] PROGMEM = "climatemaxtempturn";
const char paramClimateMinHum[] PROGMEM = "climateminhum";
const char paramClimateMaxHum[] PROGMEM = "climatemaxhum";
const char paramClimateMinHumTurn[] PROGMEM = "climateminhumturn";
const char paramClimateMaxHumTurn[] PROGMEM = "climatemaxhumturn";

// Имена JSON-переменных
const char jsonRelay[] PROGMEM = "relay";
const char jsonRelayAutoOff[] PROGMEM = "relayautooff";
const char jsonSchedulePeriod[] PROGMEM = "period";
const char jsonScheduleHour[] PROGMEM = "hour";
const char jsonScheduleMinute[] PROGMEM = "minute";
const char jsonScheduleSecond[] PROGMEM = "second";
const char jsonScheduleWeekdays[] PROGMEM = "weekdays";
const char jsonScheduleDay[] PROGMEM = "day";
const char jsonScheduleMonth[] PROGMEM = "month";
const char jsonScheduleYear[] PROGMEM = "year";
const char jsonScheduleRelay[] PROGMEM = "relay";
const char jsonScheduleTurn[] PROGMEM = "turn";
const char jsonTemperature[] PROGMEM = "temperature";
const char jsonHumidity[] PROGMEM = "humidity";

// Названия топиков для MQTT
const char mqttRelayTopic[] PROGMEM = "/Relay";
const char mqttTemperatureTopic[] PROGMEM = "/Temperature";
const char mqttHumidityTopic[] PROGMEM = "/Humidity";

const char strNone[] PROGMEM = "(None)";

const uint8_t RELAY_NAME_SIZE = 16;

class ESPWebMQTTRelay : public ESPWebMQTTBase {
public:
  ESPWebMQTTRelay() : ESPWebMQTTBase() {}

  void reboot();

protected:
  void setupExtra();
  void loopExtra();

  String getHostName();

  uint16_t readRTCmemory();
  uint16_t writeRTCmemory();
  uint16_t readConfig();
  uint16_t writeConfig(bool commit = true);
  void defaultConfig(uint8_t level = 0);
  bool setConfigParam(const String &name, const String &value);

  void setupHttpServer();
  void handleRootPage();
  String jsonData();
  void handleRelayConfig(); // Обработчик страницы настройки параметров реле
  void handleRelaySwitch(); // Обработчик страницы управления переключением реле
  void handleSchedulesConfig(); // Обработчик страницы настройки параметров расписания
  void handleGetSchedule(); // Обработчик страницы, возвращающей JSON-пакет элемента расписания
  void handleSetSchedule(); // Обработчик страницы изменения элемента расписания
  void handleClimateConfig(); // Обработчик страницы настройки параметров датчика температуры

  String navigator();
  String btnRelayConfig(); // HTML-код кнопки вызова настройки реле
  String btnSchedulesConfig(); // HTML-код кнопки вызова настройки расписания
  String btnClimateConfig(); // HTML-код кнопки вызова настройки датчика температуры

  void mqttCallback(char* topic, byte* payload, unsigned int length);
  void mqttResubscribe();

  void pulseLed();

private:
  void switchRelay(bool on, uint16_t customAutoOff = 0); // Процедура включения/выключения реле
  void toggleRelay(); // Процедура переключения реле

  uint16_t readSchedulesConfig(uint16_t offset); // Чтение из EEPROM порции параметров расписания
  uint16_t writeSchedulesConfig(uint16_t offset); // Запись в EEPROM порции параметров расписания

  void publishTemperature(); // Публикация температуры в MQTT
  void publishHumidity(); // Публикация влажности в MQTT

  struct relay_t {
    bool relayOnBoot; // Состояние реле при старте модуля
    uint16_t relayAutoOff; // Значения задержки реле в секундах до автоотключения (0 - нет автоотключения)
    uint16_t relayDblClkAutoOff; // Значения задержки реле в секундах до автоотключения при двойном нажатии на кнопку
    char relayName[RELAY_NAME_SIZE]; // Название реле
  } relay;
  uint32_t autoOff; // Значения в миллисекундах для сравнения с millis(), когда реле должно отключиться автоматически (0 - нет автоотключения)
  uint32_t lastState; // Битовое поле состояния реле для воостановления после перезагрузки

  Events *events;
  Button *button;

  enum turn_t : uint8_t { TURN_OFF, TURN_ON, TURN_TOGGLE };

  Schedule schedules[maxSchedules]; // Массив расписания событий
  turn_t scheduleTurns[maxSchedules]; // Что делать с реле по срабатыванию события

  enum sensor_t : uint8_t { SENSOR_NONE, SENSOR_MAX6675, SENSOR_DS1820, SENSOR_DHT11, SENSOR_DHT21, SENSOR_DHT22 };

  uint32_t climateReadTime; // Время в миллисекундах, после которого можно считывать новое значение сенсоров
  float climateTempTolerance; // Порог изменения температуры
  float climateHumTolerance; // Порог изменения влажности
  float climateTemperature; // Значение успешно прочитанной температуры
  float climateHumidity; // Значение успешно прочитанной влажности
  float climateMinTemp, climateMaxTemp; // Минимальное и максимальное значение температуры срабатывания реле
  turn_t climateMinTempTurn, climateMaxTempTurn; // Что делать с реле по срабатыванию события
  float climateMinHum, climateMaxHum; // Минимальное и максимальное значение влажности срабатывания реле
  turn_t climateMinHumTurn, climateMaxHumTurn; // Что делать с реле по срабатыванию события
  struct {
    sensor_t climateSensor : 3;
    bool climateMinTempTriggered : 1;
    bool climateMaxTempTriggered : 1; // Было ли срабатывание реле по порогу температуры?
    bool climateMinHumTriggered : 1;
    bool climateMaxHumTriggered : 1; // Было ли срабатывание реле по порогу влажности?
  };

  union {
    DS1820 *ds;
    DHT *dht;
    MAX6675 *Max6675;
  };
};

static String charBufToString(const char* str, uint16_t bufSize) {
  String result;

  result.reserve(bufSize);
  while (bufSize-- && *str) {
    result += *str++;
  }

  return result;
}

/***
 * ESPWebMQTTRelay class implemenattion
 */

void ESPWebMQTTRelay::reboot() {
  button->stop();

  ESPWebMQTTBase::reboot();
}

void ESPWebMQTTRelay::setupExtra() {
  ESPWebMQTTBase::setupExtra();

  autoOff = 0;

  if (lastState != (uint32_t)-1) {
    if (lastState & 0x01) {
      digitalWrite(relayPin, relayLevel);
      if (relay.relayAutoOff)
        autoOff = millis() + relay.relayAutoOff * 1000;
    } else
      digitalWrite(relayPin, ! relayLevel);
  } else
    digitalWrite(relayPin, relay.relayOnBoot == relayLevel);
  pinMode(relayPin, OUTPUT);

  events = new Events();
  button = new Button(events);
  button->start();

  if (climateSensor != SENSOR_NONE) {
    if (climateSensor == SENSOR_DS1820) {
      ds = new DS1820(climatePin);
      if (! ds->find()) {
        _log->println(F("DS18x20 device not detected!"));
        delete ds;
        ds = NULL;
      } else {
        ds->update();
        climateReadTime = millis() + ds->MEASURE_TIME;
      }
    } else 
    {
  if (climateSensor == SENSOR_MAX6675) {      
      Max6675 = new MAX6675(MAXS_CLK, MAX_CS, MAX_MISO);
      
        climateReadTime = millis() + Max6675->MEASURE_TIME;
     } else 
    { 
      
      // climateSensor == SENSOR_DHTx
      uint8_t type;

      if (climateSensor == SENSOR_DHT11)
        type = DHT11;
      else if (climateSensor == SENSOR_DHT21)
        type = DHT21;
      else if (climateSensor == SENSOR_DHT22)
        type = DHT22;
      dht = new DHT(climatePin, type);
      dht->begin();
      climateReadTime = millis() + 2000;
    }
    }
  }
  climateTemperature = NAN;
  climateMinTempTriggered = false;
  climateMaxTempTriggered = false;
  climateHumidity = NAN;
  climateMinHumTriggered = false;
  climateMaxHumTriggered = false;
}

void ESPWebMQTTRelay::loopExtra() {
  ESPWebMQTTBase::loopExtra();

  if (autoOff && ((int32_t)millis() >= (int32_t)autoOff)) {
    switchRelay(false);
    autoOff = 0;
  }

  uint32_t now = getTime();

  while (Events::event_t *evt = events->getEvent()) {
    if (evt->type == Events::EVT_BTNCLICK) {
      toggleRelay();
      logDateTime(now);
      _log->println(F(" button pressed"));
    } else if (evt->type == Events::EVT_BTNDBLCLICK) {
      if (relay.relayDblClkAutoOff)
        switchRelay(true, relay.relayDblClkAutoOff);
      else
        toggleRelay();
      logDateTime(now);
      _log->println(F(" button double pressed"));
    } else if (evt->type == Events::EVT_BTNLONGPRESSED) {
      enablePulse(FASTPULSE);
    } else if (evt->type == Events::EVT_BTNLONGCLICK) {
      clearEEPROM();
      reboot();
    }
  }

  if (now) {
    for (int8_t i = 0; i < maxSchedules; ++i) {
      if (schedules[i].period() != Schedule::NONE) {
        if (schedules[i].check(now)) {
          if (scheduleTurns[i] == TURN_TOGGLE)
            toggleRelay();
          else
            switchRelay(scheduleTurns[i] == TURN_ON);
          logDateTime(now);
          _log->print(F(" schedule \""));
          _log->print(schedules[i]);
          _log->print(F("\" turned relay "));
          if (scheduleTurns[i] == TURN_TOGGLE)
            _log->println(F("opposite"));
          else
            _log->println(scheduleTurns[i] == TURN_ON ? F("on") : F("off"));
        }
      }
    }
  }

  if (climateSensor != SENSOR_NONE) {
    if (climateSensor == SENSOR_DS1820) {
      if (ds) {
        if ((int32_t)millis() >= (int32_t)climateReadTime) {
          float v;
    
          v = ds->readTemperature();
          ds->update();
          if (! isnan(v) && (v >= -50.0) && (v <= 120.0)) {
            if (isnan(climateTemperature) || (abs(climateTemperature - v) > climateTempTolerance)) {
              climateTemperature = v;
              publishTemperature();
              if (! isnan(climateMinTemp)) {
                if (climateTemperature < climateMinTemp) {
                  if (! climateMinTempTriggered) {
                    if (climateMinTempTurn == TURN_TOGGLE)
                      toggleRelay();
                    else
                      switchRelay(climateMinTempTurn == TURN_ON);
                    logDateTime(now);
                    _log->println(F(" DS1820 minimal temperature triggered"));
                    climateMinTempTriggered = true;
                  }
                } else
                  climateMinTempTriggered = false;
              }
              if (! isnan(climateMaxTemp)) {
                if (climateTemperature > climateMaxTemp) {
                  if (! climateMaxTempTriggered) {
                    if (climateMaxTempTurn == TURN_TOGGLE)
                      toggleRelay();
                    else
                      switchRelay(climateMaxTempTurn == TURN_ON);
                    logDateTime(now);
                    _log->println(F(" DS1820 maximal temperature triggered"));
                    climateMaxTempTriggered = true;
                  }
                } else
                  climateMaxTempTriggered = false;
              }
            }
          } else {
            logDateTime(now);
            _log->println(F(" DS1820 temperature read error!"));
          }
          climateReadTime = millis() + ds->MEASURE_TIME;
        }
      }
    } else {
   if (climateSensor == SENSOR_MAX6675) {
      if (Max6675) {
        if ((int32_t)millis() >= (int32_t)climateReadTime) {
          float v;
    
          v = Max6675->readCelsius();
          
          if (! isnan(v) && (v >= -10.0) && (v <= 150.0)) {
            if (isnan(climateTemperature) || (abs(climateTemperature - v) > climateTempTolerance)) {
              climateTemperature = v;
              publishTemperature();
              if (! isnan(climateMinTemp)) {
                if (climateTemperature < climateMinTemp) {
                  if (! climateMinTempTriggered) {
                    if (climateMinTempTurn == TURN_TOGGLE)
                      toggleRelay();
                    else
                      switchRelay(climateMinTempTurn == TURN_ON);
                    logDateTime(now);
                    _log->println(F(" Max6675 minimal temperature triggered"));
                    climateMinTempTriggered = true;
                  }
                } else
                  climateMinTempTriggered = false;
              }
              if (! isnan(climateMaxTemp)) {
                if (climateTemperature > climateMaxTemp) {
                  if (! climateMaxTempTriggered) {
                    if (climateMaxTempTurn == TURN_TOGGLE)
                      toggleRelay();
                    else
                      switchRelay(climateMaxTempTurn == TURN_ON);
                    logDateTime(now);
                    _log->println(F(" Max6675 maximal temperature triggered"));
                    climateMaxTempTriggered = true;
                  }
                } else
                  climateMaxTempTriggered = false;
              }
            }
          } else {
            logDateTime(now);
            _log->println(F(" Max6675 temperature read error!"));
          }
          climateReadTime = millis() + ds->MEASURE_TIME;
        }
      }
    } else {      
      
      
      
      // climateSensor == SENSOR_DHTx
      if (dht) {
        if ((int32_t)millis() >= (int32_t)climateReadTime) {
          float v;
    
          v = dht->readTemperature();
          if (! isnan(v) && (v >= -50.0) && (v <= 50.0)) {
            if (isnan(climateTemperature) || (abs(climateTemperature - v) > climateTempTolerance)) {
              climateTemperature = v;
              publishTemperature();
              if (! isnan(climateMinTemp)) {
                if (climateTemperature < climateMinTemp) {
                  if (! climateMinTempTriggered) {
                    if (climateMinTempTurn == TURN_TOGGLE)
                      toggleRelay();
                    else
                      switchRelay(climateMinTempTurn == TURN_ON);
                    logDateTime(now);
                    _log->println(F(" DHTx minimal temperature triggered"));
                    climateMinTempTriggered = true;
                  }
                } else
                  climateMinTempTriggered = false;
              }
              if (! isnan(climateMaxTemp)) {
                if (climateTemperature > climateMaxTemp) {
                  if (! climateMaxTempTriggered) {
                    if (climateMaxTempTurn == TURN_TOGGLE)
                      toggleRelay();
                    else
                      switchRelay(climateMaxTempTurn == TURN_ON);
                    logDateTime(now);
                    _log->println(F(" DHTx maximal temperature triggered"));
                    climateMaxTempTriggered = true;
                  }
                } else
                  climateMaxTempTriggered = false;
              }
            }
          } else {
            logDateTime(now);
            _log->println(F(" DHTx temperature read error!"));
          }

          v = dht->readHumidity();
          if (! isnan(v) && (v >= 0.0) && (v <= 100.0)) {
            if (isnan(climateHumidity) || (abs(climateHumidity - v) > climateHumTolerance)) {
              climateHumidity = v;
              publishHumidity();
              if (! isnan(climateMinHum)) {
                if (climateHumidity < climateMinHum) {
                  if (! climateMinHumTriggered) {
                    if (climateMinHumTurn == TURN_TOGGLE)
                      toggleRelay();
                    else
                      switchRelay(climateMinHumTurn == TURN_ON);
                    logDateTime(now);
                    _log->println(F(" DHTx minimal humidity triggered"));
                    climateMinHumTriggered = true;
                  }
                } else
                  climateMinHumTriggered = false;
              }
              if (! isnan(climateMaxHum)) {
                if (climateHumidity > climateMaxHum) {
                  if (! climateMaxHumTriggered) {
                    if (climateMaxHumTurn == TURN_TOGGLE)
                      toggleRelay();
                    else
                      switchRelay(climateMaxHumTurn == TURN_ON);
                    logDateTime(now);
                    _log->println(F(" DHTx maximal humidity triggered"));
                    climateMaxHumTriggered = true;
                  }
                } else
                  climateMaxHumTriggered = false;
              }
            }
          } else {
            logDateTime(now);
            _log->println(F(" DHTx humidity read error!"));
          }

          climateReadTime = millis() + 2000;
        }
        }
      }
    }
  }
}

String ESPWebMQTTRelay::getHostName() {
  String result;

  result = FPSTR(overSSID);
  result += getBoardId();

  return result;
}

uint16_t ESPWebMQTTRelay::readRTCmemory() {
  uint16_t offset = ESPWebMQTTBase::readRTCmemory();

  if (offset) {
    uint32_t controlState;

    RTCmem.get(offset, lastState);
    offset += sizeof(lastState);
    RTCmem.get(offset, controlState);
    offset += sizeof(controlState);
    if (controlState != ~lastState) {
      _log->println(F("Last relay state in RTC memory is illegal!"));
      lastState = (uint32_t)-1;
    } else {
      _log->println(F("Last relay state restored from RTC memory"));
    }
  } else
    lastState = (uint32_t)-1;

  return offset;
}

uint16_t ESPWebMQTTRelay::writeRTCmemory() {
  uint16_t offset = ESPWebMQTTBase::writeRTCmemory();

  if (offset) {
    uint32_t controlState;

    controlState = ~lastState;
    RTCmem.put(offset, lastState);
    offset += sizeof(lastState);
    RTCmem.put(offset, controlState);
    offset += sizeof(controlState);
    _log->println(F("Last relay state stored to RTC memory"));
  }

  return offset;
}

uint16_t ESPWebMQTTRelay::readConfig() {
  uint16_t offset = ESPWebMQTTBase::readConfig();

  if (offset) {
    uint16_t start = offset;

    getEEPROM(offset, relay);
    offset += sizeof(relay);

    offset = readSchedulesConfig(offset);

    sensor_t _climateSensor; // Bit-field workaround

    getEEPROM(offset, _climateSensor);
    offset += sizeof(_climateSensor);
    climateSensor = _climateSensor;

    getEEPROM(offset, climateTempTolerance);
    offset += sizeof(climateTempTolerance);
    getEEPROM(offset, climateHumTolerance);
    offset += sizeof(climateHumTolerance);
    getEEPROM(offset, climateMinTemp);
    offset += sizeof(climateMinTemp);
    getEEPROM(offset, climateMaxTemp);
    offset += sizeof(climateMaxTemp);
    getEEPROM(offset, climateMinTempTurn);
    offset += sizeof(climateMinTempTurn);
    getEEPROM(offset, climateMaxTempTurn);
    offset += sizeof(climateMaxTempTurn);
    getEEPROM(offset, climateMinHum);
    offset += sizeof(climateMinHum);
    getEEPROM(offset, climateMaxHum);
    offset += sizeof(climateMaxHum);
    getEEPROM(offset, climateMinHumTurn);
    offset += sizeof(climateMinHumTurn);
    getEEPROM(offset, climateMaxHumTurn);
    offset += sizeof(climateMaxHumTurn);

    uint8_t crc = crc8EEPROM(start, offset);
    if (readEEPROM(offset++) != crc) {
      _log->println(F("CRC mismatch! Use default relay parameters."));
      defaultConfig(2);
    }
  }

  return offset;
}

uint16_t ESPWebMQTTRelay::writeConfig(bool commit) {
  uint16_t offset = ESPWebMQTTBase::writeConfig(false);
  uint16_t start = offset;

  putEEPROM(offset, relay);
  offset += sizeof(relay);

  offset = writeSchedulesConfig(offset);

  sensor_t _climateSensor = climateSensor; // Bit-field workaround

  putEEPROM(offset, _climateSensor);
  offset += sizeof(_climateSensor);

  putEEPROM(offset, climateTempTolerance);
  offset += sizeof(climateTempTolerance);
  putEEPROM(offset, climateHumTolerance);
  offset += sizeof(climateHumTolerance);
  putEEPROM(offset, climateMinTemp);
  offset += sizeof(climateMinTemp);
  putEEPROM(offset, climateMaxTemp);
  offset += sizeof(climateMaxTemp);
  putEEPROM(offset, climateMinTempTurn);
  offset += sizeof(climateMinTempTurn);
  putEEPROM(offset, climateMaxTempTurn);
  offset += sizeof(climateMaxTempTurn);
  putEEPROM(offset, climateMinHum);
  offset += sizeof(climateMinHum);
  putEEPROM(offset, climateMaxHum);
  offset += sizeof(climateMaxHum);
  putEEPROM(offset, climateMinHumTurn);
  offset += sizeof(climateMinHumTurn);
  putEEPROM(offset, climateMaxHumTurn);
  offset += sizeof(climateMaxHumTurn);

  uint8_t crc = crc8EEPROM(start, offset);
  writeEEPROM(offset++, crc);
  if (commit)
    commitConfig();

  return offset;
}

void ESPWebMQTTRelay::defaultConfig(uint8_t level) {
  if (level < 2) {
    ESPWebMQTTBase::defaultConfig(level);

    if (level < 1) {
      _ssid = FPSTR(overSSID);
      _ssid += getBoardId();
    }
    _mqttClient = FPSTR(overMQTTClient);
    _mqttClient += getBoardId();
  }

  if (level < 3) {
    relay.relayOnBoot = defRelayOnBoot;
    relay.relayAutoOff = defRelayAutoOff;
    relay.relayDblClkAutoOff = defRelayDblClkAutoOff;
    memset(relay.relayName, 0, sizeof(relay.relayName));

    for (uint8_t i = 0; i < maxSchedules; ++i) {
      schedules[i].clear();
      scheduleTurns[i] = TURN_OFF;
    }

    climateSensor = SENSOR_NONE;
    climateTempTolerance = defTemperatureTolerance;
    climateHumTolerance = defHumidityTolerance;
    climateMinTemp = NAN;
    climateMaxTemp = NAN;
    climateMinTempTurn = TURN_OFF;
    climateMaxTempTurn = TURN_OFF;
    climateMinHum = NAN;
    climateMaxHum = NAN;
    climateMinHumTurn = TURN_OFF;
    climateMaxHumTurn = TURN_OFF;
  }
}

bool ESPWebMQTTRelay::setConfigParam(const String &name, const String &value) {
  if (! ESPWebMQTTBase::setConfigParam(name, value)) {
    if (name.equals(FPSTR(paramRelayOnBoot))) {
      relay.relayOnBoot = constrain(value.toInt(), 0, 1);
    } else if (name.equals(FPSTR(paramRelayAutoOff))) {
      relay.relayAutoOff = _max(0, value.toInt());
    } else if (name.equals(FPSTR(paramRelayDblClkAutoOff))) {
      relay.relayDblClkAutoOff = _max(0, value.toInt());
    } else if (name.equals(FPSTR(paramRelayName))) {
      strncpy(relay.relayName, value.c_str(), sizeof(relay.relayName));
    } else if (name.equals(FPSTR(paramClimateSensor))) {
      sensor_t _climateSensor = (sensor_t)value.toInt();

      if (climateSensor != _climateSensor) {
        if ((climateSensor != SENSOR_NONE) && ds) { // or dht
          if (climateSensor == SENSOR_DS1820) {
            delete ds;
            ds = NULL;
          } else { // climateSensor == SENSOR_DHTx
            delete dht;
            dht = NULL;
          }
        }
        climateSensor = _climateSensor;
      }
    } else if (name.equals(FPSTR(paramClimateTempTolerance))) {
      climateTempTolerance = _max(0, value.toFloat());
    } else if (name.equals(FPSTR(paramClimateHumTolerance))) {
      climateHumTolerance = _max(0, value.toFloat());
    } else if (name.equals(FPSTR(paramClimateMinTemp))) {
      if (value.length())
        climateMinTemp = value.toFloat();
      else
        climateMinTemp = NAN;
    } else if (name.equals(FPSTR(paramClimateMaxTemp))) {
      if (value.length())
        climateMaxTemp = value.toFloat();
      else
        climateMaxTemp = NAN;
    } else if (name.equals(FPSTR(paramClimateMinTempTurn))) {
      climateMinTempTurn = (turn_t)value.toInt();
    } else if (name.equals(FPSTR(paramClimateMaxTempTurn))) {
      climateMaxTempTurn = (turn_t)value.toInt();
    } else if (name.equals(FPSTR(paramClimateMinHum))) {
      if (value.length())
        climateMinHum = value.toFloat();
      else
        climateMinHum = NAN;
    } else if (name.equals(FPSTR(paramClimateMaxHum))) {
      if (value.length())
        climateMaxHum = value.toFloat();
      else
        climateMaxHum = NAN;
    } else if (name.equals(FPSTR(paramClimateMinHumTurn))) {
      climateMinHumTurn = (turn_t)value.toInt();
    } else if (name.equals(FPSTR(paramClimateMaxHumTurn))) {
      climateMaxHumTurn = (turn_t)value.toInt();
    } else
      return false;
  }

  return true;
}

void ESPWebMQTTRelay::setupHttpServer() {
  ESPWebMQTTBase::setupHttpServer();

  httpServer->on(String(FPSTR(pathRelay)).c_str(), std::bind(&ESPWebMQTTRelay::handleRelayConfig, this));
  httpServer->on(String(FPSTR(pathSwitch)).c_str(), std::bind(&ESPWebMQTTRelay::handleRelaySwitch, this));
  httpServer->on(String(FPSTR(pathClimate)).c_str(), std::bind(&ESPWebMQTTRelay::handleClimateConfig, this));
  httpServer->on(String(FPSTR(pathSchedules)).c_str(), std::bind(&ESPWebMQTTRelay::handleSchedulesConfig, this));
  httpServer->on(String(FPSTR(pathGetSchedule)).c_str(), std::bind(&ESPWebMQTTRelay::handleGetSchedule, this));
  httpServer->on(String(FPSTR(pathSetSchedule)).c_str(), std::bind(&ESPWebMQTTRelay::handleSetSchedule, this));
}

void ESPWebMQTTRelay::handleRootPage() {
  if (! userAuthenticate())
    return;

  String style = F(".checkbox {\n\
vertical-align:top;\n\
margin:0 3px 0 0;\n\
width:17px;\n\
height:17px;\n\
}\n\
.checkbox + label {\n\
cursor:pointer;\n\
}\n\
.checkbox:not(checked) {\n\
position:absolute;\n\
opacity:0;\n\
}\n\
.checkbox:not(checked) + label {\n\
position:relative;\n\
padding:0 0 0 60px;\n\
}\n\
.checkbox:not(checked) + label:before {\n\
content:'';\n\
position:absolute;\n\
top:-4px;\n\
left:0;\n\
width:50px;\n\
height:26px;\n\
border-radius:13px;\n\
background:#CDD1DA;\n\
box-shadow:inset 0 2px 3px rgba(0,0,0,.2);\n\
}\n\
.checkbox:not(checked) + label:after {\n\
content:'';\n\
position:absolute;\n\
top:-2px;\n\
left:2px;\n\
width:22px;\n\
height:22px;\n\
border-radius:10px;\n\
background:#FFF;\n\
box-shadow:0 2px 5px rgba(0,0,0,.3);\n\
transition:all .2s;\n\
}\n\
.checkbox:checked + label:before {\n\
background:#9FD468;\n\
}\n\
.checkbox:checked + label:after {\n\
left:26px;\n\
}\n");

  String script = F("function switchRelay(on) {\n\
openUrl('");
  script += FPSTR(pathSwitch);
  script += F("?on=' + on + '&autooff=' + ");
  script += FPSTR(getElementById);
  script += FPSTR(jsonRelayAutoOff);
  script += F("').value + '&dummy=' + Date.now());\n\
}\n\
function uptimeToStr(uptime) {\n\
var tm, uptimestr = '';\n\
if (uptime >= 86400)\n\
uptimestr = parseInt(uptime / 86400) + ' day(s) ';\n\
tm = parseInt(uptime % 86400 / 3600);\n\
if (tm < 10)\n\
uptimestr += '0';\n\
uptimestr += tm + ':';\n\
tm = parseInt(uptime % 3600 / 60);\n\
if (tm < 10)\n\
uptimestr += '0';\n\
uptimestr += tm + ':';\n\
tm = parseInt(uptime % 60);\n\
if (tm < 10)\n\
uptimestr += '0';\n\
uptimestr += tm;\n\
return uptimestr;\n\
}\n\
function refreshData() {\n\
var request = getXmlHttpRequest();\n\
request.open('GET', '");
  script += FPSTR(pathData);
  script += F("?dummy=' + Date.now(), true);\n\
request.onreadystatechange = function() {\n\
if (request.readyState == 4) {\n\
var data = JSON.parse(request.responseText);\n");
  script += FPSTR(getElementById);
  script += FPSTR(jsonMQTTConnected);
  script += F("').innerHTML = (data.");
  script += FPSTR(jsonMQTTConnected);
  script += F(" != true ? \"not \" : \"\") + \"connected\";\n");
  script += FPSTR(getElementById);
  script += FPSTR(jsonFreeHeap);
  script += F("').innerHTML = data.");
  script += FPSTR(jsonFreeHeap);
  script += F(";\n");
  script += FPSTR(getElementById);
  script += FPSTR(jsonUptime);
  script += F("').innerHTML = uptimeToStr(data.");
  script += FPSTR(jsonUptime);
  script += F(");\n");
  if (WiFi.getMode() == WIFI_STA) {
    script += FPSTR(getElementById);
    script += FPSTR(jsonRSSI);
    script += F("').innerHTML = data.");
    script += FPSTR(jsonRSSI);
    script += F(";\n");
  }
  script += FPSTR(getElementById);
  script += FPSTR(jsonRelay);
  script += F("').checked = data.");
  script += FPSTR(jsonRelay);
  script += F(";\n\
if (data.");
  script += FPSTR(jsonRelay);
  script += F(" == true)\n");
  script += FPSTR(getElementById);
  script += FPSTR(jsonRelayAutoOff);
  script += F("').value = data.");
  script += FPSTR(jsonRelayAutoOff);
  script += F(";\n");
  if (climateSensor != SENSOR_NONE) {
    if (ds) { // or dht
      script += FPSTR(getElementById);
      script += FPSTR(jsonTemperature);
      script += F("').innerHTML = data.");
      script += FPSTR(jsonTemperature);
      script += F(";\n");
    }
    if ((climateSensor >= SENSOR_DHT11) && dht) {
      script += FPSTR(getElementById);
      script += FPSTR(jsonHumidity);
      script += F("').innerHTML = data.");
      script += FPSTR(jsonHumidity);
      script += F(";\n");
    }
  }
  script += F("}\n\
}\n\
request.send(null);\n\
}\n\
setInterval(refreshData, 500);\n");

  String page = ESPWebBase::webPageStart(F("Sonoff Relay"));
  page += ESPWebBase::webPageStdStyle();
  page += ESPWebBase::webPageStyle(style);
  page += ESPWebBase::webPageStdScript();
  page += ESPWebBase::webPageScript(script);
  page += ESPWebBase::webPageBody();
  page += F("<h3>Sonoff Relay</h3>\n\
<p>\n\
MQTT broker: <span id=\"");
  page += FPSTR(jsonMQTTConnected);
  page += F("\">?</span><br/>\n\
Heap free size: <span id=\"");
  page += FPSTR(jsonFreeHeap);
  page += F("\">0</span> bytes<br/>\n\
Uptime: <span id=\"");
  page += FPSTR(jsonUptime);
  page += F("\">?</span><br/>\n");
  if (WiFi.getMode() == WIFI_STA) {
    page += F("Signal strength: <span id=\"");
    page += FPSTR(jsonRSSI);
    page += F("\">?</span> dBm<br/>\n");
  }
  if (climateSensor != SENSOR_NONE) {
    if (climateSensor == SENSOR_DS1820) {
      if (ds) {
        page += F("Temperature: <span id=\"");
        page += FPSTR(jsonTemperature);
        page += F("\">?</span> <sup>o</sup>C<br/>\n");
      }
    } else {
      if (climateSensor == SENSOR_MAX6675) {
      if (Max6675) {
        page += F("Temperature: <span id=\"");
        page += FPSTR(jsonTemperature);
        page += F("\">?</span> <sup>o</sup>C<br/>\n");
      }
    } else {
      // climateSensor == SENSOR_DHTx
      if (dht) {
        page += F("Temperature: <span id=\"");
        page += FPSTR(jsonTemperature);
        page += F("\">?</span> <sup>o</sup>C<br/>\n");
        page += F("Humidity: <span id=\"");
        page += FPSTR(jsonHumidity);
        page += F("\">?</span> %<br/>\n");
      }
    } 
    }
  }
  page += F("</p>\n");
  page += F("<input type=\"checkbox\" class=\"checkbox\" id=\"");
  page += FPSTR(jsonRelay);
  page += F("\" onchange=\"switchRelay(this.checked)\" ");
  if (digitalRead(relayPin) == relayLevel)
    page += FPSTR(extraChecked);
  page += F(">\n\
<label for=\"");
  page += FPSTR(jsonRelay);
  page += F("\">");
  if (relay.relayName[0])
    page += charBufToString(relay.relayName, sizeof(relay.relayName));
  else
    page += F("Relay");
  page += F("</label>\n\
<input type=\"text\" id=\"");
  page += FPSTR(jsonRelayAutoOff);
  page += F("\" value=\"");
  page += String(relay.relayAutoOff);
  page += F("\" size=5 maxlength=5 />\n\
sec. to auto-off\n\
<p>\n");
  page += navigator();
  page += ESPWebBase::webPageEnd();

  httpServer->send(200, FPSTR(textHtml), page);
}

String ESPWebMQTTRelay::jsonData() {
  String result = ESPWebMQTTBase::jsonData();

  result += F(",\"");
  result += FPSTR(jsonRelay);
  result += F("\":");
  if (digitalRead(relayPin) == relayLevel)
    result += FPSTR(bools[1]);
  else
    result += FPSTR(bools[0]);
  result += F(",\"");
  result += FPSTR(jsonRelayAutoOff);
  result += F("\":");
  if (autoOff)
    result += String(((int32_t)autoOff - (int32_t)millis()) / 1000);
  else
    result += '0';
  if (climateSensor != SENSOR_NONE) {
    if (ds) { // or dht
      result += F(",\"");
      result += FPSTR(jsonTemperature);
      result += F("\":");
      result += isnan(climateTemperature) ? F("\"?\"") : String(climateTemperature);
    }
    if ((climateSensor >= SENSOR_DHT11) && dht) {
      result += F(",\"");
      result += FPSTR(jsonHumidity);
      result += F("\":");
      result += isnan(climateHumidity) ? F("\"?\"") : String(climateHumidity);
    }
  }

  return result;
}

void ESPWebMQTTRelay::handleRelayConfig() {
  if (! adminAuthenticate())
    return;

  String page = ESPWebBase::webPageStart(F("Relay Setup"));
  page += ESPWebBase::webPageStdStyle();
  page += ESPWebBase::webPageBody();
  page += F("<form name=\"relay\" method=\"GET\" action=\"");
  page += FPSTR(pathStore);
  page += F("\">\n\
<h3>Relay Setup</h3>\n\
<label>On boot:</label><br/>\n\
<input type=\"radio\" name=\"");
  page += FPSTR(paramRelayOnBoot);
  page += F("\" value=\"1\" ");
  if (relay.relayOnBoot)
    page += FPSTR(extraChecked);
  page += F(">ON\n\
<input type=\"radio\" name=\"");
  page += FPSTR(paramRelayOnBoot);
  page += F("\" value=\"0\" ");
  if (! relay.relayOnBoot)
    page += FPSTR(extraChecked);
  page += F(">OFF<br/>\n\
<label>Auto off<sup>*</sup>:</label><br/>\n\
<input type=\"text\" name=\"");
  page += FPSTR(paramRelayAutoOff);
  page += F("\" value=\"");
  page += String(relay.relayAutoOff);
  page += F("\" size=5 maxlength=5><br/>\n\
<label>Double click button auto off<sup>*</sup>:</label><br/>\n\
<input type=\"text\" name=\"");
  page += FPSTR(paramRelayDblClkAutoOff);
  page += F("\" value=\"");
  page += String(relay.relayDblClkAutoOff);
  page += F("\" size=5 maxlength=5><br/>\n\
<label>Relay name:</label><br/>\n\
<input type=\"text\" name=\"");
  page += FPSTR(paramRelayName);
  page += F("\" value=\"");
  page += escapeQuote(charBufToString(relay.relayName, sizeof(relay.relayName)));
  page += F("\" size=");
  page += String(sizeof(relay.relayName));
  page += F(" maxlength=");
  page += String(sizeof(relay.relayName));
  page += F("><br/>\n\
<sup>*</sup> time in seconds to auto off relay (0 to disable this feature)\n\
<p>\n");

  page += ESPWebBase::tagInput(FPSTR(typeSubmit), strEmpty, F("Save"));
  page += charLF;
  page += btnBack();
  page += ESPWebBase::tagInput(FPSTR(typeHidden), FPSTR(paramReboot), "1");
  page += F("\n\
</form>\n");
  page += ESPWebBase::webPageEnd();

  httpServer->send(200, FPSTR(textHtml), page);
}

void ESPWebMQTTRelay::handleRelaySwitch() {
  String on = httpServer->arg(F("on"));
  uint16_t customAutoOff = 0;

  if (httpServer->hasArg(F("autooff")))
    customAutoOff = _max(0, httpServer->arg(F("autooff")).toInt());
  switchRelay(on.equals(F("true")), customAutoOff);
  logDateTime();
  _log->print(F(" relay turned "));
  if (on.equals(F("true")))
    _log->print(F("on"));
  else
    _log->print(F("off"));
  _log->print(F(" with auto off after "));
  _log->print(customAutoOff);
  _log->println(F(" sec. by web interface"));

  httpServer->send(200, FPSTR(textPlain), strEmpty);
}

void ESPWebMQTTRelay::handleSchedulesConfig() {
  if (! adminAuthenticate())
    return;

  int8_t i;

  String style = F(".modal {\n\
display: none;\n\
position: fixed;\n\
z-index: 1;\n\
left: 0;\n\
top: 0;\n\
width: 100%;\n\
height: 100%;\n\
overflow: auto;\n\
background-color: rgb(0,0,0);\n\
background-color: rgba(0,0,0,0.4);\n\
}\n\
.modal-content {\n\
background-color: #fefefe;\n\
margin: 15% auto;\n\
padding: 20px;\n\
border: 1px solid #888;\n\
width: 400px;\n\
}\n\
.close {\n\
color: #aaa;\n\
float: right;\n\
font-size: 28px;\n\
font-weight: bold;\n\
}\n\
.close:hover,\n\
.close:focus {\n\
color: black;\n\
text-decoration: none;\n\
cursor: pointer;\n\
}\n\
.hidden {\n\
display: none;\n\
}\n");

  String script = F("function loadData(form) {\n\
var request = getXmlHttpRequest();\n\
request.open('GET', '");
  script += FPSTR(pathGetSchedule);
  script += F("?id=' + form.id.value + '&dummy=' + Date.now(), false);\n\
request.send(null);\n\
if (request.status == 200) {\n\
var data = JSON.parse(request.responseText);\n\
form.");
  script += FPSTR(paramSchedulePeriod);
  script += F(".value = data.");
  script += FPSTR(jsonSchedulePeriod);
  script += F(";\n\
form.");
  script += FPSTR(paramScheduleHour);
  script += F(".value = data.");
  script += FPSTR(jsonScheduleHour);
  script += F(";\n\
form.");
  script += FPSTR(paramScheduleMinute);
  script += F(".value = data.");
  script += FPSTR(jsonScheduleMinute);
  script += F(";\n\
form.");
  script += FPSTR(paramScheduleSecond);
  script += F(".value = data.");
  script += FPSTR(jsonScheduleSecond);
  script += F(";\n\
if (data.");
  script += FPSTR(jsonSchedulePeriod);
  script += F(" == 3) {\n\
var weekdaysdiv = document.getElementById('weekdays');\n\
var elements = weekdaysdiv.getElementsByTagName('input');\n\
for (var i = 0; i < elements.length; i++) {\n\
if (elements[i].type == 'checkbox') {\n\
if ((data.");
  script += FPSTR(jsonScheduleWeekdays);
  script += F(" & elements[i].value) != 0)\n\
elements[i].checked = true;\n\
else\n\
elements[i].checked = false;\n\
}\n\
}\n\
form.");
  script += FPSTR(paramScheduleWeekdays);
  script += F(".value = data.");
  script += FPSTR(jsonScheduleWeekdays);
  script += F(";\n\
} else {\n\
form.");
  script += FPSTR(paramScheduleWeekdays);
  script += F(".value = 0;\n\
form.");
  script += FPSTR(paramScheduleDay);
  script += F(".value = data.");
  script += FPSTR(jsonScheduleDay);
  script += F(";\n\
form.");
  script += FPSTR(paramScheduleMonth);
  script += F(".value = data.");
  script += FPSTR(jsonScheduleMonth);
  script += F(";\n\
form.");
  script += FPSTR(paramScheduleYear);
  script += F(".value = data.");
  script += FPSTR(jsonScheduleYear);
  script += F(";\n\
}\n\
var radios = document.getElementsByName('");
  script += FPSTR(paramScheduleTurn);
  script += F("');\n\
for (var i = 0; i < radios.length; i++) {\n\
if (radios[i].value == data.");
  script += FPSTR(jsonScheduleTurn);
  script += F(") radios[i].checked = true;\n\
}\n\
}\n\
}\n\
function openForm(form, id) {\n\
form.id.value = id;\n\
loadData(form);\n\
form.");
  script += FPSTR(paramSchedulePeriod);
  script += F(".onchange();\n\
document.getElementById(\"form\").style.display = \"block\";\n\
}\n\
function closeForm() {\n\
document.getElementById(\"form\").style.display = \"none\";\n\
}\n\
function checkNumber(field, minvalue, maxvalue) {\n\
var val = parseInt(field.value);\n\
if (isNaN(val) || (val < minvalue) || (val > maxvalue))\n\
return false;\n\
return true;\n\
}\n\
function validateForm(form) {\n\
if (form.");
  script += FPSTR(paramSchedulePeriod);
  script += F(".value > 0) {\n\
if ((form.");
  script += FPSTR(paramSchedulePeriod);
  script += F(".value > 2) && (! checkNumber(form.");
  script += FPSTR(paramScheduleHour);
  script += F(", 0, 23))) {\n\
alert(\"Wrong hour!\");\n\
form.");
  script += FPSTR(paramScheduleHour);
  script += F(".focus();\n\
return false;\n\
}\n\
if ((form.");
  script += FPSTR(paramSchedulePeriod);
  script += F(".value > 1) && (! checkNumber(form.");
  script += FPSTR(paramScheduleMinute);
  script += F(", 0, 59))) {\n\
alert(\"Wrong minute!\");\n\
form.");
  script += FPSTR(paramScheduleMinute);
  script += F(".focus();\n\
return false;\n\
}\n\
if (! checkNumber(form.");
  script += FPSTR(paramScheduleSecond);
  script += F(", 0, 59)) {\n\
alert(\"Wrong second!\");\n\
form.");
  script += FPSTR(paramScheduleSecond);
  script += F(".focus();\n\
return false;\n\
}\n\
if ((form.");
  script += FPSTR(paramSchedulePeriod);
  script += F(".value == 3) && (form.");
  script += FPSTR(paramScheduleWeekdays);
  script += F(".value == 0)) {\n\
alert(\"None of weekdays selected!\");\n\
return false;\n\
}\n\
if ((form.");
  script += FPSTR(paramSchedulePeriod);
  script += F(".value >= 4) && (! checkNumber(form.");
  script += FPSTR(paramScheduleDay);
  script += F(", 1, ");
  script += String(Schedule::LASTDAYOFMONTH);
  script += F("))) {\n\
alert(\"Wrong day!\");\n\
form.");
  script += FPSTR(paramScheduleDay);
  script += F(".focus();\n\
return false;\n\
}\n\
if ((form.");
  script += FPSTR(paramSchedulePeriod);
  script += F(".value >= 5) && (! checkNumber(form.");
  script += FPSTR(paramScheduleMonth);
  script += F(", 1, 12))) {\n\
alert(\"Wrong month!\");\n\
form.");
  script += FPSTR(paramScheduleMonth);
  script += F(".focus();\n\
return false;\n\
}\n\
if ((form.");
  script += FPSTR(paramSchedulePeriod);
  script += F(".value == 6) && (! checkNumber(form.");
  script += FPSTR(paramScheduleYear);
  script += F(", 2017, 2099))) {\n\
alert(\"Wrong year!\");\n\
form.");
  script += FPSTR(paramScheduleYear);
  script += F(".focus();\n\
return false;\n\
}\n\
var radios = document.getElementsByName('");
  script += FPSTR(paramScheduleTurn);
  script += F("');\n\
var checkedCount = 0;\n\
for (var i = 0; i < radios.length; i++) {\n\
if (radios[i].checked == true) checkedCount++;\n\
}\n\
if (checkedCount != 1) {\n\
alert(\"Wrong relay turn!\");\n\
return false;\n\
}\n\
}\n\
return true;\n\
}\n\
function periodChanged(period) {\n\
document.getElementById(\"time\").style.display = (period.value != 0) ? \"inline\" : \"none\";\n\
document.getElementById(\"hh\").style.display = (period.value > 2) ? \"inline\" : \"none\";\n\
document.getElementById(\"mm\").style.display = (period.value > 1) ? \"inline\" : \"none\";\n\
document.getElementById(\"weekdays\").style.display = (period.value == 3) ? \"block\" : \"none\";\n\
document.getElementById(\"date\").style.display = (period.value > 3) ? \"block\" : \"none\";\n\
document.getElementById(\"month\").style.display = (period.value > 4) ? \"inline\" : \"none\";\n\
document.getElementById(\"year\").style.display = (period.value == 6) ? \"inline\" : \"none\";\n\
document.getElementById(\"relay\").style.display = (period.value != 0) ? \"block\" : \"none\";\n\
}\n\
function weekChanged(wd) {\n\
var weekdays = document.form.");
  script += FPSTR(paramScheduleWeekdays);
  script += F(".value;\n\
if (wd.checked == \"\") weekdays &= ~wd.value; else weekdays |= wd.value;\n\
document.form.");
  script += FPSTR(paramScheduleWeekdays);
  script += F(".value = weekdays;\n\
}\n");

  String page = ESPWebBase::webPageStart(F("Schedules Setup"));
  page += ESPWebBase::webPageStdStyle();
  page += ESPWebBase::webPageStyle(style);
  page += ESPWebBase::webPageStdScript();
  page += ESPWebBase::webPageScript(script);
  page += ESPWebBase::webPageBody();
  page += F("<table><caption><h3>Schedules Setup</h3></caption>\n\
<tr><th>#</th><th>Event</th><th>Next time</th><th>Relay</th></tr>\n");

  for (i = 0; i < maxSchedules; ++i) {
    page += F("<tr><td><a href=\"#\" onclick=\"openForm(document.form, ");
    page += String(i);
    page += F(")\">");
    page += String(i + 1);
    page += F("</a></td><td>");
    page += schedules[i];
    page += F("</td><td>");
    page += schedules[i].nextTimeStr();
    page += F("</td><td>");
    if (schedules[i].period() != Schedule::NONE) {
      page += F("Relay ");
      if (scheduleTurns[i] == TURN_TOGGLE)
        page += F("toggle");
      else
        page += (scheduleTurns[i] == TURN_ON) ? F("on") : F("off");
    }
    page += F("</td></tr>\n");
  }
  page += F("</table>\n\
<p>\n\
<i>Don't forget to save changes!</i>\n\
<p>\n");

  page += ESPWebBase::tagInput(FPSTR(typeButton), strEmpty, F("Save"), String(F("onclick=\"location.href='")) + String(FPSTR(pathStore)) + String(F("?reboot=0'\"")));
  page += charLF;
  page += btnBack();
  page += ESPWebBase::tagInput(FPSTR(typeHidden), FPSTR(paramReboot), "0");
  page += F("\n\
<div id=\"form\" class=\"modal\">\n\
<div class=\"modal-content\">\n\
<span class=\"close\" onclick=\"closeForm()\">&times;</span>\n\
<form name=\"form\" method=\"GET\" action=\"");
  page += FPSTR(pathSetSchedule);
  page += F("\" onsubmit=\"if (validateForm(this)) closeForm(); else return false;\">\n\
<input type=\"hidden\" name=\"id\" value=\"0\">\n\
<select name=\"");
  page += FPSTR(paramSchedulePeriod);
  page += F("\" size=\"1\" onchange=\"periodChanged(this)\">\n\
<option value=\"0\">Never!</option>\n\
<option value=\"1\">Every minute</option>\n\
<option value=\"2\">Every hour</option>\n\
<option value=\"3\">Every week</option>\n\
<option value=\"4\">Every month</option>\n\
<option value=\"5\">Every year</option>\n\
<option value=\"6\">Once</option>\n\
</select>\n\
<span id=\"time\" class=\"hidden\">at\n\
<span id=\"hh\" class=\"hidden\">");
  page += ESPWebBase::tagInput(typeText, FPSTR(paramScheduleHour), "0", F("size=2 maxlength=2"));
  page += F("\n:</span>\n\
<span id=\"mm\" class=\"hidden\">");
  page += ESPWebBase::tagInput(typeText, FPSTR(paramScheduleMinute), "0", F("size=2 maxlength=2"));
  page += F("\n:</span>\n");
  page += ESPWebBase::tagInput(typeText, FPSTR(paramScheduleSecond), "0", F("size=2 maxlength=2"));
  page += F("</span><br/>\n\
<div id=\"weekdays\" class=\"hidden\">\n\
<input type=\"hidden\" name=\"");
  page += FPSTR(paramScheduleWeekdays);
  page += F("\" value=\"0\">\n");

  for (i = 0; i < 7; i++) {
    page += F("<input type=\"checkbox\" value=\"");
    page += String(1 << i);
    page += F("\" onchange=\"weekChanged(this)\">");
    page += weekdayName(i);
    page += charLF;
  }
  page += F("</div>\n\
<div id=\"date\" class=\"hidden\">\n\
<select name=\"");
  page += FPSTR(paramScheduleDay);
  page += F("\" size=\"1\">\n");

  for (i = 1; i <= 31; i++) {
    page += F("<option value=\"");
    page += String(i);
    page += F("\">");
    page += String(i);
    page += F("</option>\n");
  }
  page += F("<option value=\"");
  page += String(Schedule::LASTDAYOFMONTH);
  page += F("\">Last</option>\n\
</select>\n\
day\n\
<span id=\"month\" class=\"hidden\">of\n\
<select name=\"");
  page += FPSTR(paramScheduleMonth);
  page += F("\" size=\"1\">\n");

  for (i = 1; i <= 12; i++) {
    page += F("<option value=\"");
    page += String(i);
    page += F("\">");
    page += monthName(i);
    page += F("</option>\n");
  }
  page += F("</select>\n\
</span>\n\
<span id=\"year\" class=\"hidden\">");
  page += ESPWebBase::tagInput(typeText, FPSTR(paramScheduleYear), "2017", F("size=4 maxlength=4"));
  page += F("</span>\n\
</div>\n\
<div id=\"relay\" class=\"hidden\">\n\
<label>Turn relay</label>\n\
<input type=\"radio\" name=\"");
  page += FPSTR(paramScheduleTurn);
  page += F("\" value=\"0\">OFF\n\
<input type=\"radio\" name=\"");
  page += FPSTR(paramScheduleTurn);
  page += F("\" value=\"1\">ON\n\
<input type=\"radio\" name=\"");
  page += FPSTR(paramScheduleTurn);
  page += F("\" value=\"2\">TOGGLE\n\
</div>\n\
<p>\n\
<input type=\"submit\" value=\"Update\">\n\
</form>\n\
</div>\n\
</div>\n");
  page += ESPWebBase::webPageEnd();

  httpServer->send(200, FPSTR(textHtml), page);
}

void ESPWebMQTTRelay::handleGetSchedule() {
  int id = -1;

  if (httpServer->hasArg("id"))
    id = httpServer->arg("id").toInt();

  if ((id >= 0) && (id < maxSchedules)) {
    String page;

    page += charOpenBrace;
    page += charQuote;
    page += FPSTR(jsonSchedulePeriod);
    page += F("\":");
    page += String(schedules[id].period());
    page += F(",\"");
    page += FPSTR(jsonScheduleHour);
    page += F("\":");
    page += String(schedules[id].hour());
    page += F(",\"");
    page += FPSTR(jsonScheduleMinute);
    page += F("\":");
    page += String(schedules[id].minute());
    page += F(",\"");
    page += FPSTR(jsonScheduleSecond);
    page += F("\":");
    page += String(schedules[id].second());
    page += F(",\"");
    page += FPSTR(jsonScheduleWeekdays);
    page += F("\":");
    page += String(schedules[id].weekdays());
    page += F(",\"");
    page += FPSTR(jsonScheduleDay);
    page += F("\":");
    page += String(schedules[id].day());
    page += F(",\"");
    page += FPSTR(jsonScheduleMonth);
    page += F("\":");
    page += String(schedules[id].month());
    page += F(",\"");
    page += FPSTR(jsonScheduleYear);
    page += F("\":");
    page += String(schedules[id].year());
    page += F(",\"");
    page += FPSTR(jsonScheduleTurn);
    page += F("\":");
    page += String(scheduleTurns[id]);
    page += charCloseBrace;

    httpServer->send(200, FPSTR(textJson), page);
  } else {
    httpServer->send(204, FPSTR(textJson), strEmpty); // No content
  }
}

void ESPWebMQTTRelay::handleSetSchedule() {
  String argName, argValue;
  int8_t id = -1;
  Schedule::period_t period = Schedule::NONE;
  int8_t hour = -1;
  int8_t minute = -1;
  int8_t second = -1;
  uint8_t weekdays = 0;
  int8_t day = 0;
  int8_t month = 0;
  int16_t year = 0;
  turn_t turn = TURN_OFF;

  for (byte i = 0; i < httpServer->args(); i++) {
    argName = httpServer->argName(i);
    argValue = httpServer->arg(i);
    if (argName.equals("id")) {
      id = argValue.toInt();
    } else if (argName.equals(FPSTR(paramSchedulePeriod))) {
      period = (Schedule::period_t)argValue.toInt();
    } else if (argName.equals(FPSTR(paramScheduleHour))) {
      hour = argValue.toInt();
    } else if (argName.equals(FPSTR(paramScheduleMinute))) {
      minute = argValue.toInt();
    } else if (argName.equals(FPSTR(paramScheduleSecond))) {
      second = argValue.toInt();
    } else if (argName.equals(FPSTR(paramScheduleWeekdays))) {
      weekdays = argValue.toInt();
    } else if (argName.equals(FPSTR(paramScheduleDay))) {
      day = argValue.toInt();
    } else if (argName.equals(FPSTR(paramScheduleMonth))) {
      month = argValue.toInt();
    } else if (argName.equals(FPSTR(paramScheduleYear))) {
      year = argValue.toInt();
    } else if (argName.equals(FPSTR(paramScheduleTurn))) {
      turn = (turn_t)argValue.toInt();
    } else {
      _log->print(F("Unknown parameter \""));
      _log->print(argName);
      _log->print(F("\"!"));
    }
  }

  if ((id >= 0) && (id < maxSchedules)) {
    if (period == Schedule::NONE)
      schedules[id].clear();
    else
      schedules[id].set(period, hour, minute, second, weekdays, day, month, year);
    scheduleTurns[id] = turn;

    String page = ESPWebBase::webPageStart(F("Store Schedule"));
    page += F("<meta http-equiv=\"refresh\" content=\"1;URL=");
    page += FPSTR(pathSchedules);
    page += F("\">\n");
    page += ESPWebBase::webPageStdStyle();
    page += ESPWebBase::webPageBody();
    page += F("Configuration stored successfully.\n\
Wait for 1 sec. to return to previous page.\n");
    page += ESPWebBase::webPageEnd();

    httpServer->send(200, FPSTR(textHtml), page);
  } else {
    httpServer->send(204, FPSTR(textHtml), strEmpty);
  }
}

void ESPWebMQTTRelay::handleClimateConfig() {
  if (! adminAuthenticate())
    return;

  String page = ESPWebBase::webPageStart(F("Climate Setup"));
  page += ESPWebBase::webPageStdStyle();
  page += ESPWebBase::webPageBody();
  page += F("<form name=\"climate\" method=\"GET\" action=\"");
  page += FPSTR(pathStore);
  page += F("\">\n");
  page += F("<h3>Climate Setup</h3>\n\
<label>Climate sensor:</label><br/>\n\
<select name=\"");
  page += FPSTR(paramClimateSensor);
  page += F("\">\n\
<option value=\"");
  page += String(SENSOR_NONE);
  if (climateSensor == SENSOR_NONE)
    page += F("\" selected>");
  else
    page += F("\">");
  page += FPSTR(strNone);
  page += F("</option>\n\
<option value=\"");
  page += String(SENSOR_DS1820);
  if (climateSensor == SENSOR_DS1820)
    page += F("\" selected>");
  else
    page += F("\">");
  page += F("DS18x20</option>\n\
<option value=\"");
  page += String(SENSOR_MAX6675);
  if (climateSensor == SENSOR_MAX6675)
    page += F("\" selected>");
  else
    page += F("\">");
  page += F("MAX6675</option>\n\
<option value=\"");

  
  page += String(SENSOR_DHT11);
  if (climateSensor == SENSOR_DHT11)
    page += F("\" selected>");
  else
    page += F("\">");
  page += F("DHT11</option>\n\
<option value=\"");
  page += String(SENSOR_DHT21);
  if (climateSensor == SENSOR_DHT21)
    page += F("\" selected>");
  else
    page += F("\">");
  page += F("DHT21</option>\n\
<option value=\"");
  page += String(SENSOR_DHT22);
  if (climateSensor == SENSOR_DHT22)
    page += F("\" selected>");
  else
    page += F("\">");
  page += F("DHT22</option>\n\
</select><br/>\n\
<label>Temperature tolerance:</label><br/>\n\
<input type=\"text\" name=\"");
  page += FPSTR(paramClimateTempTolerance);
  page += F("\" value=\"");
  page += String(climateTempTolerance);
  page += F("\" size=10 maxlength=10><br/>\n\
<label>Humidity tolerance:</label><br/>\n\
<input type=\"text\" name=\"");
  page += FPSTR(paramClimateHumTolerance);
  page += F("\" value=\"");
  page += String(climateHumTolerance);
  page += F("\" size=10 maxlength=10><br/>\n\
<label>Minimal temperature:</label><br/>\n\
<input type=\"text\" name=\"");
  page += FPSTR(paramClimateMinTemp);
  page += F("\" value=\"");
  if (! isnan(climateMinTemp))
    page += String(climateMinTemp);
  page += F("\" size=10 maxlength=10>\n\
(leave blank if not used)<br/>\n\
<label>Turn relay</label>\n\
<input type=\"radio\" name=\"");
  page += FPSTR(paramClimateMinTempTurn);
  page += F("\" value=\"0\"");
  if (climateMinTempTurn == TURN_OFF)
    page += F(" checked");
  page += F(">OFF\n\
<input type=\"radio\" name=\"");
  page += FPSTR(paramClimateMinTempTurn);
  page += F("\" value=\"1\"");
  if (climateMinTempTurn == TURN_ON)
    page += F(" checked");
  page += F(">ON\n\
<input type=\"radio\" name=\"");
  page += FPSTR(paramClimateMinTempTurn);
  page += F("\" value=\"2\"");
  if (climateMinTempTurn == TURN_TOGGLE)
    page += F(" checked");
  page += F(">TOGGLE<br/>\n\
<label>Maximal temperature:</label><br/>\n\
<input type=\"text\" name=\"");
  page += FPSTR(paramClimateMaxTemp);
  page += F("\" value=\"");
  if (! isnan(climateMaxTemp))
    page += String(climateMaxTemp);
  page += F("\" size=10 maxlength=10>\n\
(leave blank if not used)<br/>\n\
<label>Turn relay</label>\n\
<input type=\"radio\" name=\"");
  page += FPSTR(paramClimateMaxTempTurn);
  page += F("\" value=\"0\"");
  if (climateMaxTempTurn == TURN_OFF)
    page += F(" checked");
  page += F(">OFF\n\
<input type=\"radio\" name=\"");
  page += FPSTR(paramClimateMaxTempTurn);
  page += F("\" value=\"1\"");
  if (climateMaxTempTurn == TURN_ON)
    page += F(" checked");
  page += F(">ON\n\
<input type=\"radio\" name=\"");
  page += FPSTR(paramClimateMaxTempTurn);
  page += F("\" value=\"2\"");
  if (climateMaxTempTurn == TURN_TOGGLE)
    page += F(" checked");
  page += F(">TOGGLE<br/>\n\
<label>Minimal humidity:</label><br/>\n\
<input type=\"text\" name=\"");
  page += FPSTR(paramClimateMinHum);
  page += F("\" value=\"");
  if (! isnan(climateMinHum))
    page += String(climateMinHum);
  page += F("\" size=10 maxlength=10>\n\
(leave blank if not used)<br/>\n\
<label>Turn relay</label>\n\
<input type=\"radio\" name=\"");
  page += FPSTR(paramClimateMinHumTurn);
  page += F("\" value=\"0\"");
  if (climateMinHumTurn == TURN_OFF)
    page += F(" checked");
  page += F(">OFF\n\
<input type=\"radio\" name=\"");
  page += FPSTR(paramClimateMinHumTurn);
  page += F("\" value=\"1\"");
  if (climateMinHumTurn == TURN_ON)
    page += F(" checked");
  page += F(">ON\n\
<input type=\"radio\" name=\"");
  page += FPSTR(paramClimateMinHumTurn);
  page += F("\" value=\"2\"");
  if (climateMinHumTurn == TURN_TOGGLE)
    page += F(" checked");
  page += F(">TOGGLE<br/>\n\
<label>Maximal humidity:</label><br/>\n\
<input type=\"text\" name=\"");
  page += FPSTR(paramClimateMaxHum);
  page += F("\" value=\"");
  if (! isnan(climateMaxHum))
    page += String(climateMaxHum);
  page += F("\" size=10 maxlength=10>\n\
(leave blank if not used)<br/>\n\
<label>Turn relay</label>\n\
<input type=\"radio\" name=\"");
  page += FPSTR(paramClimateMaxHumTurn);
  page += F("\" value=\"0\"");
  if (climateMaxHumTurn == TURN_OFF)
    page += F(" checked");
  page += F(">OFF\n\
<input type=\"radio\" name=\"");
  page += FPSTR(paramClimateMaxHumTurn);
  page += F("\" value=\"1\"");
  if (climateMaxHumTurn == TURN_ON)
    page += F(" checked");
  page += F(">ON\n\
<input type=\"radio\" name=\"");
  page += FPSTR(paramClimateMaxHumTurn);
  page += F("\" value=\"2\"");
  if (climateMaxHumTurn == TURN_TOGGLE)
    page += F(" checked");
  page += F(">TOGGLE<br/>\n\
<p>\n");

  page += ESPWebBase::tagInput(FPSTR(typeSubmit), strEmpty, F("Save"));
  page += charLF;
  page += btnBack();
  page += ESPWebBase::tagInput(FPSTR(typeHidden), FPSTR(paramReboot), "1");
  page += F("\n\
</form>\n");
  page += ESPWebBase::webPageEnd();

  httpServer->send(200, FPSTR(textHtml), page);
}

String ESPWebMQTTRelay::navigator() {
  String result = btnWiFiConfig();
  result += btnTimeConfig();
  result += btnMQTTConfig();
  result += btnRelayConfig();
  result += btnSchedulesConfig();
  result += btnClimateConfig();
  result += btnLog();
  result += btnReboot();

  return result;
}

String ESPWebMQTTRelay::btnRelayConfig() {
  String result = ESPWebBase::tagInput(FPSTR(typeButton), strEmpty, F("Relay Setup"), String(F("onclick=\"location.href='")) + String(FPSTR(pathRelay)) + String(F("'\"")));
  result += charLF;

  return result;
}

String ESPWebMQTTRelay::btnSchedulesConfig() {
  String result = ESPWebBase::tagInput(FPSTR(typeButton), strEmpty, F("Schedules Setup"), String(F("onclick=\"location.href='")) + String(FPSTR(pathSchedules)) + String(F("'\"")));
  result += charLF;

  return result;
}

String ESPWebMQTTRelay::btnClimateConfig() {
  String result = ESPWebBase::tagInput(FPSTR(typeButton), strEmpty, F("Climate Setup"), String(F("onclick=\"location.href='")) + String(FPSTR(pathClimate)) + String(F("'\"")));
  result += charLF;

  return result;
}

void ESPWebMQTTRelay::mqttCallback(char* topic, byte* payload, unsigned int length) {
  ESPWebMQTTBase::mqttCallback(topic, payload, length);

  String _mqttTopic = FPSTR(mqttRelayTopic);
  char* topicBody = topic + _mqttClient.length() + 1; // Skip "/ClientName" from topic
  if (! strcmp(topicBody, _mqttTopic.c_str())) {
    bool relay = digitalRead(relayPin);

    if (! relayLevel)
      relay = ! relay;
    if ((char)payload[0] == '0') {
      if (relay)
        switchRelay(false);
    } else if ((char)payload[0] == '1') {
      if (! relay)
        switchRelay(true);
    } else {
      mqttPublish(String(topic), String(relay));
    }
  } else {
    _log->println(F("Unexpected topic!"));
  }
}

void ESPWebMQTTRelay::mqttResubscribe() {
  String topic;

  if (_mqttClient != strEmpty) {
    topic += charSlash;
    topic += _mqttClient;
  }
  topic += FPSTR(mqttRelayTopic);
  mqttPublish(topic, String(digitalRead(relayPin) == relayLevel));

  mqttSubscribe(topic);
}

void ESPWebMQTTRelay::switchRelay(bool on, uint16_t customAutoOff) {
  bool relayState = digitalRead(relayPin);

  if (! relayLevel)
    relayState = ! relayState;

  if (relay.relayAutoOff || customAutoOff) {
    if (on) {
      if (customAutoOff)
        autoOff = millis() + customAutoOff * 1000;
      else
        autoOff = millis() + relay.relayAutoOff * 1000;
    } else
      autoOff = 0;
  }

  if (relayState != on) {
    digitalWrite(relayPin, relayLevel == on);

    if (pubSubClient->connected()) {
      String topic;

      if (_mqttClient != strEmpty) {
        topic += charSlash;
        topic += _mqttClient;
      }
      topic += FPSTR(mqttRelayTopic);
      mqttPublish(topic, String(on));
    }

    if (lastState == (uint32_t)-1)
      lastState = 0;
    else
      lastState &= ~((uint32_t)0x01);
    if (on)
      lastState |= (uint32_t)0x01;
    writeRTCmemory();
  }
}

inline void ESPWebMQTTRelay::toggleRelay() {
  switchRelay(digitalRead(relayPin) != relayLevel);
}

uint16_t ESPWebMQTTRelay::readSchedulesConfig(uint16_t offset) {
  if (offset) {
    Schedule::period_t period;
    int8_t hour;
    int8_t minute;
    int8_t second;
    uint8_t weekdays;
    int8_t day;
    int8_t month;
    int16_t year;

    for (int8_t i = 0; i < maxSchedules; ++i) {
      getEEPROM(offset, period);
      offset += sizeof(period);
      getEEPROM(offset, hour);
      offset += sizeof(hour);
      getEEPROM(offset, minute);
      offset += sizeof(minute);
      getEEPROM(offset, second);
      offset += sizeof(second);
      if (period == Schedule::WEEKLY) {
        getEEPROM(offset, weekdays);
        offset += sizeof(weekdays);
      } else {
        getEEPROM(offset, day);
        offset += sizeof(day);
        getEEPROM(offset, month);
        offset += sizeof(month);
        getEEPROM(offset, year);
        offset += sizeof(year);
      }
      getEEPROM(offset, scheduleTurns[i]);
      offset += sizeof(scheduleTurns[i]);

      if (period == Schedule::NONE)
        schedules[i].clear();
      else
        schedules[i].set(period, hour, minute, second, weekdays, day, month, year);
    }
  }

  return offset;
}

uint16_t ESPWebMQTTRelay::writeSchedulesConfig(uint16_t offset) {
  if (offset) {
    Schedule::period_t period;
    int8_t hour;
    int8_t minute;
    int8_t second;
    uint8_t weekdays;
    int8_t day;
    int8_t month;
    int16_t year;

    for (int8_t i = 0; i < maxSchedules; ++i) {
      period = schedules[i].period();
      hour = schedules[i].hour();
      minute = schedules[i].minute();
      second = schedules[i].second();
      if (period == Schedule::WEEKLY) {
        weekdays = schedules[i].weekdays();
      } else {
        day = schedules[i].day();
        month = schedules[i].month();
        year = schedules[i].year();
      }

      putEEPROM(offset, period);
      offset += sizeof(period);
      putEEPROM(offset, hour);
      offset += sizeof(hour);
      putEEPROM(offset, minute);
      offset += sizeof(minute);
      putEEPROM(offset, second);
      offset += sizeof(second);
      if (period == Schedule::WEEKLY) {
        putEEPROM(offset, weekdays);
        offset += sizeof(weekdays);
      } else {
        putEEPROM(offset, day);
        offset += sizeof(day);
        putEEPROM(offset, month);
        offset += sizeof(month);
        putEEPROM(offset, year);
        offset += sizeof(year);
      }
      putEEPROM(offset, scheduleTurns[i]);
      offset += sizeof(scheduleTurns[i]);
    }
  }

  return offset;
}

void ESPWebMQTTRelay::publishTemperature() {
  if (pubSubClient->connected()) {
    String topic;

    if (_mqttClient != strEmpty) {
      topic += charSlash;
      topic += _mqttClient;
    }
    topic += FPSTR(mqttTemperatureTopic);
    mqttPublish(topic, String(climateTemperature));
  }
}

void ESPWebMQTTRelay::publishHumidity() {
  if (pubSubClient->connected()) {
    String topic;

    if (_mqttClient != strEmpty) {
      topic += charSlash;
      topic += _mqttClient;
    }
    topic += FPSTR(mqttHumidityTopic);
    mqttPublish(topic, String(climateHumidity));
  }
}

void ESPWebMQTTRelay::pulseLed() {
  const uint8_t minBrightness = 255;
  const uint8_t halfBrightness = 127;
  const uint8_t maxBrightness = 0;
  const int8_t defDelta = -8;

  static int8_t delta = defDelta;
  static int16_t brightness = halfBrightness;
  uint8_t fromBrightness, toBrightness;

  if (digitalRead(relayPin) == relayLevel) { // Relay is on
    fromBrightness = halfBrightness;
    toBrightness = maxBrightness;
  } else {
    fromBrightness = minBrightness;
    toBrightness = halfBrightness;
  }

  analogWrite(ledPin, brightness);
  if (_pulse == PULSE) {
    if (delta != defDelta) {
      if (brightness == fromBrightness)
        brightness = toBrightness;
      else
        brightness = fromBrightness;
    }
    delta = -delta;
  } else if (_pulse == FASTPULSE) {
    if (brightness == fromBrightness)
      brightness = toBrightness;
    else
      brightness = fromBrightness;
  } else if (_pulse == BREATH) {
    brightness += delta;
    if ((brightness < toBrightness) || (brightness > fromBrightness)) {
      delta = -delta;
      brightness = constrain(brightness, toBrightness, fromBrightness);
    }
  } else if (_pulse == FADEIN) {
    brightness -= abs(delta);
    if (brightness < toBrightness) {
      brightness = fromBrightness;
    }
  } else if (_pulse == FADEOUT) {
    brightness -= abs(delta);
    if (brightness > fromBrightness) {
      brightness = toBrightness;
    }
  }
}

ESPWebMQTTRelay *app = new ESPWebMQTTRelay();


void setup() {
#ifndef NOSERIAL
  Serial.begin(115200, SERIAL_8N1, SERIAL_TX_ONLY);
  Serial.println();
#endif

  app->setup();
}

void loop() {
  app->loop();
}
