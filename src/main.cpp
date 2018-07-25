#include <pgmspace.h>
#include "ESPWebMQTT.h"
#include "Events.h"
#include "Button.h"


#include "RTCmem.h"
#include "DS1820.h"
#include "DHT.h"
#include "max6675.h"


//const int8_t maxSchedules = 10; // Количество элементов расписания

const char overSSID[] PROGMEM = "SONOFF_"; // Префикс имени точки доступа по умолчанию
const char overMQTTClient[] PROGMEM = "SONOFF_"; // Префикс имени MQTT-клиента по умолчанию

const uint8_t relayPin[2] = {1,2};
//const uint8_t relayPin2 = 2;
const bool relayLevel = HIGH; // Уровень срабатывания реле

const bool defRelayOnBoot = false; // Состояние реле при старте модуля по умолчанию
const uint16_t defRelayAutoOff = 0; // Время в секундах до автоотключения реле по умолчанию (0 - нет автоотключения)
const uint16_t defRelayDblClkAutoOff = 60; // Время в секундах до автоотключения реле при двойном нажатии на кнопку по умолчанию
const float defTemperatureTolerance = 0.2; // Порог изменения температуры
const float defAlarmTolerance = 1.0; // Порог изменения влажности

const uint8_t climatePin = 1; // Пин, к которому подключен датчик температуры/влажности


// Пины, к котором подключен датчик max6675
const uint8_t MAX_MISO = 12;
const uint8_t  MAX_CS = 15;
const uint8_t MAXS_CLK = 14;

const char pathRelay[] PROGMEM = "/relay"; // Путь до страницы настройки параметров реле
const char pathControl[] PROGMEM = "/control"; // Путь до страницы настройки параметров кнопок/ДУ
const char pathSwitch[] PROGMEM = "/switch"; // Путь до страницы управления переключением реле
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
const char paramClimateAlaTolerance[] PROGMEM = "climatehumtolerance";
const char paramClimateMinTemp[] PROGMEM = "climatemintemp";
const char paramClimateMaxTemp[] PROGMEM = "climatemaxtemp";
const char paramClimateMinTempTurn[] PROGMEM = "climatemintempturn";
const char paramClimateMaxTempTurn[] PROGMEM = "climatemaxtempturn";
const char paramClimateMinAla[] PROGMEM = "climateminhum";
const char paramClimateMaxAla[] PROGMEM = "climatemaxhum";
const char paramClimateMinAlaTurn[] PROGMEM = "climateminhumturn";
const char paramClimateMaxAlaTurn[] PROGMEM = "climatemaxhumturn";

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
const char jsonHumidity[] PROGMEM = "Alarm";

// Названия топиков для MQTT
const char mqttRelayTopic[] PROGMEM = "/Relay";
const char mqttTemperatureTopic[] PROGMEM = "/Temperature";
const char mqttAlarmTopic[] PROGMEM = "/Alarm";

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
  void handleClimateConfig(); // Обработчик страницы настройки параметров датчика температуры

  String navigator();
  String btnRelayConfig(); // HTML-код кнопки вызова настройки реле
  String btnClimateConfig(); // HTML-код кнопки вызова настройки датчика температуры

  void mqttCallback(char* topic, byte* payload, unsigned int length);
  void mqttResubscribe();

  void pulseLed();

private:
  void switchRelay(bool on, uint16_t customAutoOff = 0); // Процедура включения/выключения реле
  void switchRelay(int8_t port, bool on);
    void toggleRelay(int8_t port);// Процедура переключения реле

 // uint16_t readSchedulesConfig(uint16_t offset); // Чтение из EEPROM порции параметров расписания
  //uint16_t writeSchedulesConfig(uint16_t offset); // Запись в EEPROM порции параметров расписания

  void publishTemperature(); // Публикация температуры в MQTT
  void publishAlarm(); // Публикация влажности в MQTT

  struct relay_t {
    bool relayOnBoot; // Состояние реле при старте модуля
    uint16_t relayAutoOff; // Значения задержки реле в секундах до автоотключения (0 - нет автоотключения)
    uint16_t relayDblClkAutoOff; // Значения задержки реле в секундах до автоотключения при двойном нажатии на кнопку
    char relayName[RELAY_NAME_SIZE]; // Название реле
  uint32_t autoOff; // Значения в миллисекундах для сравнения с millis(), когда реле должно отключиться автоматически (0 - нет автоотключения)
  uint32_t lastState; // Битовое поле состояния реле для воостановления после перезагрузки

  } relay[2];

  Events *events;
  Button *button;

  enum turn_t : uint8_t { TURN_OFF, TURN_ON, TURN_TOGGLE };

  
  //turn_t scheduleTurns[maxSchedules]; // Что делать с реле по срабатыванию события

  enum sensor_t : uint8_t { SENSOR_NONE, SENSOR_MAX6675, SENSOR_DS1820, SENSOR_DHT11, SENSOR_DHT21, SENSOR_DHT22 };

  uint32_t climateReadTime; // Время в миллисекундах, после которого можно считывать новое значение сенсоров
  float climateTempTolerance; // Порог изменения температуры
  float climateAlaTolerance; // Порог изменения влажности
  float climateTemperature; // Значение успешно прочитанной температуры
  float climateAlarm; // Значение успешно прочитанной влажности
  float climateMinTemp, climateMaxTemp; // Минимальное и максимальное значение температуры срабатывания реле
  turn_t climateMinTempTurn, climateMaxTempTurn; // Что делать с реле по срабатыванию события
  float climateMinAla, climateMaxAla; // Минимальное и максимальное значение влажности срабатывания реле
  turn_t climateMinAlaTurn, climateMaxAlaTurn; // Что делать с реле по срабатыванию события
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
for (uint8_t i=0;i<2;i++)
{
  relay[i].autoOff = 0;

  if (relay[i].lastState != (uint32_t)-1) {
    if (relay[i].lastState & 0x01) {
      digitalWrite(relayPin[i], relayLevel);
      if (relay[i].relayAutoOff)
        relay[i].autoOff = millis() + relay[i].relayAutoOff * 1000;
    } else
      digitalWrite(relayPin[i], ! relayLevel);
  } else
    digitalWrite(relayPin[i], relay[i].relayOnBoot == relayLevel);
  pinMode(relayPin[i], OUTPUT);
}
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
  climateAlarm = NAN;
  climateMinHumTriggered = false;
  climateMaxHumTriggered = false;
}

void ESPWebMQTTRelay::loopExtra() {
  ESPWebMQTTBase::loopExtra();
for( uint8_t i=0;i<2;i++)
{
  if (relay[i].autoOff && ((int32_t)millis() >= (int32_t)relay[i].autoOff)) {
    switchRelay(relayPin[i], false);
    relay[i].autoOff = 0;
  }
}
  uint32_t now = getTime();

 /* while (Events::event_t *evt = events->getEvent()) {
    if (evt->type == Events::EVT_BTNCLICK) {
      toggleRelay(relayPin[1]);
      logDateTime(now);
      _log->println(F(" button pressed"));
    } else if (evt->type == Events::EVT_BTNDBLCLICK) {
      if (relay[1].relayDblClkAutoOff)
        switchRelay(true, relay[1].relayDblClkAutoOff);
      else
        toggleRelay(relayPin[1]);
      logDateTime(now);
      _log->println(F(" button double pressed"));
    } else if (evt->type == Events::EVT_BTNLONGPRESSED) {
      enablePulse(FASTPULSE);
    } else if (evt->type == Events::EVT_BTNLONGCLICK) {
      clearEEPROM();
      reboot();
    }
  }*/

  if (climateSensor != SENSOR_NONE) {
    {
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
                      toggleRelay(relayPin[0]);
                    else
                      switchRelay(relayPin[0], climateMinTempTurn == TURN_ON);
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
                      toggleRelay(relayPin[0]);
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
///////////////////////////
//          v = dht->readHumidity();
          if (! isnan(v) && (v >= 0.0) && (v <= 100.0)) {
            if (isnan(climateAlarm) || (abs(climateAlarm - v) > climateAlaTolerance)) {
              climateAlarm = v;
              //publishAlarm();
              if (! isnan(climateMinAla)) {
                if (climateAlarm < climateMinAla) {
                  if (! climateMinHumTriggered) {
                    if (climateMinAlaTurn == TURN_TOGGLE)
                      toggleRelay(relayPin[1]);
                    else
                      switchRelay(relayPin[1],climateMinAlaTurn == TURN_ON);
                    logDateTime(now);
                    _log->println(F(" DHTx minimal humidity triggered"));
                    climateMinHumTriggered = true;
                  }
                } else
                  climateMinHumTriggered = false;
              }
              if (! isnan(climateMaxAla)) {
                if (climateAlarm > climateMaxAla) {
                  if (! climateMaxHumTriggered) {
                    if (climateMaxAlaTurn == TURN_TOGGLE)
                      toggleRelay(relayPin[1]);
                    else
                      switchRelay(relayPin[1],climateMaxAlaTurn == TURN_ON);
                    logDateTime(now);
                    _log->println(F(" DHTx maximal humidity triggered"));
                    climateMaxHumTriggered = true;
                  }
                } else
                  climateMaxHumTriggered = false;
              }
            }
          }
//////////////////////          
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
                      toggleRelay(relayPin[0]);
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
                      toggleRelay(relayPin[0]);
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
            if (isnan(climateAlarm) || (abs(climateAlarm - v) > climateAlaTolerance)) {
              climateAlarm = v;
              publishAlarm();
              if (! isnan(climateMinAla)) {
                if (climateAlarm < climateMinAla) {
                  if (! climateMinHumTriggered) {
                    if (climateMinAlaTurn == TURN_TOGGLE)
                      toggleRelay(relayPin[1]);
                    else
                      switchRelay(relayPin[1],climateMinAlaTurn == TURN_ON);
                    logDateTime(now);
                    _log->println(F(" DHTx minimal humidity triggered"));
                    climateMinHumTriggered = true;
                  }
                } else
                  climateMinHumTriggered = false;
              }
              if (! isnan(climateMaxAla)) {
                if (climateAlarm > climateMaxAla) {
                  if (! climateMaxHumTriggered) {
                    if (climateMaxAlaTurn == TURN_TOGGLE)
                      toggleRelay(relayPin[1]);
                    else
                      switchRelay(relayPin[1],climateMaxAlaTurn == TURN_ON);
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
for(uint8_t i=0;i<2;i++){
    getEEPROM(offset, relay[i]);
    offset += sizeof(relay[i]);
}
    //offset = readSchedulesConfig(offset);

    sensor_t _climateSensor; // Bit-field workaround

    getEEPROM(offset, _climateSensor);
    offset += sizeof(_climateSensor);
    climateSensor = _climateSensor;

    getEEPROM(offset, climateTempTolerance);
    offset += sizeof(climateTempTolerance);
    getEEPROM(offset, climateAlaTolerance);
    offset += sizeof(climateAlaTolerance);
    getEEPROM(offset, climateMinTemp);
    offset += sizeof(climateMinTemp);
    getEEPROM(offset, climateMaxTemp);
    offset += sizeof(climateMaxTemp);
    getEEPROM(offset, climateMinTempTurn);
    offset += sizeof(climateMinTempTurn);
    getEEPROM(offset, climateMaxTempTurn);
    offset += sizeof(climateMaxTempTurn);
    getEEPROM(offset, climateMinAla);
    offset += sizeof(climateMinAla);
    getEEPROM(offset, climateMaxAla);
    offset += sizeof(climateMaxAla);
    getEEPROM(offset, climateMinAlaTurn);
    offset += sizeof(climateMinAlaTurn);
    getEEPROM(offset, climateMaxAlaTurn);
    offset += sizeof(climateMaxAlaTurn);

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
for(uint8_t i=0;i<2;i++){
  putEEPROM(offset,  relay[i]);
  offset += sizeof( relay[i]);
}
  //offset = writeSchedulesConfig(offset);

  sensor_t _climateSensor = climateSensor; // Bit-field workaround

  putEEPROM(offset, _climateSensor);
  offset += sizeof(_climateSensor);

  putEEPROM(offset, climateTempTolerance);
  offset += sizeof(climateTempTolerance);
  putEEPROM(offset, climateAlaTolerance);
  offset += sizeof(climateAlaTolerance);
  putEEPROM(offset, climateMinTemp);
  offset += sizeof(climateMinTemp);
  putEEPROM(offset, climateMaxTemp);
  offset += sizeof(climateMaxTemp);
  putEEPROM(offset, climateMinTempTurn);
  offset += sizeof(climateMinTempTurn);
  putEEPROM(offset, climateMaxTempTurn);
  offset += sizeof(climateMaxTempTurn);
  putEEPROM(offset, climateMinAla);
  offset += sizeof(climateMinAla);
  putEEPROM(offset, climateMaxAla);
  offset += sizeof(climateMaxAla);
  putEEPROM(offset, climateMinAlaTurn);
  offset += sizeof(climateMinAlaTurn);
  putEEPROM(offset, climateMaxAlaTurn);
  offset += sizeof(climateMaxAlaTurn);

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
   for(uint8_t i=0;i<2;i++){   
     relay[i].relayOnBoot = defRelayOnBoot;
     relay[i].relayAutoOff = defRelayAutoOff;
     relay[i].relayDblClkAutoOff = defRelayDblClkAutoOff;
    memset( relay[i].relayName, 0, sizeof( relay[i].relayName));
   }


    climateSensor = SENSOR_NONE;
    climateTempTolerance = defTemperatureTolerance;
    climateAlaTolerance = defAlarmTolerance;
    climateMinTemp = NAN;
    climateMaxTemp = NAN;
    climateMinTempTurn = TURN_OFF;
    climateMaxTempTurn = TURN_OFF;
    climateMinAla = NAN;
    climateMaxAla = NAN;
    climateMinAlaTurn = TURN_OFF;
    climateMaxAlaTurn = TURN_OFF;
  }
}

bool ESPWebMQTTRelay::setConfigParam(const String &name, const String &value) {
  if (! ESPWebMQTTBase::setConfigParam(name, value)) {
 if (name.equals(FPSTR(paramClimateSensor))) {
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
    } else if (name.equals(FPSTR(paramClimateAlaTolerance))) {
      climateAlaTolerance = _max(0, value.toFloat());
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
    } else if (name.equals(FPSTR(paramClimateMinAla))) {
      if (value.length())
        climateMinAla = value.toFloat();
      else
        climateMinAla = NAN;
    } else if (name.equals(FPSTR(paramClimateMaxAla))) {
      if (value.length())
        climateMaxAla = value.toFloat();
      else
        climateMaxAla = NAN;
    } else if (name.equals(FPSTR(paramClimateMinAlaTurn))) {
      climateMinAlaTurn = (turn_t)value.toInt();
    } else if (name.equals(FPSTR(paramClimateMaxAlaTurn))) {
      climateMaxAlaTurn = (turn_t)value.toInt();
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
  for(uint8_t i=0;i<2;i++)
  {
  page += F("</p>\n");
  page += F("<input type=\"checkbox\" class=\"checkbox\" id=\"");
  page += FPSTR(jsonRelay);
  page += F("\" onchange=\"switchRelay(this.checked)\" ");
  if (digitalRead(relayPin[i]) == relayLevel)
    page += FPSTR(extraChecked);
  page += F(">\n\
<label for=\"");
  page += FPSTR(jsonRelay);
  page += F("\">");
  if (relay[i].relayName[0])
    page += charBufToString(relay[i].relayName, sizeof(relay[i].relayName));
  else
    page += F("Relay");
  page += F("</label>\n\
<input type=\"text\" id=\"");
  page += FPSTR(jsonRelayAutoOff);
  page += F("\" value=\"");
  page += String(relay[i].relayAutoOff);
  page += F("\" size=5 maxlength=5 />\n\
sec. to auto-off\n\
<p>\n");
  }
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
      result += isnan(climateAlarm) ? F("\"?\"") : String(climateAlarm);
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
  page += FPSTR(paramClimateAlaTolerance);
  page += F("\" value=\"");
  page += String(climateAlaTolerance);
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
  page += FPSTR(paramClimateMinAla);
  page += F("\" value=\"");
  if (! isnan(climateMinAla))
    page += String(climateMinAla);
  page += F("\" size=10 maxlength=10>\n\
(leave blank if not used)<br/>\n\
<label>Turn relay</label>\n\
<input type=\"radio\" name=\"");
  page += FPSTR(paramClimateMinAlaTurn);
  page += F("\" value=\"0\"");
  if (climateMinAlaTurn == TURN_OFF)
    page += F(" checked");
  page += F(">OFF\n\
<input type=\"radio\" name=\"");
  page += FPSTR(paramClimateMinAlaTurn);
  page += F("\" value=\"1\"");
  if (climateMinAlaTurn == TURN_ON)
    page += F(" checked");
  page += F(">ON\n\
<input type=\"radio\" name=\"");
  page += FPSTR(paramClimateMinAlaTurn);
  page += F("\" value=\"2\"");
  if (climateMinAlaTurn == TURN_TOGGLE)
    page += F(" checked");
  page += F(">TOGGLE<br/>\n\
<label>Maximal humidity:</label><br/>\n\
<input type=\"text\" name=\"");
  page += FPSTR(paramClimateMaxAla);
  page += F("\" value=\"");
  if (! isnan(climateMaxAla))
    page += String(climateMaxAla);
  page += F("\" size=10 maxlength=10>\n\
(leave blank if not used)<br/>\n\
<label>Turn relay</label>\n\
<input type=\"radio\" name=\"");
  page += FPSTR(paramClimateMaxAlaTurn);
  page += F("\" value=\"0\"");
  if (climateMaxAlaTurn == TURN_OFF)
    page += F(" checked");
  page += F(">OFF\n\
<input type=\"radio\" name=\"");
  page += FPSTR(paramClimateMaxAlaTurn);
  page += F("\" value=\"1\"");
  if (climateMaxAlaTurn == TURN_ON)
    page += F(" checked");
  page += F(">ON\n\
<input type=\"radio\" name=\"");
  page += FPSTR(paramClimateMaxAlaTurn);
  page += F("\" value=\"2\"");
  if (climateMaxAlaTurn == TURN_TOGGLE)
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
  //result += btnSchedulesConfig();
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



inline void ESPWebMQTTRelay::toggleRelay(uint8_t port) {
  switchRelay(port,digitalRead(port) != relayLevel);
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

void ESPWebMQTTRelay::publishAlarm() {
  if (pubSubClient->connected()) {
    String topic;

    if (_mqttClient != strEmpty) {
      topic += charSlash;
      topic += _mqttClient;
    }
    topic += FPSTR(mqttHumidityTopic);
    mqttPublish(topic, String(climateAlarm));
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
