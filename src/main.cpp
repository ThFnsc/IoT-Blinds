#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <env.h>

enum MotorState
{
  Off,
  UpSlow,
  DownSlow,
  UpFast,
  DownFast
};

enum BlindsPosition
{
  Unknown,
  FullyClosed,
  Extended,
  Middle,
  AlmostOpen,
  FullyOpen
};

class Sensor
{
private:
  int pin;
  unsigned long debounceDelay;
  unsigned long willDebounce;
  bool lastButtonState;
  bool buttonState;

public:
  Sensor() {}
  Sensor(int pin, unsigned long debounceDelay = 10)
  {
    this->pin = pin;
    this->debounceDelay = debounceDelay;
    pinMode(pin, INPUT_PULLUP);
    willDebounce = 0;
    lastButtonState = buttonState = !digitalRead(pin);
  }

  // Call this function in your loop to check for button state changes
  bool changed()
  {
    bool reading = !digitalRead(pin);

    if (reading != lastButtonState)
      willDebounce = millis() + debounceDelay;

    if (reading != buttonState && millis() > willDebounce)
    {
      buttonState = reading;
      Serial.printf("[%lu] Button %i changed to %i\n", millis(), pin, buttonState);
      return true;
    }

    lastButtonState = reading;
    return false;
  }

  bool getChangedState()
  {
    return buttonState;
  }

  bool getActualState()
  {
    return buttonState = lastButtonState = !digitalRead(pin);
  }
};

class Timer
{
private:
  unsigned long timeoutMs;
  unsigned long intervalMs;

public:
  Timer() {}
  Timer(unsigned long intervalMs, bool beginAtStartup)
  {
    this->intervalMs = intervalMs;
    this->timeoutMs = beginAtStartup;
  }

  void restart()
  {
    timeoutMs = millis() + intervalMs;
  }

  void restart(unsigned long newInterval)
  {
    intervalMs = newInterval;
    restart();
  }

  void stop()
  {
    timeoutMs = 0;
  }

  bool elapsed()
  {
    return timeoutMs && millis() >= timeoutMs;
  }
};

class Motor
{
private:
  byte pinUp;
  byte pinDown;
  byte pinAccelerate;
  byte pinEnable;
  MotorState currentState;
  Timer *applyDelay, *timeout;

  void setOutputs(bool on, bool up, bool fast)
  {
    digitalWrite(pinUp, on && up);
    digitalWrite(pinDown, on && !up);
    digitalWrite(pinAccelerate, on && fast);
    digitalWrite(pinEnable, !on);
  }

  void setInternal(MotorState state)
  {
    this->currentState = state;
    Serial.printf("[%lu] Motor changed to %i\n", millis(), currentState);
    switch (state)
    {
    case UpFast:
      setOutputs(true, true, true);
      break;
    case UpSlow:
      setOutputs(true, true, false);
      break;
    case DownFast:
      setOutputs(true, false, true);
      break;
    case DownSlow:
      setOutputs(true, false, false);
      break;
    case Off:
    default:
      setOutputs(false, false, false);
      break;
    }
  }

public:
  Motor() {}
  Motor(byte pinUp, byte pinDown, byte pinAccelerate, byte pinEnable)
  {
    pinMode(pinUp, OUTPUT);
    pinMode(pinDown, OUTPUT);
    pinMode(pinAccelerate, OUTPUT);
    pinMode(pinEnable, OUTPUT);
    setOutputs(false, false, false);
    this->pinUp = pinUp;
    this->pinDown = pinDown;
    this->pinAccelerate = pinAccelerate;
    this->pinEnable = pinEnable;
    this->currentState = Off;
    this->applyDelay = new Timer(100, false);
    this->timeout = new Timer(0, false);
  }

  void loop()
  {
    if (timeout->elapsed())
    {
      setInternal(Off);
      timeout->stop();
    }
    else if (applyDelay->elapsed())
    {
      setInternal(currentState);
      applyDelay->stop();
    }
  }

  void set(MotorState state, unsigned long timeoutMs = 0)
  {
    MotorState previousState = currentState;
    currentState = state;

    if (state == Off)
      setInternal(state);
    else if (timeoutMs)
      timeout->restart(timeoutMs);
    else
      timeout->stop();

    if (previousState == state)
      setInternal(state);
    else
    {
      setInternal(Off);
      currentState = state;
      applyDelay->restart();
    }
  }

  MotorState get()
  {
    return currentState;
  }
};

Sensor topSensor(D2), bottomSensor(D1);
Motor motor(D8, D6, D7, D4);

#define EXTENDED_TO_FULLY_CLOSED 3100
#define ALMOST_OPEN_TO_FULLY_OPEN 4000
#define FULLY_OPEN_TO_ALMOST_OPEN 1000
#define ALMOST_OPEN_TO_EXTENDED 10000
#define EXTENDED_TO_AMOST_OPEN 4500
#define FULLY_CLOSED_TO_EXTENDED 3000

class Blinds
{
private:
  Motor *motor;
  Sensor *topSensor, *bottomSensor;
  Timer *timeout;
  bool initialized;
  BlindsPosition currentPosition, desiredPosition;

  void initializeLoop()
  {
    if (currentPosition == Unknown)
    {
      if (topSensor->getActualState())
      {
        currentPosition = FullyOpen;
        initialized = true;
      }
      else if (bottomSensor->getActualState())
      {
        currentPosition = Extended;
        motor->set(UpFast, FULLY_CLOSED_TO_EXTENDED);
      }
      else
      {
        currentPosition = Middle;
        motor->set(DownSlow, ALMOST_OPEN_TO_EXTENDED);
      }
    }
    else
    {
      switch (motor->get())
      {
      case UpFast:
        if (bottomSensor->changed() && !bottomSensor->getChangedState())
          motor->set(DownSlow, EXTENDED_TO_FULLY_CLOSED);

        if (topSensor->getActualState())
        {
          Serial.printf("Top sensor activated when it should not");
          motor->set(Off);
        }
        break;
      case DownSlow:
        if (bottomSensor->changed() && bottomSensor->getActualState())
          motor->set(DownSlow, EXTENDED_TO_FULLY_CLOSED);
        break;
      case Off:
        currentPosition = FullyClosed;
        initialized = true;
        break;
      default:
        break;
      }
    }
  }

public:
  Blinds(Motor &motor, Sensor &topSensor, Sensor &bottomSensor)
  {
    this->motor = &motor;
    this->topSensor = &topSensor;
    this->bottomSensor = &bottomSensor;
    this->timeout = new Timer(100, true);
    initialized = false;
    currentPosition = Unknown;
  }

  void loop()
  {
    motor->loop();
    if (!initialized)
    {
      initializeLoop();
      return;
    }

    if (currentPosition == desiredPosition)
      return;

    switch (motor->get())
    {
    case Off:
      if (currentPosition == Extended && desiredPosition == FullyClosed)
        currentPosition = FullyClosed;
      else if (currentPosition == Middle && desiredPosition == FullyOpen)
      {
        currentPosition = AlmostOpen;
        motor->set(UpSlow, ALMOST_OPEN_TO_FULLY_OPEN);
      }
      else if (currentPosition == AlmostOpen && desiredPosition == FullyClosed)
      {
        currentPosition = Middle;
        motor->set(DownSlow, ALMOST_OPEN_TO_EXTENDED);
      }
    case DownSlow:
    case DownFast:
      if (bottomSensor->changed() && bottomSensor->getChangedState())
      {
        currentPosition = Extended;
        motor->set(DownSlow, EXTENDED_TO_FULLY_CLOSED);
      }
      break;
    case UpFast:
    case UpSlow:
      if (bottomSensor->changed() && !bottomSensor->getChangedState())
      {
        currentPosition = Middle;
        motor->set(UpFast, EXTENDED_TO_AMOST_OPEN);
      }
      else if (topSensor->getActualState())
      {
        currentPosition = FullyOpen;
        motor->set(Off);
      }
    }
  }

  void set(BlindsPosition position)
  {
    if (!initialized)
      return;

    if (currentPosition == position)
      return;

    desiredPosition = position;

    switch (position)
    {
    case FullyOpen:
      if (currentPosition == AlmostOpen)
        motor->set(UpSlow, ALMOST_OPEN_TO_FULLY_OPEN);
      else if (currentPosition == FullyClosed)
        motor->set(UpFast, FULLY_CLOSED_TO_EXTENDED);
      else
        motor->set(UpFast, EXTENDED_TO_AMOST_OPEN);
      break;
    case FullyClosed:
      if (bottomSensor->getActualState())
        motor->set(DownSlow, EXTENDED_TO_FULLY_CLOSED);
      else if (currentPosition == FullyOpen)
      {
        currentPosition = AlmostOpen;
        motor->set(DownSlow, FULLY_OPEN_TO_ALMOST_OPEN);
      }
      else
        motor->set(DownSlow, ALMOST_OPEN_TO_EXTENDED);
      break;
    default:
      break;
    }
  }

  void stop()
  {
    motor->set(Off);
    desiredPosition = currentPosition;
  }
};

Blinds blinds(motor, topSensor, bottomSensor);

void callback(char *topic, byte *payload, unsigned int length)
{
  char body[length + 1];
  for (unsigned int i = 0; i < length; i++)
    body[i] = payload[i];
  body[length] = 0;

  Serial.printf("MQTT [%s]=[%s]\n", topic, body);

  if (strcmp(topic, TOPIC_IN)) // NOT the input topic
  {
    Serial.println("Unknown topic");
    return;
  }

  if (strcmp(body, "OPEN") == 0)
    blinds.set(FullyOpen);
  else if (strcmp(body, "CLOSE") == 0)
    blinds.set(FullyClosed);
  else if (strcmp(body, "STOP") == 0)
    blinds.stop();
  else
    Serial.println("Command not recognized");
}

WiFiClient espClient;
PubSubClient mqtt(SVRIP, PORT, callback, espClient);

void publishState(const char *message)
{
  mqtt.publish(TOPIC_OUT, message, true);
}

void subscribe(const char *topic)
{
  if (mqtt.subscribe(topic))
    Serial.printf("Subscribed to %s\n", topic);
  else
    Serial.printf("Failed to subscribe to %s\n", topic);
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Initialized");
  Serial.printf("Connecting to %s", WIFI_SSID);
  WiFi.persistent(true);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWD);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.printf(" Connected.\n");

  ArduinoOTA.setHostname("Blinds-1");

  ArduinoOTA.onStart([]()
                     { Serial.println("Start"); });
  ArduinoOTA.onEnd([]()
                   { Serial.println("\nEnd"); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                        { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); });
  ArduinoOTA.onError([](ota_error_t error)
                     {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed"); });
  ArduinoOTA.begin();
}

Timer mqttReconnectTimer(0, true);
void mqttLoop()
{
  if (mqtt.connected())
  {
    mqtt.loop();
    return;
  }

  if (!mqttReconnectTimer.elapsed())
    return;

  Serial.println("Connecting to MQTT server...");

  if (mqtt.connect(MQTT_NAME, MQTT_USER, MQTT_PASSWD))
  {
    Serial.println("Success");
    subscribe(TOPIC_IN);
  }
  else
    Serial.println("Failed. Trying again later");

  mqttReconnectTimer.restart(1000);
}

void loop()
{
  ArduinoOTA.handle();
  mqttLoop();
  blinds.loop();
  if (Serial.available() > 0)
    ESP.restart();
}
