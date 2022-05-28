#include <Arduino.h>
#include <WiFi.h>

// Rename secrets.h.example to secrets.h and update the settings
#include "secrets.h"
#include <AsyncMqttClient.h>

#include "LittleFS.h"
#define FORMAT_LITTLEFS_IF_FAILED true
char gateOpenedStatusFileName[] = "/gatestatus.txt";

// Copy secrets
char wifi_ssid[] = WIFI_SSID;
char wifi_password[] = WIFI_PASSWORD;
char mqtt_user[] = MQTT_USER;
char mqtt_password[] = MQTT_PASSWORD;

const int trigPin = 12;
const int echoPin = 14;
const int ledPin = 21;
const int openClosePin = 23;

// Control flag for sending gate status once per change
bool hasGateStatusChanged;
// If MQTT connection is not available for 60000 milliseconds, restart the ESP
const long MS_SINCE_MQTT_DISCONNECTED_THRESHOLD = 60000;
long lastReportSendTime;

// Timer frequencies
const long DISTANCE_MEASURE_REPORT_TIMER_FREQUENCY = 10000;
const long RECONNECT_TIMER_FREQUENCY = 2000;

//define sound speed in cm/microSec (343 m/s)
#define SOUND_SPEED 0.0343

AsyncMqttClient mqttClient;

TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;
TimerHandle_t distanceMeasureReportTimer;

/************************************/
/*     Open-close event handlers    */
/************************************/
void IRAM_ATTR gateStatusChangedHandler()
{
  hasGateStatusChanged = true;
}

/************************************/
/*      MQTT event handlers         */
/************************************/
void onMqttConnect(bool sessionPresent)
{
  Serial.println("Connected to MQTT.");
  xTimerStart(distanceMeasureReportTimer, 0);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
  Serial.println("Disconnected from MQTT.");
  xTimerStop(distanceMeasureReportTimer, 0);
  if (WiFi.isConnected())
  {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void connectToMqtt()
{
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

/************************************/
/*      Wifi event handlers         */
/************************************/
void connectToWifi()
{
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(wifi_ssid, wifi_password);
}

void wiFiEventHandler(WiFiEvent_t event)
{
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch (event)
  {
  case SYSTEM_EVENT_STA_GOT_IP:
    Serial.println("WiFi connected");
    connectToMqtt();
    break;
  case SYSTEM_EVENT_STA_DISCONNECTED:
    Serial.println("WiFi lost connection");
    xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
    xTimerStart(wifiReconnectTimer, 0);
    break;
  default:
    break;
  }
}

/************************************************/
/*    Distance measure and publication handler  */
/************************************************/
void measureAndReportDistance()
{
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  long duration = pulseIn(echoPin, HIGH);
  // Calculate the distance
  float distanceInCm = duration * SOUND_SPEED / 2;

  // publish the distance on MQTT
  Serial.printf("Distance: %f cm\n", distanceInCm);

  size_t bufSize = snprintf(NULL, 0, "{ \"distanceInCm\": %3.2f }", distanceInCm);
  char *payload = (char *)malloc(bufSize + 1);
  sprintf(payload, "{ \"distanceInCm\": %3.2f }", distanceInCm);
  mqttClient.publish("garage/distance", 0, true, payload);
  free(payload);
}

/************************************************/
/*              File I/O handlers               */
/************************************************/

String readFileContent(const char *path)
{
  File file = LittleFS.open(path);
  if (!file || file.isDirectory())
  {
    Serial.println("- failed to open file for reading");
    return "";
  }
  String fileContent = file.readString();
  file.close();

  return fileContent;
}

bool writeFile(const char *path, const char *message)
{
  File file = LittleFS.open(path, FILE_WRITE);
  if (!file)
  {
    Serial.println("- failed to open file for writing");
    return false;
  }
  if (!file.print(message))
  {
    Serial.println("- write failed");
    return false;
  }

  file.close();

  return true;
}

/************************************/
/*             Setup                */
/************************************/
void setup()
{
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(openClosePin, INPUT_PULLUP);

  LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED);
  lastReportSendTime = millis();

  // Read the current open/close status
  String isGateOpened = digitalRead(openClosePin) > 0 ? "true" : "false";
  String storedGateOpenedStatusInFile = readFileContent(gateOpenedStatusFileName);
  Serial.printf("Stored gate opened status:%s\n", storedGateOpenedStatusInFile);
  if (storedGateOpenedStatusInFile != isGateOpened)
  {
    hasGateStatusChanged = true;
  }

  // Subscribe to pin value changes
  attachInterrupt(openClosePin, gateStatusChangedHandler, CHANGE);

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(RECONNECT_TIMER_FREQUENCY), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(RECONNECT_TIMER_FREQUENCY), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));
  distanceMeasureReportTimer = xTimerCreate("distanceMeasureReportTimer", pdMS_TO_TICKS(DISTANCE_MEASURE_REPORT_TIMER_FREQUENCY), pdTRUE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(measureAndReportDistance));

  WiFi.onEvent(wiFiEventHandler);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials(mqtt_user, mqtt_password);

  connectToWifi();
}

/************************************/
/*           Main loop              */
/************************************/
void loop()
{
  // Status LED blinks if MQTT is not connected
  if (!mqttClient.connected())
  {
    digitalWrite(ledPin, HIGH);
    delay(1000);
    digitalWrite(ledPin, LOW);
    delay(1000);

    if (millis() - lastReportSendTime > MS_SINCE_MQTT_DISCONNECTED_THRESHOLD)
    {
      ESP.restart();
    }
  }
  else
  {
    lastReportSendTime = millis();

    // Publish gate status change message
    if (hasGateStatusChanged)
    {
      String isGateOpened = digitalRead(openClosePin) > 0 ? "true" : "false";
      Serial.println(isGateOpened == "true" ? "Gate is opening." : "Gate is closed.");

      writeFile(gateOpenedStatusFileName, isGateOpened.c_str());

      size_t bufSize = snprintf(NULL, 0, "{ \"opened\": %s }", isGateOpened);
      char *payload = (char *)malloc(bufSize + 1);
      sprintf(payload, "{ \"opened\": %s }", isGateOpened);
      mqttClient.publish("garage/gateStatus", 0, true, payload);
      hasGateStatusChanged = false;
      free(payload);
    }

    delay(1000);
  }
}