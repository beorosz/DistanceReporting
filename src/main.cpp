#include <Arduino.h>
#include <WiFi.h>

#include "secrets.h"
#include <AsyncMqttClient.h>

// Copy secrets
char wifi_ssid[] = WIFI_SSID;
char wifi_password[] = WIFI_PASSWORD;
char mqtt_user[] = MQTT_USER;
char mqtt_password[] = MQTT_PASSWORD;

const int trigPin = 14;
const int echoPin = 12;
const int ledPin = 21;
//define sound speed in cm/microSec (343 m/s)
#define SOUND_SPEED 0.0343

AsyncMqttClient mqttClient;

TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;
TimerHandle_t distanceMeasureReportTimer;

TaskHandle_t mqttReconnectTask = NULL;

/*    MQTT connection handlers    */
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

/*    Wifi connection handlers  */
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
    Serial.printf("WiFi connected, IP address: %s\n", WiFi.localIP());
    connectToMqtt();
    break;
  case SYSTEM_EVENT_STA_DISCONNECTED:
    Serial.println("WiFi lost connection");
    xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
    xTimerStart(wifiReconnectTimer, 0);
    break;
  }
}

/*    Distance measure and publication handler  */
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

/************************************/
/*             Setup                */
/************************************/

void setup()
{
  Serial.begin(115200);     // Starts the serial communication
  pinMode(trigPin, OUTPUT); // Sets the trigger pin as an Output
  pinMode(echoPin, INPUT);  // Sets the echo pin as an Input
  pinMode(ledPin, OUTPUT);  // Sets the LED pin as an Output

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));
  distanceMeasureReportTimer = xTimerCreate("distanceMeasureReportTimer", pdMS_TO_TICKS(10000), pdTRUE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(measureAndReportDistance));

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
  if (!mqttClient.connected())
  {
    digitalWrite(ledPin, HIGH);
    delay(1000);
    digitalWrite(ledPin, LOW);
    delay(1000);
  }
}