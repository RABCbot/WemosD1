#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <RunningMedian.h>

// Uncomment to enable 
//#define USE_SERIAL 1
//#define USE_ANALOG 1
//#define USE_MOTION 1
//#define USE_DHT 1
#define USE_DISTANCE 1
#define USE_RELAY 1

const char* FIRMWARE = "RABCBot 2017-09-10 10AM";
const int TIME_DELAY = 3500;
const char* ssid = "your-ssid";
const char* password = "your-ssid-password";

const char* mqttBroker = "your-mqtt-host";
const int mqttPort = 1883;
String mqttUser;

const String topicPrefix = "home/";
String topicFirmware = "/firmware";
String topicFirmwareGet = "/firmware/get"; // Return firmware
String topicTemp = "/temperature";
String topicHumid = "/humidity";
String topicDist = "/distance";
String topicDistMedian = "/distanceMedian";
String topicMotion = "/motion";
String topicAnalog = "/analog";
String topicRelay = "/relay"; // Returns status of relay
String topicRelaySet = "/relay/set"; // 1 is on, 0 is off
String topicForce = "home/status"; // Force publish status

const int pinRelay = D1;
const int pinDht = D4;
const int pinTrig = D5;
const int pinEcho = D6;
const int pinMotion = D7;
const int pinAnalog = A0;

#if defined(USE_DHT)
DHT dht(pinDht, DHT11);
#endif

WiFiClient clientWifi;
PubSubClient clientMqtt(clientWifi);

RunningMedian distMedian = RunningMedian(5);

void setup_wifi()
{
  #if defined(USE_SERIAL)
  Serial.println();
  Serial.print("Connecting to ");
  Serial.print(ssid);
  #endif

  delay(10);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    #if defined(USE_SERIAL)
    Serial.print(".");
    #endif
  }

  randomSeed(micros());

  #if defined(USE_SERIAL)
  Serial.println("connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("");
  #endif
}

void setup_mqtt()
{
  clientMqtt.setServer(mqttBroker, mqttPort);
  clientMqtt.setCallback(callback);
  mqttUser = "ESP_" + String(ESP.getChipId(), HEX);
  mqttUser.toUpperCase();
  topicFirmware = topicPrefix + mqttUser + topicFirmware;
  topicFirmwareGet = topicPrefix + mqttUser + topicFirmwareGet;
  topicTemp = topicPrefix + mqttUser + topicTemp;
  topicHumid = topicPrefix + mqttUser + topicHumid;
  topicDist = topicPrefix + mqttUser + topicDist;
  topicDistMedian = topicPrefix + mqttUser + topicDistMedian;
  topicMotion = topicPrefix + mqttUser + topicMotion;
  topicAnalog = topicPrefix + mqttUser + topicAnalog;
  topicRelaySet = topicPrefix + mqttUser + topicRelaySet;
  topicRelay = topicPrefix + mqttUser + topicRelay;
  reconnect();
}

void callback(char* topic, byte* payload, unsigned int length)
{
  #if defined(USE_SERIAL)
  Serial.print("Mqtt message received [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
    Serial.print((char)payload[i]);
  Serial.println();
  #endif

  if (String(topic) == topicRelaySet)
  {
    if ((char)payload[0] == '1') 
      digitalWrite(pinRelay, HIGH);
    else
      digitalWrite(pinRelay, LOW);
    PublishRelay(true);
  }
  if (String(topic) == topicForce)
  {
    PublishAll(true);    
  }
  if (String(topic) == topicFirmwareGet)
  {
    PublishFirmware();    
  }
}

void reconnect()
{
  // Loop until we're reconnected
  while (!clientMqtt.connected()) 
  {
    #if defined(USE_SERIAL)
      Serial.print(mqttUser);
      Serial.print(" Connecting to MQTT Broker...");
    #endif
    // Attempt to connect
    if (clientMqtt.connect(mqttUser.c_str())) 
    {
      #if defined(USE_SERIAL)
        Serial.println("connected");
        Serial.println("");
      #endif
      // re-subscribe
      clientMqtt.subscribe(topicForce.c_str());
      clientMqtt.subscribe(topicFirmwareGet.c_str());
      #if defined(USE_RELAY)
      clientMqtt.subscribe(topicRelaySet.c_str());
      #endif
    }
    else
    {
      #if defined(USE_SERIAL)
      Serial.print("failed because ");
      Serial.print(clientMqtt.state());
      Serial.println(" try again in 5 seconds");
      #endif
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup_pins()
{
  #if defined(USE_RELAY)
  pinMode(pinRelay, OUTPUT);
  #endif
  #if defined(USE_DISTANCE)
  pinMode(pinTrig, OUTPUT);
  pinMode(pinEcho, INPUT);
  #endif
  #if defined(USE_MOTION)
  pinMode(pinMotion, INPUT);
  #endif  
}

void setup_dht()
{
  #if defined(USE_DHT)
  dht.begin();
  #endif
}

void setup()
{
  #if defined(USE_SERIAL)
  Serial.begin(115200);
  #endif
  setup_wifi();
  setup_mqtt();
  setup_dht();
  setup_pins();
  PublishAll(true);
}

long prevTime = 0;

void loop()
{
  if (!clientMqtt.connected())
  {
    reconnect();
  }
  clientMqtt.loop();

  long now = millis();
  if (now - prevTime > TIME_DELAY)
  {
    prevTime = now;

    PublishAll(false);
  }
}

float prevDist, prevDistMedian;
void PublishDistance(boolean forced)
{
  float val, dist;
  String aux;

  #if defined(USE_DISTANCE)
    digitalWrite(pinTrig, HIGH);
    delayMicroseconds(10);
    digitalWrite(pinTrig, LOW);
    val = pulseIn(pinEcho, HIGH);
    dist = val / 160;
    #if defined(USE_SERIAL)
      Serial.print("Distance measured ");
      Serial.println(dist);
    #endif
    
    // Publish distance median
    aux = String(dist, 1);
    if(prevDist != dist || forced)
    {
      clientMqtt.publish(topicDist.c_str(), aux.c_str());
      prevDist = dist;
      #if defined(USE_SERIAL)
      Serial.print(topicDist);
      Serial.println(" MQTT Published");
      #endif
    }

    // Add value to running median
    distMedian.add(dist);
    // pull median
    dist = distMedian.getMedian();

    #if defined(USE_SERIAL)
      Serial.print("Distance median ");
      Serial.println(distMedian.getMedian());
    #endif

    // Publish distance median
    aux = String(dist, 1);
    if(prevDistMedian != dist || forced)
    {
      clientMqtt.publish(topicDistMedian.c_str(), aux.c_str());
      prevDistMedian = dist;
      #if defined(USE_SERIAL)
      Serial.print(topicDistMedian);
      Serial.println(" MQTT Published");
      #endif
    }
  #endif
}

float prevAnalog;
void PublishAnalog(boolean forced)
{
  float val;
  String aux;

  #if defined(USE_ANALOG)
    val = analogRead(pinAnalog);
    #if defined(USE_SERIAL)
      Serial.print("Analog measured ");
      Serial.println(val);
    #endif
    aux = String(val, 1);
    if(prevAnalog != val || forced)
    {
      clientMqtt.publish(topicAnalog.c_str(), aux.c_str());
      prevAnalog = val;
      #if defined(USE_SERIAL)
      Serial.print(topicAnalog);
      Serial.println(" MQTT Published");
      #endif
    }
  #endif
}

float prevMotion;
void PublishMotion(boolean forced)
{
  float val;
  String aux;

  #if defined(USE_MOTION)
    val = digitalRead(pinMotion);
    #if defined(USE_SERIAL)
      Serial.print("Motion measured ");
      Serial.println(val);
    #endif
    aux = String(val, 0);
    if(prevMotion != val || forced)
    {
      clientMqtt.publish(topicMotion.c_str(), aux.c_str());
      prevMotion = val;
      #if defined(USE_SERIAL)
      Serial.print(topicMotion);
      Serial.println(" MQTT Published");
      #endif
    }
  #endif
}

float prevTemp, prevHumid;
void PublishDht(boolean forced)
{
  float val;
  String aux;

  #if defined(USE_DHT)
    val = dht.readTemperature(true);
    #if defined(USE_SERIAL)
      Serial.print("Temperature measured ");
      Serial.println(val);
    #endif
    if (!isnan(val) && (prevTemp != val || forced))
    {
      aux = String(val, 1);
      prevTemp = val;
      clientMqtt.publish(topicTemp.c_str(), aux.c_str());
      #if defined(USE_SERIAL)
      Serial.print(topicTemp);
      Serial.println(" MQTT Published");
      #endif
    }
    
    val = dht.readHumidity();
    #if defined(USE_SERIAL)
      Serial.print("Humidity measured ");
      Serial.println(val);
    #endif
    if (!isnan(val) && (prevHumid != val || forced))
    {
      prevHumid = val;
      aux = String(val, 1);
      clientMqtt.publish(topicHumid.c_str(), aux.c_str());
      #if defined(USE_SERIAL)
      Serial.print(topicHumid);
      Serial.println(" MQTT Published");
      #endif
    }
  #endif
}

float prevRelay;
void PublishRelay(boolean forced)
{
  float val;
  String aux;

  #if defined(USE_RELAY)
    val = digitalRead(pinRelay);
    #if defined(USE_SERIAL)
      Serial.print("Relay status ");
      Serial.println(val);
    #endif

    if (!isnan(val) && (prevRelay != val || forced))
    {
      prevRelay = val;
      aux = String(val, 1);
      clientMqtt.publish(topicRelay.c_str(), aux.c_str());
      #if defined(USE_SERIAL)
        Serial.print(topicRelay);
        Serial.println(" MQTT Published");
      #endif
    }
  #endif
}

void PublishFirmware()
{
  clientMqtt.publish(topicFirmware.c_str(), FIRMWARE);
  #if defined(USE_SERIAL)
    Serial.print(topicFirmware);
    Serial.print(FIRMWARE);
    Serial.println(" MQTT Published");
  #endif
}

void PublishAll(boolean forced)
{
  PublishDistance(forced);
  PublishRelay(forced);
  PublishAnalog(forced);
  PublishMotion(forced);
  PublishDht(forced);
}

