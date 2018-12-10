#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <RunningMedian.h>

// Uncomment to enable 
// #define USE_SERIAL 1
// Analog input
//#define USE_ANALOG 1
// Motion detector
#define USE_MOTION 1
// Temp & Humid module
// #define USE_DHT 1
// Sonar sensor
// #define USE_DISTANCE 1
// Relay
#define USE_RELAY 1
// Relay
#define USE_RELAY2 1
// Built in led
#define USE_LED 1

const char* FIRMWARE = "RABCBot 2018-12-10 7:20AM";
const int TIME_DELAY = 2500;

const char* ssid = "your-ssid";
const char* password = "your-password";

const char* mqttBroker = "your-broker";
const int mqttPort = 1883;
String mqttUser;

const char* PAYLOAD_ON = "ON";
const char* PAYLOAD_OFF = "OFF";
const char* PAYLOAD_BLINK = "BLINK";

const String topicPrefix = "home/";
String topicFirmware = "/firmware";
String topicFirmwareGet = "/firmware/get"; // Return firmware

String topicPinDht = "/pin/dht";
String topicPinMotion = "/pin/motion";
String topicPinRelay = "/pin/relay";
String topicPinRelay2 = "/pin/relay2";
String topicPinTrigger = "/pin/trigger";
String topicPinEcho = "/pin/echo";
String topicPinAnalog = "/pin/analog";
String topicPinGet = "/pin/get"; 

String topicTemp = "/temperature";
String topicHumid = "/humidity";
String topicDist = "/distance";
String topicDistMedian = "/distanceMedian";
String topicMotion = "/motion";
String topicAnalog = "/analog";
String topicLedSet = "/led/set";
String topicRelay = "/relay"; // Returns status of relay
String topicRelaySet = "/relay/set"; // 1 is on, 0 is off
String topicRelay2 = "/relay2"; // Returns status of relay
String topicRelay2Set = "/relay2/set"; // 1 is on, 0 is off
String topicForce = "home/status"; // Force publish status

/* Wemos D1 Pins
TX  TXD TXD
RX  RXD RXD
A0  Analog input, max 3.3V input  A0
D0  IO  GPIO16
D1  IO, SCL GPIO5
D2  IO, SDA GPIO4
D3  IO, 10k Pull-up GPIO0
D4  IO, 10k Pull-up, BUILTIN_LED  GPIO2
D5  IO, SCK GPIO14
D6  IO, MISO  GPIO12
D7  IO, MOSI  GPIO13
D8  IO, 10k Pull-down, SS GPIO15
G Ground  GND
*/

int pinMotion = 16;  // D0
int pinRelay = 5; // D1
int pinRelay2 = 4; // D2
int pinDht = 0; // D3
int pinLed = 2;
int pinTrig = 12;  // D6
int pinEcho = 13;  // D7
int pinAnalog = A0;

#if defined(USE_LED)
bool ledBlink = false;
#endif

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
  
  topicPinDht = topicPrefix + mqttUser + topicPinDht;
  topicPinMotion = topicPrefix + mqttUser + topicPinMotion;
  topicPinRelay = topicPrefix + mqttUser + topicPinRelay;
  topicPinRelay2 = topicPrefix + mqttUser + topicPinRelay2;
  topicPinTrigger = topicPrefix + mqttUser + topicPinTrigger;
  topicPinEcho = topicPrefix + mqttUser + topicPinEcho;
  topicPinAnalog = topicPrefix + mqttUser + topicPinAnalog;
  topicPinGet = topicPrefix + mqttUser + topicPinGet;
  
  topicTemp = topicPrefix + mqttUser + topicTemp;
  topicHumid = topicPrefix + mqttUser + topicHumid;
  topicDist = topicPrefix + mqttUser + topicDist;
  topicDistMedian = topicPrefix + mqttUser + topicDistMedian;
  topicMotion = topicPrefix + mqttUser + topicMotion;
  topicAnalog = topicPrefix + mqttUser + topicAnalog;
  topicRelaySet = topicPrefix + mqttUser + topicRelaySet;
  topicRelay = topicPrefix + mqttUser + topicRelay;
  topicRelay2Set = topicPrefix + mqttUser + topicRelay2Set;
  topicRelay2 = topicPrefix + mqttUser + topicRelay2;

  topicLedSet = topicPrefix + mqttUser + topicLedSet;

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

  #if defined(USE_RELAY)
  if (String(topic) == topicRelaySet)
  {
    if (strncmp((char *)payload, PAYLOAD_ON, length) == 0) 
      digitalWrite(pinRelay, HIGH);
    else 
    if (strncmp((char *)payload, PAYLOAD_OFF, length) == 0) 
      digitalWrite(pinRelay, LOW);
    PublishRelay(true);
  }
  #endif

  #if defined(USE_RELAY2)
  if (String(topic) == topicRelay2Set)
  {
    if (strncmp((char *)payload, PAYLOAD_ON, length) == 0) 
      digitalWrite(pinRelay2, HIGH);
    else 
    if (strncmp((char *)payload, PAYLOAD_OFF, length) == 0) 
      digitalWrite(pinRelay2, LOW);
    PublishRelay2(true);
  }
  #endif

  #if defined(USE_LED)
  if (String(topic) == topicLedSet)
  {
    if (strncmp((char *)payload, PAYLOAD_ON, length) == 0) 
    {
      ledBlink = false; 
      digitalWrite(pinLed, LOW);
    }
    else 
    if (strncmp((char *)payload, PAYLOAD_OFF, length) == 0) 
    {
      ledBlink = false; 
      digitalWrite(pinLed, HIGH);
    }
    else
    if (strncmp((char *)payload, PAYLOAD_BLINK, length) == 0) 
      ledBlink = true;
  }
  #endif

  if (String(topic) == topicForce)
  {
    PublishAll(true);    
  }
  if (String(topic) == topicFirmwareGet)
  {
    PublishFirmware();    
  }
  if (String(topic) == topicPinGet)
  {
    PublishPins();    
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
      clientMqtt.subscribe(topicPinGet.c_str());
      #if defined(USE_RELAY)
      clientMqtt.subscribe(topicRelaySet.c_str());
      clientMqtt.subscribe(topicRelay2Set.c_str());
      #endif
      #if defined(USE_LED)
      clientMqtt.subscribe(topicLedSet.c_str());
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
  #if defined(USE_RELAY2)
  pinMode(pinRelay2, OUTPUT);
  #endif
  #if defined(USE_DISTANCE)
  pinMode(pinTrig, OUTPUT);
  pinMode(pinEcho, INPUT);
  #endif
  #if defined(USE_MOTION)
  pinMode(pinMotion, INPUT);
  #endif  
  #if defined(USE_LED)
  pinMode(pinLed, OUTPUT);
  digitalWrite(pinLed, HIGH);
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
  Serial.begin(57600);
  #endif
  setup_wifi();
  setup_mqtt();
  setup_dht();
  setup_pins();
  //PublishAll(true);
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
    if(ledBlink) Blink();
  }
}

void Blink()
{
#if defined(USE_LED)
  float val;

  val = digitalRead(pinLed);
  #if defined(USE_SERIAL)
  Serial.print("Led status ");
  Serial.println(val);
  #endif

  if (!isnan(val))
  {
    if(val == 1)
      digitalWrite(pinLed, LOW);
    else
      digitalWrite(pinLed, HIGH);
  }
#endif  
}


float prevDistMedian;
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
    
    // Publish distance
    /*
    aux = String(dist, 1);
    if(prevDist != dist || forced)
    {
      // clientMqtt.publish(topicDist.c_str(), aux.c_str());
      prevDist = dist;
      #if defined(USE_SERIAL)
      Serial.print(topicDist);
      Serial.println(" MQTT Published");
      #endif
    }
    */

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

  #if defined(USE_MOTION)
    val = digitalRead(pinMotion);
    #if defined(USE_SERIAL)
      Serial.print("Motion measured ");
      Serial.println(val);
    #endif
    if(prevMotion != val || forced)
    {
      if(val == 1)
        clientMqtt.publish(topicMotion.c_str(), "ON");
      else
        clientMqtt.publish(topicMotion.c_str(), "OFF");
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

  #if defined(USE_RELAY)
    val = digitalRead(pinRelay);
    #if defined(USE_SERIAL)
      Serial.print("Relay status ");
      Serial.println(val);
    #endif

    if (!isnan(val) && (prevRelay != val || forced))
    {
      prevRelay = val;
      if(val == 1)
        clientMqtt.publish(topicRelay.c_str(), "ON");
      else
        clientMqtt.publish(topicRelay.c_str(), "OFF");
      #if defined(USE_SERIAL)
        Serial.print(topicRelay);
        Serial.println(" MQTT Published");
      #endif
    }
  #endif
}

float prevRelay2;
void PublishRelay2(boolean forced)
{
  float val;
  String aux;

  #if defined(USE_RELAY2)
    val = digitalRead(pinRelay2);
    #if defined(USE_SERIAL)
      Serial.print("Relay2 status ");
      Serial.println(val);
    #endif

    if (!isnan(val) && (prevRelay2 != val || forced))
    {
      prevRelay2 = val;
      if(val == 1)
        clientMqtt.publish(topicRelay2.c_str(), "ON");
      else
        clientMqtt.publish(topicRelay2.c_str(), "OFF");
      #if defined(USE_SERIAL)
        Serial.print(topicRelay2);
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

void PublishPins()
{
  float val;
  String aux;
  
  #if defined(USE_DHT)
  val = pinDht;
  aux = String(val, 0);
  #if defined(USE_SERIAL)
  Serial.print(topicPinDht);
  Serial.print(aux);
  Serial.println(" MQTT Published");
  #endif
  clientMqtt.publish(topicPinDht.c_str(), aux.c_str());
  #endif

  #if defined(USE_MOTION)
  val = pinMotion;
  aux = String(val, 0);
  #if defined(USE_SERIAL)
  Serial.print(topicPinMotion);
  Serial.print(aux);
  Serial.println(" MQTT Published");
  #endif
  clientMqtt.publish(topicPinMotion.c_str(), aux.c_str());
  #endif

  #if defined(USE_RELAY)
  val = pinRelay;
  aux = String(val, 0);
  #if defined(USE_SERIAL)
  Serial.print(topicPinRelay);
  Serial.print(aux);
  Serial.println(" MQTT Published");
  #endif
  clientMqtt.publish(topicPinRelay.c_str(), aux.c_str());
  #endif

  #if defined(USE_RELAY2)
  val = pinRelay2;
  aux = String(val, 0);
  #if defined(USE_SERIAL)
  Serial.print(topicPinRelay2);
  Serial.print(aux);
  Serial.println(" MQTT Published");
  #endif
  clientMqtt.publish(topicPinRelay2.c_str(), aux.c_str());
  #endif
  
  #if defined(USE_DISTANCE)
  val = pinTrig;
  aux = String(val, 0);
  #if defined(USE_SERIAL)
  Serial.print(topicPinTrigger);
  Serial.print(aux);
  Serial.println(" MQTT Published");
  #endif
  clientMqtt.publish(topicPinTrigger.c_str(), aux.c_str());
  val = pinEcho;
  aux = String(val, 0);
  #if defined(USE_SERIAL)
  Serial.print(topicPinEcho);
  Serial.print(aux);
  Serial.println(" MQTT Published");
  #endif
  clientMqtt.publish(topicPinEcho.c_str(), aux.c_str());
  #endif

  #if defined(USE_ANALOG)
  val = pinAnalog;
  aux = String(val, 0);
  #if defined(USE_SERIAL)
  Serial.print(topicPinAnalog);
  Serial.print(aux);
  Serial.println(" MQTT Published");
  #endif
  clientMqtt.publish(topicPinAnalog.c_str(), aux.c_str());
  #endif
}

void PublishAll(boolean forced)
{
  PublishDistance(forced);
  PublishRelay(forced);
  PublishRelay2(forced);
  PublishAnalog(forced);
  PublishMotion(forced);
  PublishDht(forced);
}
