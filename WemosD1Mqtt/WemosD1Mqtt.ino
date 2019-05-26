#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <RunningMedian.h>

// Uncomment to enable 
// #define USE_SERIAL 1
// Analog input
// #define USE_ANALOG 1
// Motion detector
// #define USE_MOTION 1
// Temp & Humid module
// #define USE_DHT 1
// Sonar sensor
#define USE_DISTANCE 1
#define USE_PRESENCE 1
// Relay1
// #define USE_RELAY1 1
// Relay2
// #define USE_RELAY2 1
// Built in led = blink
#define USE_BLINK 1

const char* FIRMWARE = "RABCBot 2019-05-26 01:00PM";
String topicShema = "shema";
const char* SHEMA = "Hear, O Israel: The Lord our God, the Lord is one. (Deut 6:4)";
const char* SHEMA2 = "You shall love the Lord your God with all your heart and with all your soul and with all your might. (Deut 6:5)";

const int TIME_DELAY = 2000;

const char* ssid = "<your-ssid>";
const char* password = "<your-password>";
String hostname;

const char* mqttBroker = "<your-broker-ip>";
const int mqttPort = 1883;

const char* PAYLOAD_TRUE = "true";
const char* PAYLOAD_FALSE = "false";
const char* PAYLOAD_BLINK = "blink";

String topicPrefix = "home/"; // Prefix + mqtt name
String topicStatus = "home/status";
String topicFirmware = "firmware";
String topicPins = "pins";

/* Wemos D1 Pins reference
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

#if defined(USE_DHT)
int pinDht = 0;
DHT dht(pinDht, DHT11);
String topicTemperature = "temperature";
String topicHumidity = "humidity";
String topicPinDht = "pin_dht";
#endif

#if defined(USE_DISTANCE)
int pinTrig = 5; // D1
int pinEcho = 4; // D2
RunningMedian distMedian = RunningMedian(4);
bool forceDistance = false;
float distanceThreshold = 27;
String topicDistance = "distance";
String topicForceDistance = "publish_distance";
String topicPresence = "presence";
String topicDistanceThreshold = "distance_threshold";
String topicPinTrigger = "pin_trigger";
String topicPinEcho = "pin_echo";
#endif

#if defined(USE_MOTION)
int pinMotion = 0;
String topicMotion = "motion";
String topicPinMotion = "pin_motion";
#endif

#if defined(USE_ANALOG)
int pinAnalog = A0;
String topicAnalog = "analog";
String topicPinAnalog = "pin_analog";
#endif

#if defined(USE_BLINK)
int pinBlink = 2; // 2 is D4 builtin led
bool blink = false;
String topicBlink = "blink";
String topicPinBlink = "pin_blink";
#endif

#if defined(USE_RELAY1)
int pinRelay1 = 0;
String topicRelay1 = "relay1"; 
String topicPinRelay1 = "pin_relay1";
#endif

#if defined(USE_RELAY2)
int pinRelay2 = 0;
String topicRelay2 = "relay2"; 
String topicPinRelay2 = "pin_relay2";
#endif

WiFiClient clientWifi;
PubSubClient clientMqtt(clientWifi);

void setup_wifi()
{
  #if defined(USE_SERIAL)
  Serial.println();
  Serial.print("Connecting to ");
  Serial.print(ssid);
  #endif

  hostname = "ESP_" + String(ESP.getChipId(), HEX);
  hostname.toUpperCase();

  delay(10);
  WiFi.mode(WIFI_STA);
  WiFi.hostname(hostname.c_str());
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
  topicPrefix = topicPrefix + hostname + "/";
  #if defined(USE_SERIAL)
  Serial.print("Topic prefix: ");
  Serial.println(topicPrefix);
  #endif
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

  String verb, key;
  SplitTopic(topic, &verb, &key);

  #if defined(USE_RELAY1)
  if (verb == "set" && key == topicRelay1)
  {
    if (strncmp((char *)payload, PAYLOAD_TRUE, length) == 0) 
      digitalWrite(pinRelay1, HIGH);
    else 
    if (strncmp((char *)payload, PAYLOAD_FALSE, length) == 0) 
      digitalWrite(pinRelay1, LOW);
    PublishRelay1(true);
  }
  #endif

  #if defined(USE_RELAY2)
  if (verb == "set" && key == topicRelay2)
  {
    if (strncmp((char *)payload, PAYLOAD_TRUE, length) == 0) 
      digitalWrite(pinRelay2, HIGH);
    else 
    if (strncmp((char *)payload, PAYLOAD_FALSE, length) == 0) 
      digitalWrite(pinRelay2, LOW);
    PublishRelay2(true);
  }
  #endif

  #if defined(USE_BLINK)
  if (verb == "set" && key == topicBlink)
  {
    if (strncmp((char *)payload, PAYLOAD_TRUE, length) == 0) 
    {
      blink = false; 
      digitalWrite(pinBlink, LOW);
    }
    else 
    if (strncmp((char *)payload, PAYLOAD_FALSE, length) == 0) 
    {
      blink = false; 
      digitalWrite(pinBlink, HIGH);
    }
    else
    if (strncmp((char *)payload, PAYLOAD_BLINK, length) == 0) 
      blink = true;
    PublishBlink(true);
  }
  #endif

  #if defined(USE_DISTANCE)
  if (verb == "set" && key == topicForceDistance)
  {
    if (strncmp((char *)payload, PAYLOAD_TRUE, length) == 0) 
    {
      forceDistance = true; 
    }
    else 
    if (strncmp((char *)payload, PAYLOAD_FALSE, length) == 0) 
    {
      forceDistance = false; 
    }
    PublishForceDistance();
  }
  #endif

 #if defined(USE_PRESENCE)
  if (verb == "set" && key == topicDistanceThreshold)
  {
    distanceThreshold = strtol((char *)payload, NULL, 0);
  }
  #endif

  if (String(topic) == topicStatus)
  {
    PublishAll(true);    
  }
  if (verb == "get" && key == topicFirmware)
  {
    PublishFirmware();    
  }
  if (verb == "get" && key == topicPins)
  {
    PublishPins();    
  }
  if (verb == "get" && key == topicShema)
  {
    PublishShema();    
  }
  #if defined(USE_DISTANCE)
  if (verb == "get" && key == topicForceDistance)
  {
    PublishForceDistance();    
  }
  #endif
}

void SplitTopic(char * topic, String *verb, String *key)
{
  char* ptr = strtok(topic, "/");
  char* strs[12];
  byte i = 0;
  while (ptr != NULL)
  {
    strs[i++] = ptr;
    ptr = strtok(NULL, "/");
  }
  if (i > 2)
  {
    *verb = strs[i-2];
    *key = strs[i-1];
  }
}

void reconnect()
{
  String topic;

  // Loop until we're reconnected
  while (!clientMqtt.connected()) 
  {
    #if defined(USE_SERIAL)
      Serial.print("Connecting to MQTT Broker...");
    #endif
    // Attempt to connect
    if (clientMqtt.connect(hostname.c_str())) 
    {
      #if defined(USE_SERIAL)
        Serial.println("connected");
        Serial.println("");
      #endif
      // re-subscribe
      clientMqtt.subscribe(topicStatus.c_str());
      topic = topicPrefix + "set/#";
      clientMqtt.subscribe(topic.c_str());
      topic = topicPrefix + "get/#";
      clientMqtt.subscribe(topic.c_str());
      topic = topicPrefix + "cmd/#";
      clientMqtt.subscribe(topic.c_str());
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
  #if defined(USE_RELAY1)
  pinMode(pinRelay1, OUTPUT);
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
  #if defined(USE_BLINK)
  pinMode(pinBlink, OUTPUT);
  digitalWrite(pinBlink, HIGH);
  #endif  
}

void setup_dht()
{
  #if defined(USE_DHT)
  dht.begin();
  #endif
}

void setup_serial()
{
  #if defined(USE_SERIAL)
  Serial.begin(57600);
  #endif
}

void setup()
{
  setup_serial();
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
    #if defined(USE_BLINK)
    if(blink) Blink();
    #endif
  }
}

void Blink()
{
#if defined(USE_BLINK)
  float val;

  val = digitalRead(pinBlink);
  #if defined(USE_SERIAL)
  Serial.print("Led status ");
  Serial.println(val);
  #endif

  if (!isnan(val))
  {
    if(val == 1)
      digitalWrite(pinBlink, LOW);
    else
      digitalWrite(pinBlink, HIGH);
  }
#endif  
}


bool prevPresence;
void PublishDistance(boolean forced)
{
  float val, dist;
  bool presence;
  String topic;
  String aux;
  
  #if defined(USE_DISTANCE)
    digitalWrite(pinTrig, HIGH);
    delayMicroseconds(10);
    digitalWrite(pinTrig, LOW);
    val = pulseIn(pinEcho, HIGH);
    dist = val / 160;
    #if defined(USE_SERIAL)
      Serial.print("Instance distance measured: ");
      Serial.println(dist);
    #endif
    
    // Publish current value
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
      Serial.print("Median distance calculated: ");
      Serial.println(dist);
    #endif

    // Publish distance
    if(forced || forceDistance)
    {
      aux = String(dist, 1);
      topic = topicPrefix + topicDistance;
      clientMqtt.publish(topic.c_str(), aux.c_str());
      #if defined(USE_SERIAL)
      Serial.print(topic);
      Serial.print(aux);
      Serial.println(" MQTT Published");
      #endif
    }
    
    // Detect presence
    #if defined(USE_DISTANCE)
    if (dist < distanceThreshold)
      presence = true;
    else
      presence = false;
    if(prevPresence != presence || forced)
    {
      topic = topicPrefix + topicPresence;
      if(presence)
      {
        clientMqtt.publish(topic.c_str(), PAYLOAD_TRUE);
        prevPresence = true;
      }
      else
      {
        clientMqtt.publish(topic.c_str(), PAYLOAD_FALSE);
        prevPresence = false;
      }
      #if defined(USE_SERIAL)
      Serial.print(topic);
      Serial.print(presence);
      Serial.println(" MQTT Published");
      #endif
    }
    #endif
  
  
  #endif
}

void PublishForceDistance()
{
  #if defined(USE_DISTANCE)
  String topic = topicPrefix + topicForceDistance;
  
  if(forceDistance)
    clientMqtt.publish(topic.c_str(), PAYLOAD_TRUE);
  else
    clientMqtt.publish(topic.c_str(), PAYLOAD_FALSE);
  #if defined(USE_SERIAL)
    Serial.print(topic);
    Serial.print(forceDistance);
    Serial.println(" MQTT Published");
  #endif
  #endif
}

float prevAnalog;
void PublishAnalog(boolean forced)
{
  #if defined(USE_ANALOG)
    float val;
    String aux;
    String topic = topicPrefix + topicAnalog;

    val = analogRead(pinAnalog);
    #if defined(USE_SERIAL)
      Serial.print("Analog measured ");
      Serial.println(val);
    #endif
    aux = String(val, 1);
    if(prevAnalog != val || forced)
    {
      clientMqtt.publish(topic.c_str(), aux.c_str());
      prevAnalog = val;
      #if defined(USE_SERIAL)
      Serial.print(topic);
      Serial.println(" MQTT Published");
      #endif
    }
  #endif
}

float prevMotion;
void PublishMotion(boolean forced)
{
  #if defined(USE_MOTION)
    float val;
    String topic = topicPrefix + topicMotion;
    
    val = digitalRead(pinMotion);
    #if defined(USE_SERIAL)
      Serial.print("Motion measured ");
      Serial.println(val);
    #endif
    if(prevMotion != val || forced)
    {
      if(val == 1)
        clientMqtt.publish(topic.c_str(), PAYLOAD_TRUE);
      else
        clientMqtt.publish(topic.c_str(), PAYLOAD_FALSE);
      prevMotion = val;
      #if defined(USE_SERIAL)
      Serial.print(topic);
      Serial.println(" MQTT Published");
      #endif
    }
  #endif
}

float prevTemp, prevHumid;
void PublishDht(boolean forced)
{
  #if defined(USE_DHT)
    float val;
    String aux;
    String topic;

    val = dht.readTemperature(true);
    #if defined(USE_SERIAL)
      Serial.print("Temperature measured ");
      Serial.println(val);
    #endif

    topic = topicPrefix + topicTemperature;
    if (!isnan(val) && (prevTemp != val || forced))
    {
      aux = String(val, 1);
      prevTemp = val;
      clientMqtt.publish(topic.c_str(), aux.c_str());
      #if defined(USE_SERIAL)
      Serial.print(topic);
      Serial.println(" MQTT Published");
      #endif
    }
    
    val = dht.readHumidity();
    #if defined(USE_SERIAL)
      Serial.print("Humidity measured ");
      Serial.println(val);
    #endif

    topic = topicPrefix + topicHumidity;
    if (!isnan(val) && (prevHumid != val || forced))
    {
      prevHumid = val;
      aux = String(val, 1);
      clientMqtt.publish(topic.c_str(), aux.c_str());
      #if defined(USE_SERIAL)
      Serial.print(topic);
      Serial.println(" MQTT Published");
      #endif
    }
  #endif
}

float prevRelay1;
void PublishRelay1(boolean forced)
{
  #if defined(USE_RELAY1)
    float val;
    String topic = topicPrefix + topicRelay1;
  
    val = digitalRead(pinRelay1);
    #if defined(USE_SERIAL)
      Serial.print("Relay1 status ");
      Serial.println(val);
    #endif

    if (!isnan(val) && (prevRelay1 != val || forced))
    {
      prevRelay1 = val;
      if(val == 1)
        clientMqtt.publish(topic.c_str(), PAYLOAD_TRUE);
      else
        clientMqtt.publish(topic.c_str(), PAYLOAD_FALSE);
      #if defined(USE_SERIAL)
        Serial.print(topic);
        Serial.println(" MQTT Published");
      #endif
    }
  #endif
}

float prevRelay2;
void PublishRelay2(boolean forced)
{
  #if defined(USE_RELAY2)
    float val;
    String topic = topicPrefix + topicRelay2;

    val = digitalRead(pinRelay2);
    #if defined(USE_SERIAL)
      Serial.print("Relay2 status ");
      Serial.println(val);
    #endif

    if (!isnan(val) && (prevRelay2 != val || forced))
    {
      prevRelay2 = val;
      if(val == 1)
        clientMqtt.publish(topic.c_str(), PAYLOAD_TRUE);
      else
        clientMqtt.publish(topic.c_str(), PAYLOAD_FALSE);
      #if defined(USE_SERIAL)
        Serial.print(topic);
        Serial.println(" MQTT Published");
      #endif
    }
  #endif
}

float prevBlink;
void PublishBlink(boolean forced)
{
  #if defined(USE_BLINK)
    float val;
    String topic = topicPrefix + topicBlink;
  
    val = digitalRead(pinBlink);
    #if defined(USE_SERIAL)
      Serial.print("Blink status ");
      Serial.println(val);
    #endif

    if (!isnan(val) && (prevBlink != val || forced))
    {
      prevBlink = val;
      if(val == 1)
        clientMqtt.publish(topic.c_str(), PAYLOAD_TRUE);
      else
        clientMqtt.publish(topic.c_str(), PAYLOAD_FALSE);
      #if defined(USE_SERIAL)
        Serial.print(topic);
        Serial.println(" MQTT Published");
      #endif
    }
  #endif
}

void PublishFirmware()
{
  String topic = topicPrefix + topicFirmware;
  
  clientMqtt.publish(topic.c_str(), FIRMWARE);
  #if defined(USE_SERIAL)
    Serial.print(topic);
    Serial.print(FIRMWARE);
    Serial.println(" MQTT Published");
  #endif
}

void PublishShema()
{
  String topic;
  
  topic = topicPrefix + topicShema;
  clientMqtt.publish(topic.c_str(), SHEMA);
   #if defined(USE_SERIAL)
    Serial.print(topic);
    Serial.print(SHEMA);
    Serial.println(" MQTT Published");
  #endif
}

void PublishPins()
{
  float val;
  String aux;
  String topic;
  
  #if defined(USE_DHT)
  val = pinDht;
  aux = String(val, 0);
  #if defined(USE_SERIAL)
  Serial.print(topicPinDht);
  Serial.print(aux);
  Serial.println(" MQTT Published");
  #endif
  topic = topicPrefix + topicPinDht;
  clientMqtt.publish(topic.c_str(), aux.c_str());
  #endif

  #if defined(USE_MOTION)
  val = pinMotion;
  aux = String(val, 0);
  topic = topicPrefix + topicPinMotion;
  #if defined(USE_SERIAL)
  Serial.print(topic);
  Serial.print(aux);
  Serial.println(" MQTT Published");
  #endif
  clientMqtt.publish(topic.c_str(), aux.c_str());
  #endif

  #if defined(USE_RELAY1)
  val = pinRelay1;
  aux = String(val, 0);
  topic = topicPrefix + topicPinRelay1;
  #if defined(USE_SERIAL)
  Serial.print(topic);
  Serial.print(aux);
  Serial.println(" MQTT Published");
  #endif
  clientMqtt.publish(topic.c_str(), aux.c_str());
  #endif

  #if defined(USE_RELAY2)
  val = pinRelay2;
  aux = String(val, 0);
  topic = topicPrefix + topicRelay2;
  #if defined(USE_SERIAL)
  Serial.print(topic);
  Serial.print(aux);
  Serial.println(" MQTT Published");
  #endif
  clientMqtt.publish(topic.c_str(), aux.c_str());
  #endif
  
  #if defined(USE_DISTANCE)
  val = pinTrig;
  aux = String(val, 0);
  topic = topicPrefix + topicPinTrigger;
  #if defined(USE_SERIAL)
  Serial.print(topic);
  Serial.print(aux);
  Serial.println(" MQTT Published");
  #endif
  clientMqtt.publish(topic.c_str(), aux.c_str());

  val = pinEcho;
  aux = String(val, 0);
  topic = topicPrefix + topicPinEcho;
  #if defined(USE_SERIAL)
  Serial.print(topic);
  Serial.print(aux);
  Serial.println(" MQTT Published");
  #endif
  clientMqtt.publish(topic.c_str(), aux.c_str());
  #endif

  #if defined(USE_ANALOG)
  val = pinAnalog;
  aux = String(val, 0);
  topic = topicPrefix + topicAnalog;
  #if defined(USE_SERIAL)
  Serial.print(topic);
  Serial.print(aux);
  Serial.println(" MQTT Published");
  #endif
  clientMqtt.publish(topic.c_str(), aux.c_str());
  #endif
}

void PublishAll(boolean forced)
{
  PublishDistance(forced);
  PublishRelay1(forced);
  PublishRelay2(forced);
  PublishAnalog(forced);
  PublishMotion(forced);
  PublishDht(forced);
}
