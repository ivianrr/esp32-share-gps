#include <WiFi.h>
#include <ArduinoJson.h>
#include <MQTT.h>
#include <PubSubClient.h>
#include <PubSubClient_JSON.h>
#include "config.h"
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>  
#include "SSD1306.h" 


#define DEVICENAME "esp32loragps"

#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISO
#define MOSI    27   // GPIO27 -- SX1278's MOSI
#define SS      18   // GPIO18 -- SX1278's CS
#define RST     23   // GPIO14 -- SX1278's RESET
#define DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define BAND    868E6
#define SF      9
#define SBW     62.5E3

SSD1306 display(0x3c, 21, 22);
String rssi = "RSSI --";
int rssi_n;
String packSize = "--";
String packet ;

const char* ssid     = WIFI_SSID;     // your network SSID (name of wifi network)
const char* password = WIFI_PASSWORD; // your network password

const char* mqtt_server = MQTT_SERVER;
const char* status_topic = "home/esp32gps/status";
const char* data_topic = "home/esp32gps/data";
const char* ip_topic = "home/esp32gps/ip";


IPAddress server = IPAddress(192, 168, 78, 61);  // Server IP address
const int port = 2947; // GPSD server's port 
const String query ="?SHGPS.LOCATION;\r\n";

WiFiClient client;
PubSubClient mqttclient(client);


void callback(const MQTT::Publish& pub);

void gpsreconnect();

IPAddress ip_from_str(String ipstr){
  int a[4];
  int i1=0;
  int i2=0;
  for(int i=0;i<4;i++){
    i2=ipstr.indexOf(".",i1);
    if(i2==-1) i2=ipstr.length();
    a[i]=ipstr.substring(i1,i2).toInt();
    i1=i2+1;
  }
  return IPAddress(a[0], a[1], a[2], a[3]); 
}

void loraData(){
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  display.drawString(0 , 15 , "Received "+ packSize + " bytes");
  display.drawStringMaxWidth(0 , 26 , 128, packet);
  display.drawString(0 , 40 , "SF: "+String(SF,DEC));
  display.drawString(0, 0, rssi); 
  display.display();
  Serial.println(rssi);
}

void hellora(String msg){
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, "LoRa module connecting."); 
  display.drawString(0 , 15 , "Please wait.");
  display.drawStringMaxWidth(0 , 26 , 128, msg);
  display.display();
}

void cbk(int packetSize) {
  packet ="";
  packSize = String(packetSize,DEC);
  for (int i = 0; i < packetSize; i++) { packet += (char) LoRa.read(); }
  rssi = "RSSI " + String(LoRa.packetRssi(), DEC) ;
  rssi_n=LoRa.packetRssi();
  loraData();
}

void setup() {
  // put your setup code here, to run once:
  pinMode(16,OUTPUT);
  digitalWrite(16, LOW);    // set GPIO16 low to reset OLED
  delay(50); 
  digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 in high、
  Serial.begin(115200);
  delay(100);

    Serial.println("LoRa Receiver Callback");
  SPI.begin(SCK,MISO,MOSI,SS);
  LoRa.setPins(SS,RST,DI0);  
  if (!LoRa.begin(868E6)) {
    Serial.println("Starting LoRa failed!");
    
    while (1);
  }
  LoRa.setSpreadingFactor(SF);  
  LoRa.setSignalBandwidth(SBW);

  //LoRa.onReceive(cbk);
  //LoRa.receive(); //no es necesario
  Serial.println("init ok");
  display.init();
  display.flipScreenVertically();  
  display.setFont(ArialMT_Plain_10);

  hellora("Lora init OK.");

  
  Serial.print("Attempting to connect to SSID: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  // attempt to connect to Wifi network:
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    // wait 1 second for re-trying
    delay(1000);
  }

  Serial.print("Connected to ");
  Serial.println(ssid);
  mqttclient.set_server(mqtt_server, 1883);
  mqttclient.set_callback(callback);
  mqttclient.subscribe(ip_topic);
  mqttclient.loop();
  hellora("Connected to WiFi");
  gpsreconnect();
}


void loop() {
  // put your main code here, to run repeatedly:
  int packetSize = LoRa.parsePacket();
  if (packetSize) { cbk(packetSize);  }
  else{
    delay(100);
    return;
  }
  Serial.print("Looking for GPSD server at ");
  Serial.print(server);
  Serial.print(":");
  Serial.println(port);

  if (!client.connect(server, port)) {
      Serial.println("Connection to GPSD server failed.");
      gpsreconnect();
      return;
  } 
  
  client.print("?SHGPS.LOCATION;\r\n");
  
  int maxloops = 0;

  //wait for the server's reply to become available
  while (!client.available() && maxloops < 1000)
  {
    maxloops++;
    delay(1); //delay 1 msec
  }
  if(maxloops==1000){
    Serial.println("client.available() timed out ");
    return;
  }
  
  Serial.println("Conected to client, awaiting for response.");

  // When connected, we receive a version json thaat we need to read before we can retreive the location info.
  while (client.connected()) {
    String line = client.readStringUntil('\n');
    if (line.indexOf("VERSION")!=-1) {
      Serial.println("Header received.");
      Serial.println(line);
      break;
    }
  }
  // This string should countain the location
  String msg = client.readStringUntil('\r');
  
  Serial.println("Msg is:");
  Serial.println(msg);
  
  Serial.println("Closing connection.");
  client.stop();

  // Parsing json and retreiving the coordinates.
  Serial.println("Parsing json...");
  StaticJsonDocument<300> doc;
  DeserializationError error = deserializeJson(doc, msg);
  // Test if parsing succeeds.
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }
  doc["rssi"]=rssi_n;
  doc["sf"]=SF;
  int gpsmode=doc["mode"];
  double alt=doc["alt"];
  double latit=doc["lat"];
  double longi=doc["lon"];
  const char* datetime = doc["time"];
  Serial.println(datetime);
  Serial.print("Mode: ");
  Serial.println(gpsmode);
  Serial.print("Altitude: ");
  Serial.println(alt,4);
  Serial.print("Latitude: ");
  Serial.println(latit, 4);
  Serial.print("Longitude: ");
  Serial.println(longi, 4);  
  Serial.print("RSSI: ");
  Serial.println(rssi_n);

  Serial.println("Attempting send data to MQTT server.");
  if (!mqttclient.connected()) {
    reconnect();
  }
  mqttclient.loop();
  Serial.println("\nPublisihing json to MQTT server...");
  mqttclient.publish(MQTT::PublishJSON(data_topic, doc)
              .set_qos(1));
  Serial.println("Done");
  
  Serial.println("Waiting 2 seconds before restarting...");
  delay(2000);
}

void callback(const MQTT::Publish& pub) {
  Serial.print(pub.topic());
  Serial.print(" => ");
  if (pub.has_stream()) {
//    uint8_t buf[BUFFER_SIZE];
//    int read;
//    while (read = pub.payload_stream()->read(buf, BUFFER_SIZE)) {
//      Serial.write(buf, read);
//    }
//    pub.payload_stream()->stop();
    Serial.println("Payload is stream?");
  } else{
    String ip_str=pub.payload_string();
    Serial.println(ip_str);
    
    server = ip_from_str(ip_str);
    }
}



void reconnect() {
  // Loop until we're reconnected
  while (!mqttclient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    MQTT::Connect con(DEVICENAME);
    con.set_will(status_topic, "Device down.",true).set_keepalive(30);
    if (mqttclient.connect(con)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      mqttclient.publish(MQTT::Publish(status_topic, "Device online.").set_retain());
      // ... and resubscribe
      //client.subscribe("nodemcu8266/led");
    } else {
      Serial.print("failed, rc=");
      //Serial.print(client.state()); //no funciona, no se como sustituirlo
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void gpsreconnect(){
    hellora("Connecting to GPS...");

    Serial.println("Trying  to connect to MQTT server to retrieve new ip.");
    
    if (!mqttclient.connected()) {
      reconnect();
    }    

    if (mqttclient.connected()){
      Serial.println("MQTT loop");
      mqttclient.loop();
      mqttclient.subscribe(ip_topic);  
      Serial.println("Waiting 5 seconds before retrying...");
      delay(5000);
      mqttclient.loop();
    }   
}
