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

#define SCK 5      // GPIO5  -- SX1278's SCK
#define MISO 19    // GPIO19 -- SX1278's MISO
#define MOSI 27    // GPIO27 -- SX1278's MOSI
#define SS 18      // GPIO18 -- SX1278's CS
#define RST 23     // GPIO14 -- SX1278's RESET
#define DI0 26     // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define BAND 868E6 // Band depends on the region, for Europe it is 868E6 (868 MHz)
#define SF 9       // Spreading factor, 6-12 default 7. Higher is slowwer but allows longer range.
#define SBW 62.5E3 // Bandwidth, defaults to 125E3. Supported values are 7.8E3, 10.4E3,
                   // 15.6E3, 20.8E3, 31.25E3, 41.7E3, 62.5E3, 125E3, and 250E3.

SSD1306 display(0x3c, 21, 22);
String rssi = "RSSI --";
int rssi_n;
String packSize = "--";
String packet;

const char *ssid = WIFI_SSID;         // your network SSID (name of wifi network)
const char *password = WIFI_PASSWORD; // your network password

const char *mqtt_server = MQTT_SERVER;             // Address of the MQTT server to which measurements are sent
                                                   // and from wich GPSD server IP address is retrieved
const char *status_topic = "home/esp32gps/status"; // Optional topic that shows if device has been connected during the last 30 seconds
const char *data_topic = "home/esp32gps/data";     // Topic for the data (Nodered should subscribe to this topic if data is to be stored in a db).
const char *ip_topic = "home/esp32gps/ip";         // Topic that has the GPSD IP address retained as a a string.

IPAddress server = IPAddress(192, 168, 78, 61); // Provisional server IP address. UPdating via mqtt can be avoided if this address is correct. However, Android devices reset it randomly.
const int port = 2947;                          // GPSD server's port
const String query = "?SHGPS.LOCATION;\r\n";

WiFiClient client;
PubSubClient mqttclient(client);

/**
 * MQTT callback function. Only called when the GPSD server IP is updated via ip_topic.
 * @param pub Received published message.
 */
void callback(const MQTT::Publish &pub);

/**
 * Reconnect to MQTT server.
 */
void reconnect();

/**
 * Connect to MQTT and wait for IP. Update server global variable with new IPAddress object.
 */
void gpsreconnect();

/**
 * Split IP inside String and construct an IPAddress with it.
 *
 * @param ipstr Address string in "111.222.333.444" format or similar.
 * @return IPAddress object
 */
IPAddress ip_from_str(String ipstr);

/**
 * Read LoRa message and call loraData to display it.
 *
 * @param packetSize Size of the packet in bytes.
 */
void cbk(int packetSize);

/**
 * Display LoRa data of last received message.
 */
void loraData();

/**
 * Show a message in the OLEd screen during initiallization.
 *
 * @param msg Message to output.
 */
void displaymsg(String msg);

void setup()
{
  pinMode(16, OUTPUT);
  digitalWrite(16, LOW); // set GPIO16 low to reset OLED
  delay(50);
  digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 in high、
  Serial.begin(115200);
  delay(100);

  Serial.println("LoRa starting...");
  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RST, DI0);
  if (!LoRa.begin(868E6))
  {
    Serial.println("Starting LoRa failed!");

    while (1)
      ;
  }
  LoRa.setSpreadingFactor(SF);
  LoRa.setSignalBandwidth(SBW);

  // LoRa.onReceive(cbk);
  // LoRa.receive(); // not needed
  Serial.println("init ok");
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);

  displaymsg("Lora init OK. Looking for WiFi.");

  Serial.print("Attempting to connect to SSID: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  // attempt to connect to Wifi network:
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    // wait 1 second for re-trying
    delay(1000);
  }

  Serial.print("Connected to ");
  Serial.println(ssid);
  displaymsg("Connected to WiFi");

  mqttclient.set_server(mqtt_server, 1883);
  mqttclient.set_callback(callback);
  mqttclient.subscribe(ip_topic);
  mqttclient.loop();

  // Connect to MQTT server to get retained message with the ip address of the GPS server (Usually the hotspot phone)
  gpsreconnect();
}

void loop()
{
  // Poll every 100 ms for new messages, if there are none skip the rest of the code.
  int packetSize = LoRa.parsePacket();
  if (packetSize)
  {
    // If msg is found, read it, display it and proceed with the rest of the code.
    cbk(packetSize);
  }
  else
  {
    delay(100);
    return;
  }

  Serial.print("Looking for GPSD server at ");
  Serial.print(server);
  Serial.print(":");
  Serial.println(port);

  if (!client.connect(server, port))
  {
    Serial.println("Connection to GPSD server failed.");
    gpsreconnect();
    return;
  }

  client.print("?SHGPS.LOCATION;\r\n");

  int maxloops = 0;

  // wait for the server's reply to become available
  while (!client.available() && maxloops < 1000)
  {
    maxloops++;
    delay(1); // delay 1 msec
  }
  if (maxloops == 1000)
  {
    Serial.println("client.available() timed out ");
    return;
  }

  Serial.println("Conected to client, awaiting for response.");

  // When connected, we receive a version json that we need to read before we can retreive the location info.
  while (client.connected())
  {
    String line = client.readStringUntil('\n');
    if (line.indexOf("VERSION") != -1)
    {
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

  // Parsing json and retrieving the coordinates.
  Serial.println("Parsing json...");
  StaticJsonDocument<300> doc;
  DeserializationError error = deserializeJson(doc, msg);
  // Test if parsing succeeds.
  if (error)
  {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  // Add the LoRa data to the json before forwarding it to the MQTT server
  doc["rssi"] = rssi_n;
  doc["sf"] = SF;

  // Display GPS data via serial
  int gpsmode = doc["mode"];
  double alt = doc["alt"];
  double latit = doc["lat"];
  double longi = doc["lon"];
  const char *datetime = doc["time"];
  Serial.println(datetime);
  Serial.print("Mode: ");
  Serial.println(gpsmode);
  Serial.print("Altitude: ");
  Serial.println(alt, 4);
  Serial.print("Latitude: ");
  Serial.println(latit, 4);
  Serial.print("Longitude: ");
  Serial.println(longi, 4);
  Serial.print("RSSI: ");
  Serial.println(rssi_n);

  Serial.println("Attempting send data to MQTT server.");
  if (!mqttclient.connected())
  {
    reconnect();
  }
  mqttclient.loop();
  Serial.println("\nPublisihing json to MQTT server...");
  mqttclient.publish(MQTT::PublishJSON(data_topic, doc)
                         .set_qos(1));
  Serial.println("Done");

  Serial.println("Waiting 2 seconds before restarting...");
  delay(2000); // At least 2 seconds between measurements, although lora beacon should have a longer period anyway.
}

void callback(const MQTT::Publish &pub)
{
  Serial.print(pub.topic());
  Serial.print(" => ");
  if (pub.has_stream())
  {
    //    uint8_t buf[BUFFER_SIZE];
    //    int read;
    //    while (read = pub.payload_stream()->read(buf, BUFFER_SIZE)) {
    //      Serial.write(buf, read);
    //    }
    //    pub.payload_stream()->stop();
    Serial.println("Payload is stream?");
  }
  else
  {
    String ip_str = pub.payload_string();
    Serial.println(ip_str);

    server = ip_from_str(ip_str);
  }
}

void reconnect()
{
  // Loop until we're reconnected
  while (!mqttclient.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    MQTT::Connect con(DEVICENAME);
    con.set_will(status_topic, "Device down.", true).set_keepalive(30);
    if (mqttclient.connect(con))
    {
      Serial.println("connected");
      // Once connected, publish an announcement...
      mqttclient.publish(MQTT::Publish(status_topic, "Device online.").set_retain());
      // ... and resubscribe
      // client.subscribe("nodemcu8266/led");
    }
    else
    {
      Serial.print("failed, rc=");
      // Serial.print(client.state()); //no funciona, no se como sustituirlo
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void gpsreconnect()
{
  displaymsg("Connecting to GPS...");

  Serial.println("Trying  to connect to MQTT server to retrieve new ip.");

  if (!mqttclient.connected())
  {
    reconnect();
  }

  if (mqttclient.connected())
  {
    Serial.println("MQTT loop");
    mqttclient.loop();
    mqttclient.subscribe(ip_topic);
    Serial.println("Waiting 5 seconds before retrying...");
    delay(5000);
    mqttclient.loop();
  }
}

IPAddress ip_from_str(String ipstr)
{
  int a[4];
  int i1 = 0;
  int i2 = 0;
  for (int i = 0; i < 4; i++)
  {
    i2 = ipstr.indexOf(".", i1);
    if (i2 == -1)
      i2 = ipstr.length();
    a[i] = ipstr.substring(i1, i2).toInt();
    i1 = i2 + 1;
  }
  return IPAddress(a[0], a[1], a[2], a[3]);
}

void loraData()
{
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 15, "Received " + packSize + " bytes");
  display.drawStringMaxWidth(0, 26, 128, packet);
  display.drawString(0, 40, "SF: " + String(SF, DEC));
  display.drawString(0, 0, rssi);
  display.display();
  Serial.println(rssi);
}

void displaymsg(String msg)
{
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, "LoRa module connecting.");
  display.drawString(0, 15, "Please wait.");
  display.drawStringMaxWidth(0, 26, 128, msg);
  display.display();
}

void cbk(int packetSize)
{
  packet = "";
  packSize = String(packetSize, DEC);
  for (int i = 0; i < packetSize; i++)
  {
    packet += (char)LoRa.read();
  }
  rssi = "RSSI " + String(LoRa.packetRssi(), DEC);
  rssi_n = LoRa.packetRssi();
  loraData();
}
