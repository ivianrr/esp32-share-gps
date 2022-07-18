#include <WiFi.h>
#include "config.h"

const char *ssid = WIFI_SSID;         // your network SSID (name of wifi network)
const char *password = WIFI_PASSWORD; // your network password

const IPAddress server = IPAddress(192, 168, 52, 141); // Server IP address
const int port = 2947;                                 // GPSD server's port
const String query = "?SHGPS.LOCATION;\r\n";

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(100);

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
}

void loop()
{
  // put your main code here, to run repeatedly:
  WiFiClient client;
  if (!client.connect(server, port))
  {
    Serial.println("Connection failed.");
    Serial.println("Waiting 5 seconds before retrying...");
    delay(5000);
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

  // When connected, we receive a version json thaat we need to read before we can retreive the location info.
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

  Serial.println("Waiting 5 seconds before restarting...");

  delay(5000);
}
