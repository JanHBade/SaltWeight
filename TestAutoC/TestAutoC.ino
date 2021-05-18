/*
 Name:		TestAutoC.ino
 Created:	10.02.2021 19:15:18
 Author:	jhb
*/


#include <ESP8266Wifi.h>
#include <ESP8266Webserver.h>
#include <AutoConnect.h>
#include <Wire.h>
#include <ESP8266Ping.h>
#include <ESP8266HTTPUpdateServer.h>
#include <PubSubClient.h>
#include <eHaJo_LM75.h>
#include <ArduinoJson.h>
#include <mDNSResolver.h>

#define LM75_ADDR  0x48
#define IO_ADDR 0x20

#define JSON_BUFFER_SIZE 150

#define HOSTNAME "Debug"

#define WLAN_NAME "Zuhause in Verl"
#define WLAN_PASSWORT "2103063030349494"

#define MQTT_BROKER "isis"
#define MQTT_PREFIX "SPS/"
#define MQTT_TOPIC_LED MQTT_PREFIX HOSTNAME "/Led"
#define MQTT_TOPIC_HW MQTT_PREFIX HOSTNAME "/HelloWorld"
#define MQTT_TOPIC_JSON MQTT_PREFIX HOSTNAME

WiFiUDP udp;
mDNSResolver::Resolver resolver(udp);

ESP8266WebServer Server;
AutoConnect Portal(Server);
AutoConnectConfig Config;
AutoConnectAux  Settings;

bool oldValue = false;

static const char Settings_Page[] PROGMEM = R"(
{
  "uri": "/settings",
  "title": "Einstellungen",
  "menu": false,
  "element": [
    {
      "name": "Tara",
      "type": "ACCheckbox",
      "label": "Tara",
      "checked": false
    },
    {
      "name": "apply",
      "type": "ACSubmit",
      "value": "&#220;bernehmen",
      "url": "/settings"
    },
    {
      "name": "AcInfos",
      "type": "ACElement",
      "value": "<a href=\"/_ac\">Infos</a>"
    }
  ]
}
)";

// the setup function runs once when you press reset or power the board
void setup()
{
    Serial.begin(115200);

    /*Settings.load(FPSTR(Settings_Page));
    Portal.join({ Settings });


    Portal.begin();*/

    // We start by connecting to a WiFi network
    Serial.print("Connecting to ");
    Serial.println(WLAN_NAME);
    WiFi.persistent(false);	//verhindert Flash Schreiben
    WiFi.setPhyMode(WIFI_PHY_MODE_11N); //braucht am wenigsten Strom
    WiFi.mode(WIFI_STA);	//Nur Client Mode
    WiFi.hostname(HOSTNAME);	//damit die Fritz.Box und so weiter den namen anzeigen
    WiFi.begin(WLAN_NAME, WLAN_PASSWORT);	//Wlan Start

    while (WiFi.status() != WL_CONNECTED) {
        //while (1) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");

    Serial.println("WiFi connected");
    Serial.print("Hostname: ");
    Serial.println(HOSTNAME);
    Serial.print("MAC: ");
    Serial.println(WiFi.macAddress());
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Gateway: ");
    Serial.println(WiFi.gatewayIP());

    IPAddress mserver;                                       // zu ermittelnde IP Adresse remote Host
    const char* mhostname = "www.google.de";                // remote Host
    Serial.println(WiFi.hostByName(mhostname, mserver));
    Serial.println(mserver);

    mhostname = "Isis";                // remote Host
    mserver = resolver.search(mhostname);
    Serial.println(mserver);

    mhostname = "Isis.local";                // remote Host
    mserver = resolver.search(mhostname);
    Serial.println(mserver);

    Serial.println("Startup fertig");
}

// the loop function runs over and over again until power down or reset
void loop()
{  
    /*Portal.handleClient();

    AutoConnectCheckbox& tara = Settings["Tara"].as<AutoConnectCheckbox>();
    if (oldValue != tara.checked)
    {
        Serial.print("changed ");
        Serial.println(tara.checked);

        oldValue = tara.checked;
    }*/



}
