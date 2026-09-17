/*
 Name:		SaltWeight.ino
 Created:	02.02.2021 14:51:24
 Author:	jhb
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
Adafruit_BME280 bme; // I2C
bool EnvSensorAktiv = false;

#include <WS2812FX.h>
#define LEDPIN 13
#define NUMPIXELS 4
WS2812FX ws2812fx = WS2812FX(NUMPIXELS, LEDPIN, NEO_GRB + NEO_KHZ800);

#include <HCSR04.h>
#define TRGPIN 14
#define ECHOPIN 12
UltraSonicDistanceSensor distanceSensor(TRGPIN, ECHOPIN, 200);
float DistanceContainerEmpty = 35;
float DistanceHysteresis = 3;

#include <ESP8266Wifi.h>
#include <ESP8266Webserver.h>
ESP8266WebServer Server;
#define HOSTNAME "SaltWeight"

#include "gitrevision.h"

#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ESP8266Ping.h>
#define JSON_BUFFER_SIZE 150
#define MQTT_BROKER "Isis"
#define MQTT_PREFIX "SPS/" HOSTNAME
#define MQTT_TOPIC_HW MQTT_PREFIX "/HelloWorld"
#define MQTT_TOPIC_JSON MQTT_PREFIX

#include <secrets.h>
#ifndef WIFI_SSID
#define WIFI_SSID "Your WiFi SSID"
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "Your WiFi PASSWORD"
#endif

char hw_buf[JSON_BUFFER_SIZE];
WiFiClient espClient;
PubSubClient MqttClient(espClient);
int PingCnt = 0;
int UpdateCnt = 0;

struct ValuesToSend
{
    float Temp;
    float Hum;
    float Pres;
    float Distance;
    int32_t WlanSignal;
    int Count;
}; 
ValuesToSend Values;

void rootPage() {
    String  content =
        "<html>"
        "<head>"
        "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
        "<title>SaltWeight</title>"        
        "</head>"
        "<body>"
        "<h2 align=\"center\" style=\"color:#{color};margin:20px;\">Salt Weight</h2>"
        "<p>Temperatur: {Temp}</p>"
        "<p>Feuchte: {Hum}</p>"
        "<p>Druck: {Pres}</p>"
        "<p>Abstand: {Distance}</p>"
        "<p>WLAN: {WLAN}</p>"
        "</body>"
        "</html>";

    char buf[15];

    sprintf(buf, "%06x", ws2812fx.getColor());
    content.replace("{color}", buf);
    sprintf(buf, "%.1f GrC", Values.Temp);
    content.replace("{Temp}", buf);
    sprintf(buf, "%.1f %%", Values.Hum);
    content.replace("{Hum}", buf);
    sprintf(buf, "%.0f hPa", Values.Pres);
    content.replace("{Pres}", buf);
    sprintf(buf, "%.3f cm", Values.Distance);
    content.replace("{Distance}", buf);
    sprintf(buf, "%d", Values.WlanSignal);
    content.replace("{WLAN}", buf);
    

    Server.send(200, "text/html", content);
}

void scan()
{
    Serial.println("Scanning I2C Addresses Channel 1");
    uint8_t cnt = 0;
    for (uint8_t i = 0; i < 128; i++) {
        Wire.beginTransmission(i);
        uint8_t ec = Wire.endTransmission(true);
        if (ec == 0) {
            if (i < 16)Serial.print('0');
            Serial.print(i, HEX);
            cnt++;
        }
        else Serial.print("..");
        Serial.print(' ');
        if ((i & 0x0f) == 0x0f)Serial.println();
    }
    Serial.print("Scan Completed, ");
    Serial.print(cnt);
    Serial.println(" I2C Devices found.");
}

void reconnect()
{    
    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.print("Attempting to connect to SSID: ");
        Serial.println(WIFI_SSID);
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        int cnt = 0;
        while (WiFi.status() != WL_CONNECTED)
        {
            delay(500);
            Serial.print(".");
            if (cnt++ > 20)
                break;
        }
        if (WiFi.status() == WL_CONNECTED)
        {
            Serial.println("WiFi connected");
            Serial.println("IP address: ");
            Serial.println(WiFi.localIP());
        }
    }
	
    if (WiFi.status() == WL_CONNECTED)
    {
        if (!MqttClient.connected())
        {
            Serial.print("Attempting MQTT connection...");
            // Attempt to connect
            MqttClient.setServer(MQTT_BROKER, 1883);
            MqttClient.connect((HOSTNAME + WiFi.macAddress()).c_str());
            Serial.println("connected");
            // Once connected, publish an announcement...
            StaticJsonDocument<JSON_BUFFER_SIZE> hw_doc;
            hw_doc["MAC"] = WiFi.macAddress();
            hw_doc["IP"] = WiFi.localIP().toString();
            hw_doc["Gateway"] = WiFi.gatewayIP().toString();
            hw_doc["git"] = gitRevShort;
            serializeJson(hw_doc, hw_buf, JSON_BUFFER_SIZE);

            MqttClient.publish(MQTT_TOPIC_HW, hw_buf);

            Values.WlanSignal = WiFi.RSSI();
        }
    }
}

// the setup function runs once when you press reset or power the board
void setup()
{
    Serial.begin(115200);    

	Wire.begin();
	scan();

    Serial.println("=== WLAN Start ===");
    reconnect();

    EnvSensorAktiv = bme.begin(0x76);
    // You can also pass in a Wire library object like &Wire2
    // status = bme.begin(0x76, &Wire2)
    if (!EnvSensorAktiv)
    {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(), 16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
    }
    else
    {

        Values.Temp = bme.readTemperature();
        Values.Hum = bme.readHumidity();
        Values.Pres = (bme.readPressure() / 100.0);

        Values.Distance =  distanceSensor.measureDistanceCm(Values.Temp);
    }

    ws2812fx.init();
    // Set the LED’s overall brightness. 0=strip off, 255=strip at full intensity
    ws2812fx.setBrightness(32);
    ws2812fx.setMode(FX_MODE_BREATH);
    ws2812fx.setColor(0, 0, 255);
    ws2812fx.start();    

    Serial.println(gitRevision);

    Serial.println("Startup fertig");
}

// the loop function runs over and over again until power down or reset
void loop()
{
    if (EnvSensorAktiv)
    {
        //float buf  = bme.readTemperature();
        Values.Temp = 0.8 * Values.Temp + 0.2 * bme.readTemperature();
        Values.Hum = 0.8 * Values.Hum + 0.2 * bme.readHumidity();
        Values.Pres = 0.8 * Values.Pres + 0.2 * (bme.readPressure() / 100.0);

        //Values.Distance = 0.8 * Values.Distance + 0.2 * distanceSensor.measureDistanceCm(buf);
        Values.Distance = 0.8 * Values.Distance + 0.2 * distanceSensor.measureDistanceCm();
    }
    else
        Values.Distance = 0.8 * Values.Distance + 0.2 * distanceSensor.measureDistanceCm();

    Serial.print("Dist: ");
    Serial.println(Values.Distance);
    if (Values.Distance < 10)
    {
        ws2812fx.setColor(0, 0, 255);   //Blau -> Fehlmessung        
    }
    else if (Values.Distance > (DistanceContainerEmpty + DistanceHysteresis))
    {
        ws2812fx.setColor(255, 0, 0);   //Rot -> zuwenig Salz drin
    }
    else if (Values.Distance < (DistanceContainerEmpty - DistanceHysteresis))
    {
        ws2812fx.setColor(0, 255, 0);   //Grün alles okay
    }

    reconnect();

    if (WiFi.status() == WL_CONNECTED)
    {        
        if (MqttClient.connected())
        {
            if(ws2812fx.getMode() != FX_MODE_BREATH)
                ws2812fx.setMode(FX_MODE_BREATH);
        }
        else
        {
            if (ws2812fx.getMode() != FX_MODE_LARSON_SCANNER)
                ws2812fx.setMode(FX_MODE_LARSON_SCANNER);
        }

        Values.WlanSignal = 0.8 * Values.WlanSignal + 0.2 * WiFi.RSSI();

		if (0 == UpdateCnt)
		{
			UpdateCnt = 10;

			StaticJsonDocument<JSON_BUFFER_SIZE> doc;
			doc["Temperatur"] = Values.Temp;
			doc["Feuchte"] = Values.Hum;
			doc["Druck"] = Values.Pres;
			doc["Abstand"] = Values.Distance;
            doc["WlanSignal"] = Values.WlanSignal;
			doc["Zaehler"] = Values.Count++;

			char buf[JSON_BUFFER_SIZE];
			serializeJson(doc, buf, JSON_BUFFER_SIZE);
#if DEBUG
			Serial.print("JSON: ");
			Serial.println(buf);
#endif // DEBUG

            if (MqttClient.connected())
			    MqttClient.publish(MQTT_TOPIC_JSON, buf);
		}
		else
			UpdateCnt--;

		if (0 == PingCnt)
		{
			PingCnt = 61;
#if DEBUG
			Serial.println("Ping");
#endif // DEBUG
			Ping.ping(WiFi.gatewayIP(), 2);
		}
		else
			PingCnt--;
    }
    else
    {
        if (ws2812fx.getMode() != FX_MODE_LARSON_SCANNER)
            ws2812fx.setMode(FX_MODE_LARSON_SCANNER);
    }

    for (int i = 0;i < 900;i++)
    {
        Server.handleClient();
        ws2812fx.service();
        MqttClient.loop();
        delay(1);
    }
}
