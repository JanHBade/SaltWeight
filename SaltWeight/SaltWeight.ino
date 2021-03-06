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

#include <HX711_ADC.h>
//pins:
const int HX711_dout = 12; //mcu > HX711 dout pin
const int HX711_sck = 14; //mcu > HX711 sck pin
//HX711 constructor:
HX711_ADC LoadCell(HX711_dout, HX711_sck);
bool LoadCellAktiv = false;
float LoadCellOffset = 0.0;

#include <WS2812FX.h>
#define LEDPIN 13
#define NUMPIXELS 4
WS2812FX ws2812fx = WS2812FX(NUMPIXELS, LEDPIN, NEO_GRB + NEO_KHZ800);

#include <ESP8266Wifi.h>
#include <ESP8266Webserver.h>
#include <AutoConnect.h>
#define HOSTNAME "SaltWeight"

ESP8266WebServer Server;
AutoConnect Portal(Server);
AutoConnectConfig Config;
AutoConnectAux  Settings;

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
      "name": "Offset",
      "type": "ACInput",
      "label": "Offset" 
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

#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ESP8266Ping.h>
#define JSON_BUFFER_SIZE 150
#define MQTT_BROKER "Isis"
#define MQTT_PREFIX "SPS/" HOSTNAME
#define MQTT_TOPIC_HW MQTT_PREFIX "/HelloWorld"
#define MQTT_TOPIC_JSON MQTT_PREFIX

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
    float Weight;
    int32_t WlanSignal;
    int Count;
}; 
ValuesToSend Values;

void rootPage() {
    String  content =
        "<html>"
        "<head>"
        "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"        
        "</head>"
        "<body>"
        "<h2 align=\"center\" style=\"color:#{color};margin:20px;\">Salt Weight</h2>"
        "<p>Temperatur: {Temp}</p>"
        "<p>Feuchte: {Hum}</p>"
        "<p>Druck: {Pres}</p>"
        "<p>Gewicht: {Weight}</p>"
        "<p>WLAN: {WLAN}</p>"
        "<p></p><p style=\"padding-top:15px;text-align:center\">" AUTOCONNECT_LINK(COG_24) "</p>"
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
    sprintf(buf, "%.2f kg", Values.Weight);
    content.replace("{Weight}", buf);
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
	Serial.print("Attempting MQTT connection...");
	// Attempt to connect
	if (MqttClient.connect(HOSTNAME))
	{
		Serial.println("connected");
		// Once connected, publish an announcement...
		StaticJsonDocument<JSON_BUFFER_SIZE> hw_doc;
		hw_doc["MAC"] = WiFi.macAddress();
		hw_doc["IP"] = WiFi.localIP().toString();
		hw_doc["Gateway"] = WiFi.gatewayIP().toString();
		serializeJson(hw_doc, hw_buf, JSON_BUFFER_SIZE);

		MqttClient.publish(MQTT_TOPIC_HW, hw_buf);

		Values.WlanSignal = WiFi.RSSI();
	}
	else
	{
		Serial.print("failed, rc=");
		Serial.print(MqttClient.state());
	}
}

// the setup function runs once when you press reset or power the board
void setup()
{
    Serial.begin(115200);

	Wire.begin();
	scan();

    
    // default settings
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

    LoadCell.begin();
    float calibrationValue; // calibration value (see example file "Calibration.ino")
    calibrationValue = 22901.36; // uncomment this if you want to set the calibration value in the sketch

    unsigned long stabilizingtime = 2000; // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
    boolean _tare = false; //set this to false if you don't want tare to be performed in the next step
    LoadCell.start(stabilizingtime, _tare);
    if (LoadCell.getTareTimeoutFlag())
    {
        Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    }
    else
    {
        LoadCellAktiv = true;
        LoadCell.setCalFactor(calibrationValue); // set calibration value (float)
    }

    ws2812fx.init();
    ws2812fx.setBrightness(255);
    ws2812fx.setSpeed(5);
    ws2812fx.setMode(FX_MODE_BREATH);
    ws2812fx.setColor(0, 0, 255);
    ws2812fx.start();

    Settings.load(FPSTR(Settings_Page));
    Portal.join({ Settings });
    Server.on("/", rootPage);
    
    Config.hostName = HOSTNAME;
    Config.immediateStart = false;	//kein Wifi Verbindungsversuch
    Config.portalTimeout = 100;  // It will time out in 0,1 second
    Config.retainPortal = true;		//Portal bleibt vorhanden
    Config.apid = "Weight-12345678";
    Config.ota = AC_OTA_BUILTIN;	//Update Server
    Config.homeUri = "/settings";
    Config.bootUri = AC_ONBOOTURI_HOME;
    Portal.config(Config);
    Portal.begin();

    Serial.println("Startup fertig");
}

// the loop function runs over and over again until power down or reset
void loop()
{
    if (EnvSensorAktiv)
    {
        Values.Temp = bme.readTemperature();
        Values.Hum = bme.readHumidity();
        Values.Pres = bme.readPressure() / 100.0;
    }

    if (LoadCellAktiv)
    {
        Values.Weight = LoadCell.getData()+LoadCellOffset;

        AutoConnectCheckbox& tara = Settings["Tara"].as<AutoConnectCheckbox>();
        AutoConnectInput& Offset = Settings["Offset"].as<AutoConnectInput>();
        if (tara.enable && tara.checked)
        {
            Serial.println("TARA");
            LoadCell.tare();
            tara.checked = false;

            LoadCellOffset = Offset.value.toFloat();
        }
    }

    if (Values.Weight < 5)
    {
        ws2812fx.setColor(255, 0, 0);
    }
    else if (Values.Weight > 100)
    {
        ws2812fx.setColor(0, 0, 255);
    }
    else
    {
        ws2812fx.setColor(0, 255, 0);
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        if (!MqttClient.connected())
        {
            Serial.print("MQTT Server: ");
            IPAddress ip;
            Serial.println(WiFi.hostByName(MQTT_BROKER, ip));
            Serial.println(ip);
            MqttClient.setServer(ip, 1883);
            reconnect();
        }

        Values.WlanSignal = 0.8 * Values.WlanSignal + 0.2 * WiFi.RSSI();

		if (0 == UpdateCnt)
		{
			UpdateCnt = 10;

			StaticJsonDocument<JSON_BUFFER_SIZE> doc;
			doc["Temperatur"] = Values.Temp;
			doc["Feuchte"] = Values.Hum;
			doc["Druck"] = Values.Pres;
			doc["Gewicht"] = Values.Weight;
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

    for (int i = 0;i < 900;i++)
    {
        Portal.handleClient();
        ws2812fx.service();
        LoadCell.update();
        delay(1);
    }
}
