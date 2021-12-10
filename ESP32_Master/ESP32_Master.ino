#include "esp_wpa2.h"
#include <ArduinoJson.h>
#include <ThingsBoard.h> // ThingsBoard SDK
#include <WiFi.h>        // WiFi control for ESP32
#include <Wire.h>

// Helper macro to calculate array size
#define COUNT_OF(x) ((sizeof(x) / sizeof(0 [x])) / ((size_t)(!(sizeof(x) % sizeof(0 [x])))))

// Define Slave I2C Address. All slaves must be allocated a
// unique number.
#define SLAVE_ADDR 9
#define BYTE_SIZE 16

#define EAP_IDENTITY "username@ucl.ac.uk"
#define EAP_PASSWORD "CSEEE17"

#define TOKEN "ESP32ANG"
#define THINGSBOARD_SERVER "demo.thingsboard.io"

// Define the pins used for SDA and SCL. This is important because
// there is a problem with the TTGO and I2C will not work properly
// unless you do.
#define I2C_SDA 21
#define I2C_SCL 22

const char *essid = "eduroam";

WiFiClient espClient;        // Initialize ThingsBoard client
ThingsBoard tb(espClient);   // Initialize ThingsBoard instance
int status = WL_IDLE_STATUS; // the Wifi radio's status

// Set to true if application is subscribed for the RPC messages
bool subscribed = false;

float curPH = 6.6f;
float curTemperature = 30.0f;
int curRPM = 750;

RPC_Response processGetRPM(const RPC_Data &data)
{
    //    Serial.println("RPM get value method");

    return RPC_Response(NULL, curRPM);
}

RPC_Response processSetTemperature(const RPC_Data &data)
{
    //    Serial.println("Temperature set value method");

    curTemperature = data;

    //    Serial.print("Set new Temperature: ");
    //    Serial.println(curTemperature);
    char buf[BYTE_SIZE];
    String Temp_to_send = "3" + String(curTemperature, 1);
    Temp_to_send.toCharArray(buf, BYTE_SIZE);
    Wire.beginTransmission(SLAVE_ADDR);
    Wire.write(buf);
    Wire.endTransmission();

    return RPC_Response(NULL, curTemperature);
}

RPC_Response processSetPH(const RPC_Data &data)
{
    //    Serial.println("PH set value method");

    curPH = data;

    //    Serial.print("Set new PH: ");
    //    Serial.println(curPH);
    char buf[BYTE_SIZE];
    String PH_to_send = "4" + String(curPH, 1);
    PH_to_send.toCharArray(buf, BYTE_SIZE);
    Wire.beginTransmission(SLAVE_ADDR);
    Wire.write(buf);
    Wire.endTransmission();

    return RPC_Response(NULL, curPH);
}

RPC_Response processGetPH(const RPC_Data &data)
{
    //    Serial.println("PH get value method");

    return RPC_Response(NULL, curPH);
}

RPC_Response processSetRPM(const RPC_Data &data)
{
    //    Serial.println("RPM set value method");

    curRPM = data;

    //    Serial.print("Set new RPM: ");
    //    Serial.println(curRPM);
    char buf[BYTE_SIZE];
    String RPM_to_send = "5" + String(curRPM);
    RPM_to_send.toCharArray(buf, BYTE_SIZE);
    Wire.beginTransmission(SLAVE_ADDR);
    Wire.write(buf);
    Wire.endTransmission();

    return RPC_Response(NULL, curRPM);
}

RPC_Response processGetTemperature(const RPC_Data &data)
{
    //    Serial.println("Temperature get value method");

    return RPC_Response(NULL, curTemperature);
}

// RPC handlers
RPC_Callback callbacks[] = {
    {"ph_setValue", processSetPH},
    {"ph_getValue", processGetPH},
    {"rpm_setValue", processSetRPM},
    {"rpm_getValue", processGetRPM},
    {"temperature_setValue", processSetTemperature},
    {"temperature_getValue", processGetTemperature},
};

void parseAndSendData(String text)
{
    char code = text[0];
    String sValue = text.substring(1);
    if (code == '0')
    {
        float value = sValue.toFloat();
        tb.sendTelemetryFloat("temperature", value);
    }
    else if (code == '1')
    {
        float value = sValue.toFloat();
        tb.sendTelemetryFloat("pH", value);
    }
    else if (code == '2')
    {
        int value = sValue.toInt();
        tb.sendTelemetryInt("RPM", value);
    }
    else if (code == '6')
    {
        bool value = sValue.toInt() ? true : false;
        tb.sendAttributeBool("status_temperature", value);
    }
    else if (code == '7')
    {
        bool value = sValue.toInt() ? true : false;
        tb.sendAttributeBool("status_pH", value);
    }
    else if (code == '8')
    {
        bool value = sValue.toInt() ? true : false;
        tb.sendAttributeBool("status_RPM", value);
    }
}

void connectWiFi()
{
    bool eduroamFound = false;
    // Scan available WiFi networks until eduroam is seen
    // Set WiFi to station mode and disconnect from an AP if it was previously connected
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // Repeatedly scan until we see eduroam
    while (!eduroamFound)
    {
        Serial.println("scan start");
        int n = WiFi.scanNetworks(); // WiFi.scanNetworks returns the number of networks found
        Serial.println("scan done");

        if (n == 0)
        {
            Serial.println("no networks found");
        }
        else
        {
            Serial.print(n);
            Serial.println(" networks found");

            for (int i = 0; i < n; ++i)
            {
                String ssid = WiFi.SSID(i);
                int rssi = WiFi.RSSI(i);

                // Print SSID and RSSI for each network found
                Serial.print(i + 1);
                Serial.print(": ");
                Serial.print(ssid);
                Serial.print(" (");
                Serial.print(rssi);
                Serial.print(")");
                Serial.print((WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? " " : "*");
                delay(10);

                ssid.trim();
                if (ssid == essid)
                {
                    Serial.print(" <==== eduroam found");
                    eduroamFound = true;
                }
                Serial.println("");
            }
        }
        Serial.println("");

        // Wait a bit before scanning again
        if (!eduroamFound)
            delay(5000);
    }

    // If we come here, we've successfully found eduroam. Try to connect to it.
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(essid);

    // Set WiFi to station mode and disconnect from an AP if it was previously connected
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // This is where the wpa2 magic happens to allow us to connect to eduroam
    esp_wifi_sta_wpa2_ent_set_username((uint8_t *)EAP_IDENTITY, strlen(EAP_IDENTITY));
    esp_wifi_sta_wpa2_ent_set_password((uint8_t *)EAP_PASSWORD, strlen(EAP_PASSWORD));
    esp_wifi_sta_wpa2_ent_enable();

    WiFi.begin(essid);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.print("WiFi is connected to ");
    Serial.println(essid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}

void setup()
{
    Wire.begin(I2C_SDA, I2C_SCL);
    Serial.begin(115200);
    connectWiFi();
}

void loop()
{
    // Reconnect to WiFi, if needed
    if (WiFi.status() != WL_CONNECTED)
    {
        connectWiFi();
        return;
    }

    // Reconnect to ThingsBoard, if needed
    if (!tb.connected())
    {
        Serial.print("Connecting to: ");
        Serial.print(THINGSBOARD_SERVER);
        Serial.print(" with token ");
        Serial.println(TOKEN);
        if (!tb.connect(THINGSBOARD_SERVER, TOKEN))
        {
            Serial.println("Failed to connect");
            return;
        }
    }

    // Subscribe for RPC, if needed
    if (!subscribed)
    {
        Serial.println("Subscribing for RPC...");

        // Perform a subscription. All consequent data processing will happen in
        // callbacks as denoted by callbacks[] array.
        if (!tb.RPC_Subscribe(callbacks, COUNT_OF(callbacks)))
        {
            Serial.println("Failed to subscribe for RPC");
            return;
        }

        Serial.println("Subscribe done");
        subscribed = true;
    }

    //    Wire.requestFrom(SLAVE_ADDR, BYTE_SIZE);
    Wire.requestFrom(SLAVE_ADDR, (BYTE_SIZE + 1) * 6);
    // device SLAVE_ADDR
    String received_string = "";
    while (Wire.available())
    {
        char c = Wire.read();
        if (c == '\n')
        {
            //   Serial.println(received_string);
            parseAndSendData(received_string);
            received_string = "";
        }
        else
        {
            received_string += c;
        }
    }

    tb.loop();
    delay(1000);
}