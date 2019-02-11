/**
 * Build options: Genergic ESP32 Dev Module; 240MHz; 4M (Default partition).
 */
#include "WiFiParameters.h"
#include "DeviceParameters.h"

extern "C" int rom_phy_get_vdd33();
#include <rom/rtc.h>
#include "esp_system.h"

#include <PubSubClient.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <Update.h>
#include <WiFi.h>

#include <BME280_MOD-1022.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <Wire.h>

#include <TimeLib.h>

#define VERSION        "esp32-0.2.0-2"

#define MAX_OW_DEVICES 10

#define PWM_CHANNEL    0
#define PWM_RESOLUTION 9
#define PWM_MAX_VALUE  512
#define PWM_MAX_FREQ   32768

#define D0             26

#define D1             22
#define D2             21
#define D3             17
#define D4             16

#define D5             18
#define D6             19
#define D7             23
#define D8             5

#define PIN_WAKE_UP    D0
#define PIN_POWER_ADC  36

/**
 * Pin layout.
 */
volatile int PIN_SDA     = D1;
volatile int PIN_SCL     = D2;
volatile int PIN_DS20B18 = D3; // Need 1k pull-up
volatile int PIN_SWITCH  = D4; // Need 100 pull-up

volatile int PIN_PIR     = D5;
volatile int PIN_RELAY   = D6;
volatile int PIN_POWER   = D7;
volatile int PIN_PWM     = D8;

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

/**
 * Reset reason string and UUID of the node.
 */
String resetReason = "unknown";
String MAC_ADDRESS;
String DEVICE_ID;
String DEVICE_KEY;

/**
 * State of the PWM.
 */
volatile int pwmEnabled = 0;

volatile int pwmBrightnessLookup[PWM_MAX_VALUE + 1];
volatile int pwmMaxPower = PWM_MAX_VALUE;
volatile int pwmCurrentPower = 0;
volatile int pwmTargetPower = -1;
volatile int pwmPrevTargetPower = -1;

/**
 * State of the PWM and the switch.
 */
volatile int switchEnabled = 0;

volatile int switchState = 1;
volatile int switchSavedState = 1;
volatile int switchOldState = 1;
volatile int switchDirection = 0;

volatile unsigned long switchLastChanged = millis();
volatile unsigned long switchLastReceived = millis();

/**
 * State of PIR sensor.
 */
volatile int pirEnabled = 0;
volatile int pirState = 0;
volatile unsigned long pirLastChange = 0;
volatile unsigned long pirSwitchDuration = 300000;

/**
 * Relay state.
 */
int relayEnabled = 0;
volatile unsigned long relayState = LOW;

/**
 * State of environmental sensors.
 */
volatile unsigned long sensorLastSent = 0;
volatile unsigned long sensorSendDuration = 60000;

volatile double temp;
volatile double humidity;
volatile double pressure;

/**
 * DS20B18 sensor
 */
OneWire oneWireBus(PIN_DS20B18);
DallasTemperature sensors;

/**
 * NTP synchronization.
 */
WiFiUDP udp;

long ntpTimestampOffset = 0;
volatile unsigned long ntpSendDuration = 600000;
volatile unsigned long ntpLastSent = millis();

/**
 * Update check.
 */
WiFiClient updateClient;
volatile unsigned long updateLastChecked = 0;
volatile unsigned long updateCheckDuration = 60000;

/**
 * Start web server on the port 80 and start a client.
 */
WebServer server(80);

/**
 * The IoT Guru MQTT constants.
 */
const char* mqttUsername = "mqttReader";
const char* mqttPassword = "mqttReader";
const char* mqttServer = "mqtt.iotguru.live";
volatile unsigned long mqttLastConnected = 0;
volatile unsigned long mqttReconnectDuration = 5000;
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

/**
 * PWM updater timer method.
 */
void IRAM_ATTR updatePWM() {

    if (pwmEnabled) {
        portENTER_CRITICAL_ISR(&timerMux);
        if (pwmTargetPower > pwmCurrentPower) {
            pwmCurrentPower += 1;
        }
        if (pwmTargetPower < pwmCurrentPower) {
            pwmCurrentPower -= 1;
        }

        if (pwmCurrentPower <= 0) {
            ledcWrite(PWM_CHANNEL, 0);
        } else if (pwmCurrentPower >= PWM_MAX_VALUE) {
            ledcWrite(PWM_CHANNEL, PWM_MAX_VALUE);
        } else {
            ledcWrite(PWM_CHANNEL, pwmCurrentPower);
        }
        portEXIT_CRITICAL_ISR(&timerMux);  

        if (pwmTargetPower != -1) {
            if (pirState == 1) {
              pwmTargetPower = pwmPrevTargetPower;
            } else {
                if (pirLastChange + pirSwitchDuration < millis() && pwmTargetPower > pwmPrevTargetPower / 4 * 3) {
                    pwmPrevTargetPower = pwmTargetPower;
                    pwmTargetPower = pwmPrevTargetPower / 4 * 3;
                } else if (pirLastChange + pirSwitchDuration * 2 < millis()) {
                  pwmTargetPower = -1;
                  pwmPrevTargetPower = -1;
                  switchDirection = 0;
                  digitalWrite(PIN_RELAY, LOW);
                }
            }
        }

        switchState = digitalRead(PIN_SWITCH);
        if (switchState == switchSavedState && switchLastReceived > millis()) {
            return;
        }

        if (switchState != switchSavedState) {
            switchLastReceived = millis() + 50;
            switchSavedState = switchState;
            return;
        }

        if (switchState == switchOldState) {
            return;
        }
        switchOldState = switchState;

        if (switchState == LOW) {
            switchLastChanged = millis();
        }

        if (switchState == HIGH) {
            if (switchLastReceived - switchLastChanged > 250) {
                pirLastChange = millis();
                pwmTargetPower = pwmCurrentPower;
                pwmPrevTargetPower = pwmCurrentPower;
                if (switchLastReceived - switchLastChanged > 5000) {
                    pirLastChange = millis();
                    if (relayEnabled) {
                        digitalWrite(PIN_RELAY, HIGH);
                    }
                }
            }
            
            return;
        }

        if (switchDirection == 0) {
            pirLastChange = millis();

            pwmTargetPower = pwmMaxPower + 1;
            pwmPrevTargetPower = pwmMaxPower + 1;
            switchDirection = 1;
        } else {
            pirLastChange = millis();
            pwmTargetPower = -1;
            pwmPrevTargetPower = -1;
            switchDirection = 0;
            if (relayEnabled) {
                digitalWrite(PIN_RELAY, LOW);
            }
        }
    }
}

void setup(){
    /**
     * Clearing the serial line.
     */
    Serial.begin(115200);
    while (!Serial) {
        delay(1);
    }

    Serial.println();
    for (int i = 0; i < 64; i++) {
        Serial.print(".");
        delay(1);
    }
    Serial.println();
    Serial.println();

    /**
     * Query the unique id from MAC address.
     */
    byte mac[6]; esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char macString[13]; sprintf(macString, "%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    MAC_ADDRESS = String(macString);
    Serial.println("ESP8266 MAC = " + MAC_ADDRESS);
    Serial.println();

    /**
     * Query runtime parameters from the static parameter map.
     */
    for(int i = 0; i < NUMBER_OF_DEVICES; i++) {
        if (MAC_ADDRESS.equals(deviceParametersMap[i].name)) {
            DEVICE_ID = deviceParametersMap[i].deviceId;
            DEVICE_KEY = deviceParametersMap[i].deviceKey;

            pwmEnabled = deviceParametersMap[i].pwmEnabled;
            relayEnabled = deviceParametersMap[i].relayEnabled;
        }
    }

    if (pwmEnabled) {
        ledcSetup(PWM_CHANNEL, PWM_MAX_FREQ, PWM_RESOLUTION);
        ledcAttachPin(PIN_PWM, PWM_CHANNEL);
        ledcWrite(PWM_CHANNEL, 0);
    }

    wifiConnect();

    /**
     *  Prints the version number and the device parameters.
     */
    Serial.println();
    Serial.println("Version [" + String(VERSION) + "] is starting...");
    Serial.println();
    Serial.println("PWM enabled [" + DEVICE_ID + "]: " + String(pwmEnabled));
    Serial.println("Relay enabled [" + DEVICE_ID + "]: " + String(relayEnabled));
    Serial.println();

    /**
     * Do HTTP update.
     */
    doHttpUpdate();

    /**
     * Examine reset reason for CPUs.
     */
    Serial.println();
    printResetReason(1); printResetReason(0);
    Serial.println();

    /**
     * Calculate the non-linear brightness of PWM output.
     */
    float R = (PWM_MAX_VALUE * log10(2)) / log10(PWM_MAX_VALUE);
    for (int i = 0; i <= PWM_MAX_VALUE; i++) {
        int brightness = pow(2, (i / R));
        pwmBrightnessLookup[i] = brightness - 1;
    }
    pwmBrightnessLookup[PWM_MAX_VALUE - 1] = PWM_MAX_VALUE;
    pwmBrightnessLookup[PWM_MAX_VALUE] = PWM_MAX_VALUE;

    /**
     * Set PIN modes and attach interrupts.
     */
    pinMode(PIN_RELAY, OUTPUT);  // Relay
    pinMode(PIN_SDA, INPUT);     // BME280 SDA
    pinMode(PIN_SWITCH, INPUT);  // Switch
    pinMode(PIN_PIR, INPUT);     // PIR
    pinMode(PIN_SCL, INPUT);     // BME280 SCL
    pinMode(PIN_POWER, OUTPUT);  // BME280 power
    
    digitalWrite(PIN_POWER, HIGH);
    attachInterrupt(PIN_PIR, handlePirPin, CHANGE);

    hw_timer_t * timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &updatePWM, true);
    timerAlarmWrite(timer, 5000, true);
    timerAlarmEnable(timer);

    /**
     * Initialize DS20B18 sensors communication.
     */
    oneWireBus = OneWire(PIN_DS20B18);
    sensors = DallasTemperature(&oneWireBus);
    sensors.begin();
}

/**
 * Reset reason.
 */
void printResetReason(int cpu) {
    int reason = rtc_get_reset_reason(cpu);
    switch (reason) {
        case 1:  resetReason = "Power on reset"; break;
        case 3:  resetReason = "Software reset digital core"; break;
        case 4:  resetReason = "Legacy watch dog reset digital core"; break;
        case 5:  resetReason = "Deep Sleep reset digital core"; break;
        case 6:  resetReason = "Reset by SLC module, reset digital core"; break;
        case 7:  resetReason = "Timer Group0 Watch dog reset digital core"; break;
        case 8:  resetReason = "Timer Group1 Watch dog reset digital core"; break;
        case 9:  resetReason = "RTC Watch dog Reset digital core"; break;
        case 10: resetReason = "Instrusion tested to reset CPU"; break;
        case 11: resetReason = "Time Group reset CPU"; break;
        case 12: resetReason = "Software reset CPU"; break;
        case 13: resetReason = "RTC Watch dog Reset CPU"; break;
        case 14: resetReason = "for APP CPU, reseted by PRO CPU"; break;
        case 15: resetReason = "Reset when the vdd voltage is not stable"; break;
        case 16: resetReason = "RTC Watch dog reset digital core and rtc module"; break;
        default: resetReason = "UNKNOWN: " + String(reason);
    }
    Serial.println("Reset reason for CPU " + String(cpu) + ": " + resetReason);
}

/**
 * Connect to the WiFi.
 */
void wifiConnect() {
    if (WiFi.status() == WL_CONNECTED) {
        return;
    }
  
    if (WiFi.status() != WL_CONNECTED) {
        wifiConnect(AP0_SSID, AP0_PASSWORD);
    }

    if (WiFi.status() != WL_CONNECTED) {
        wifiConnect(AP1_SSID, AP1_PASSWORD);
    }

    if (WiFi.status() != WL_CONNECTED) {
        wifiConnect(AP2_SSID, AP2_PASSWORD);
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Cannot connect to WiFi SSIDs...");
        return;
    }

    Serial.print("Use this URL to connect: ");
    Serial.print("http://");
    Serial.print(WiFi.localIP());
    Serial.println("/");
    Serial.println();
    Serial.println();
}

/**
 * Connect to the WiFi.
 */
void wifiConnect(char* ssid, char* password) {
    WiFi.mode(WIFI_STA);
    WiFi.setAutoConnect(true);
    WiFi.setAutoReconnect(true);
    
    long start = micros();
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi [ssid=" + String(ssid) + "]: ");

    /**
     * Wait 10 seconds to connect...
     */
    int counter = 10000;
    while (WiFi.status() != WL_CONNECTED && counter > 0) {
        delay(1); counter--;
        if (counter % 100 == 0) {
            Serial.print(".");
        }
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println(" connected (in " + String((micros()-start)/1000) + " ms)");

        /**
         * Starting UDP port for NTP.
         */
        Serial.print("Starting UDP for NTP... local port: ");
        udp.begin(2390);
        Serial.println(udp.remotePort());

        setSyncProvider(timeProvider);
        setSyncInterval(10);

        setupWebServer();
        
        return;
    }

    WiFi.disconnect();
    Serial.println(" cannot connect.");
}

/**
 * Periodically check things... :)
 */
void loop(){
    wifiConnect();
    handleSensors();
    handleNtpPacket();
    server.handleClient();    
    handleMQTT();
    checkForUpdate();
    
    delay(1);
}

/**
 * Handle the PIR sensor output.
 */
void handlePirPin() {
    if (pwmEnabled) {
        int pin = digitalRead(PIN_PIR);

        if (pin == HIGH && pirState == LOW) {
            pirLastChange = millis();
            pirState = HIGH;
        } else if (pin == LOW && pirState == HIGH) {
            pirLastChange = millis();
            pirState = LOW;
        }
    }
}

/**
 * Handle the sensor.
 */
void handleSensors() {
    if (sensorLastSent == 0 || sensorLastSent + sensorSendDuration < millis()) {
        sensorLastSent = millis();
    } else {
        return;
    }

    Serial.println("");
    Serial.println("");
    Serial.println("Handle sensors");
    String vcc = String(analogRead(PIN_POWER_ADC));
    Serial.println("VCC measurement: " + vcc);
  
    HTTPClient http;
    http.useHTTP10(true);
    http.setTimeout(8000);

    /**
     * TODO: report the VCC via DEVICE_ID.
     */
    String NODE_KEY = nodeKeyByDeviceId();
    if (String("not-found-node-key") != NODE_KEY) {
        http.begin("http://" + String(IOT_BASE_HOST) + "/measurement/create/" + NODE_KEY + "/vcc/" + vcc);
        Serial.println("Status code of 'vcc': " + String(http.GET()));
        http.end();
    }

    /**
     * Initiate the DS20B18 sensors.
     */
    DeviceAddress deviceAddresses[MAX_OW_DEVICES];
    int devices = discoverOneWireDevices(deviceAddresses, MAX_OW_DEVICES);

    sensors.requestTemperatures();
    for (int i = 0; i < devices; i++) {
        sendTemperature(deviceAddresses[i], vcc);
    }


    /**
     * Initiate the BME280 sensor with power.
     */
    digitalWrite(PIN_POWER, HIGH);
    delay(5);

    /**
     * Start measurements.
     */
    Wire.begin(PIN_SDA, PIN_SCL);
    uint8_t chipID = BME280.readChipId();
    Serial.print("ChipID = 0x");
    Serial.println(chipID, HEX);

    if (chipID != 0x60) {
        digitalWrite(PIN_POWER, LOW);
        return;
    }

    BME280.readCompensationParams();
    BME280.writeOversamplingPressure(os16x);
    BME280.writeOversamplingTemperature(os16x);
    BME280.writeOversamplingHumidity(os16x);
    BME280.writeMode(smForced);

    int result = repeatedBMERead();
    BME280.writeMode(smNormal);
    digitalWrite(PIN_POWER, LOW);

    if (result == 0) {
        Serial.println("Cannot read valid measurements...");
        return;
    }

    Serial.println("Temperature:  " + String(temp));
    http.begin("http://" + String(IOT_BASE_HOST) + "/measurement/create/" + NODE_KEY + "/temperature/" + String(temp));
    Serial.println("Status code of 'temperature': " + String(http.GET()));
    http.end();

    Serial.println("Humidity:     " + String(humidity));
    http.begin("http://" + String(IOT_BASE_HOST) + "/measurement/create/" + NODE_KEY + "/pressure/" + String(pressure));
    Serial.println("Status code of 'pressure': " + String(http.GET()));
    http.end();

    Serial.println("Pressure:     " + String(pressure));
    http.begin("http://" + String(IOT_BASE_HOST) + "/measurement/create/" + NODE_KEY + "/humidity/" + String(humidity));
    Serial.println("Status code of 'humidity': " + String(http.GET()));
    http.end();
}


/**
 * Repeated BME read.
 */
int repeatedBMERead() {
    for (int i = 0; i < 10; i++) {
        Serial.print("Measuring...");

        int measuring;
        do {
            Serial.print(".");delay(10);
            measuring = BME280.isMeasuring();
        }
        while (measuring);

        Serial.print(" reading... ");
        BME280.readMeasurements();

        temp = BME280.getTemperature();
        humidity = BME280.getHumidity();
        pressure = BME280.getPressure();

        BME280.writeStandbyTime(tsb_0p5ms);
        BME280.writeFilterCoefficient(fc_16);

        if (pressure > 800) {
            Serial.println("done.");
            return 1;
        }

        Serial.println("failed. Repeating (" + String(i + 1) + "/10).");
        delay(10);
    }

    return 0;
}

/**
 * Send temperature of the specified DS18B20 device.
 */
void sendTemperature(DeviceAddress address, String vcc) {
    temp = getTemperature(address);

    HTTPClient http;
    http.useHTTP10(true);
    http.setTimeout(8000);

    String NODE_KEY = nodeKeyByAddress(addressToString(address));
    http.begin("http://" + String(IOT_BASE_HOST) + "/measurement/create/" + NODE_KEY + "/vcc/" + vcc);
    Serial.println("Status code of 'vcc': " + String(http.GET()));
    http.end();

    http.begin("http://" + String(IOT_BASE_HOST) + "/measurement/create/" + NODE_KEY + "/temperature/" + String(temp));
    Serial.println("Status code of 'temperature': " + String(http.GET()));
    http.end();
}

/**
 * Returns with the temperature of the Dallas unit.
 *
 * @param address the address
 * @return the temperature
 */
float getTemperature(DeviceAddress address) {
    int counter = 0;
    do {
        temp = sensors.getTempC(address);
        delay((counter++) * 10);
    } while ((temp == 85.0 || temp == (-127.0)) && counter < 10);    
    Serial.println("Temperature of '" + addressToString(address) + "': " + String(temp));

    return temp;
}

/**
 * Returns nodeKey by deviceId.
 */
String nodeKeyByDeviceId() {
    for(int i = 0; i < NUMBER_OF_NODES; i++) {
        if (MAC_ADDRESS.equals(nodeParametersMap[i].name)) {
          return nodeParametersMap[i].nodeKey;
        }
    }

    return "not-found-node-key";
}

/**
 * Returns nodeKey by address.
 */
String nodeKeyByAddress(String address) {
    for(int i = 0; i < NUMBER_OF_NODES; i++) {
        if (address.equals(nodeParametersMap[i].name)) {
          return nodeParametersMap[i].nodeKey;
        }
    }

    return "not-found-node-key";
}

/**
 * Discover DS18B20 devices.
 */
int discoverOneWireDevices(DeviceAddress deviceAddresses[], int maxDevices) {
    byte addr[8];
    int count = 0;
    while (count < maxDevices && oneWireBus.search(addr)) {
        for(int i = 0; i < 8; i++) {
            deviceAddresses[count][i] = addr[i];
        }
        if (OneWire::crc8(addr, 7) != addr[7]) {
            Serial.println("CRC is not valid, aborting...");
            return 0;
        }

        Serial.println("Found 1-Wire device with address: " + addressToString(addr));
        count++;
    }

    if (count == 0) {
        Serial.println("There is no 1-Wire devices...");
    }

    oneWireBus.reset_search();

    return count;
}

/**
 * Convert DS1820's address to String.
 */
String addressToString(DeviceAddress address) {
    String result = "";
    for (int i = 0; i < 8; i++) {
        if (address[i] < 16) {
            result += "0";
        }

        result += String(address[i], HEX);
    }

    return result;
}

/**
 * Return with UNIX timestamp.
 */
long unixTimestamp() {
    if (ntpTimestampOffset == 0) {
        return 0;
    }

    return millis() / 1000 + ntpTimestampOffset;
}

/**
 * Time provider of Time library.
 */
time_t timeProvider() {
    return unixTimestamp();
}

/**
 * .Handle the NTP packet.
 */
void handleNtpPacket() {
    byte packetBuffer[48];
    unsigned long sendDuration = (ntpTimestampOffset == 0) ? 1000 : ntpSendDuration;
    if (ntpLastSent + sendDuration < millis()) {
        ntpLastSent = millis();

        const char* ntpServerName = "0.hu.pool.ntp.org";
        IPAddress timeServerIP;
        WiFi.hostByName(ntpServerName, timeServerIP);

        memset(packetBuffer, 0, 48);
        packetBuffer[0] = 0b11100011;
        packetBuffer[1] = 0;
        packetBuffer[2] = 6;
        packetBuffer[3] = 0xEC;
        packetBuffer[12]  = 49;
        packetBuffer[13]  = 0x4E;
        packetBuffer[14]  = 49;
        packetBuffer[15]  = 52;

        udp.beginPacket(timeServerIP, 123);
        udp.write(packetBuffer, 48);
        udp.endPacket();
        Serial.println("NTP request sent");

        return;
    }

    if (udp.parsePacket() != 48) {
        return;
    }

    udp.read(packetBuffer, 48);
    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    unsigned long milliseconds = packetBuffer[39] * 1000L / 256;

    ntpTimestampOffset = secsSince1900 - 2208988800UL - millis() / 1000;
    Serial.println("NTP response received, the timestamp is " + String(millis() / 1000 + ntpTimestampOffset));
}

/**
 * Set up the web server.
 */
void setupWebServer() {
    server.on("/", []() {
        Serial.println("Server status");
        server.send(200, "text/plain",
            "Server MAC is '"+ MAC_ADDRESS +"', last reset reason was '" + resetReason + "'.\n" +
            "Version [" + String(VERSION) + "].\n" +
            "Current uptime (timestamp): " + String(millis()) + " ms\n" +
            "Current NTP (timestamp): " + String(unixTimestamp()) + "\n" +
            "Local date and time: " + String(year()) + "-" + String(month()) + "-" + String(day()) + " " +
                String(hour()) + ":" + String(minute()) + ":" + String(second()) + "\n\n" +

            "PWM enabled: " + String(pwmEnabled) + "\n" +
            "Relay enabled: " + String(relayEnabled) + "\n" +

            "PWM: " + String(pwmCurrentPower) + "\n" +
            "PWM target power: " + String(pwmTargetPower) + "\n" +
            "PWM previous target power: " + String(pwmPrevTargetPower) + "\n" +
            "PWM maximum power: " + String(pwmMaxPower) + "\n\n" +

            "Switch state: " + String(switchSavedState) + "\n" +
            "Switch direction: " + String(switchDirection) + "\n" +
            "Switch last timestamp: " + String(switchLastReceived) + "\n\n" +

            "Sensor last sent: " + String(sensorLastSent) + "\n" +
            "Temp: " + String(temp) + "\n" +
            "Humidity: " + String(humidity) + "\n" +
            "Pressure: " + String(pressure) + "\n\n" +

            "PIR state: " + String(pirState) + "\n" +
            "PIR last change: " + String(pirLastChange) + "\n\n" +

            "Relay state: " + String(relayState) + "\n" +
            "");          
    });

    /**
     * Set the maximum PWM power.
     */
    server.on("/pwm", []() {
        String power = server.arg("power");
        int intPower = power.toInt();
        if (intPower >= 0 && intPower <= PWM_MAX_VALUE) {
            pwmMaxPower = intPower;
        }

        Serial.println("Maximum power of the PWM is " + String(power));
        server.send(200, "text/plain", "Maximum power of the PWM is " + String(pwmMaxPower));
    });

    /**
     * Toggle the PWM.
     */
    server.on("/toggle", []() {
        if (pwmTargetPower <= pwmMaxPower) {
            pirLastChange = millis();
            pwmTargetPower = pwmMaxPower + 1;
            pwmPrevTargetPower = pwmMaxPower + 1;
            Serial.println("PWM ON");
        } else {
            pwmTargetPower = -1;
            pwmPrevTargetPower = -1;
            Serial.println("PWM OFF");
        }

        Serial.println("The target power of the PWM is " + String(pwmTargetPower));
        server.send(200, "text/plain", "The target power of the PWM is " + String(pwmTargetPower));
    });


    /**
     * Start the HTTP server on the port 80.
     */
    server.begin();
    Serial.println("HTTP server started...");
    Serial.println();
}

/**
 * Connect and reconnect to the MQTT server.
 */
void mqttConnect() {
    if (mqttClient.connected()) {
       return;
    }

    Serial.println("MQTT clientId: " + MAC_ADDRESS);
    mqttClient.setServer(mqttServer, 1883);
    mqttClient.setCallback(mqttCallback);

    if (mqttClient.connect(MAC_ADDRESS.c_str(), mqttUsername, mqttPassword)) {      
        String nodeKey = nodeKeyByDeviceId();
        String field = "led";
        String topic = String("sub/" + nodeKey + "/" + field);
      
        mqttClient.subscribe(topic.c_str());
      
        Serial.println("");
        Serial.print("Connected to the MQTT server and subscribed to the '");Serial.print(topic);Serial.println("' topic.");
        Serial.println("");
    } else {
        Serial.print("MQTT connection failed, rc=");Serial.print(mqttClient.state());Serial.println(" try again later...");
    }
}

/**
 * Handle MQTT communication.
 */
void handleMQTT() {
    mqttClient.loop();
    
    if (mqttLastConnected == 0 || mqttLastConnected + mqttReconnectDuration < millis()) {
        mqttLastConnected = millis();
    } else {
        return;
    }

    mqttConnect();
}

/**
 * MQTT message callback.
 */
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message arrived [");Serial.print(topic);Serial.print("] ");
    char payloadChars[length + 1];
    for (int i = 0; i < length; i++) {
        payloadChars[i] = (char)payload[i];
        Serial.print((char)payload[i]);
    }
    payloadChars[length] = '\0';
    Serial.println();
    
    String payloadString = String(payloadChars);
    pwmTargetPower = payloadString.toInt();
    pirLastChange = millis();
}

/**
 * Try to update the firmware.
 */
void doHttpUpdate() {
    String updateUrl = "/firmware/update/" + String(DEVICE_KEY) + "/" + String(VERSION);

    int contentLength = 0;
    bool isValidContentType = false;
    if (updateClient.connect(IOT_BASE_HOST, 80)) {
        updateClient.print(String("GET ") + updateUrl + " HTTP/1.1\r\n" +
                "Host: " + IOT_BASE_HOST + "\r\n" +
                "Cache-Control: no-cache\r\n" +
                "Connection: close\r\n\r\n");

        unsigned long timeout = millis();
        while (updateClient.available() == 0) {
            delay(1);
            if (millis() - timeout > 5000) {
                Serial.println("Client Timeout!");
                updateClient.stop();
                return;
            }
        }

        while (updateClient.available()) {
            String line = updateClient.readStringUntil('\n');
            line.trim();

            if (!line.length()) {
                break;
            }

            if (line.startsWith("HTTP/1.1")) {
                if (line.indexOf("200") < 0) {
                    Serial.println("Got a non 200 status code from server (" + line + "). Exiting OTA Update.");
                    break;
                }
            }

            if (line.startsWith("Content-Length: ")) {
                contentLength = atoi((getHeaderValue(line, "Content-Length: ")).c_str());
                Serial.println("Got " + String(contentLength) + " bytes from server");
            }

            if (line.startsWith("Content-Type: ")) {
                String contentType = getHeaderValue(line, "Content-Type: ");
                Serial.println("Got " + contentType + " payload.");
                if (contentType == "application/octet-stream; charset=UTF-8") {
                    isValidContentType = true;
                }
            }
        }
    } else {
        Serial.println("Connection to " + String(IOT_BASE_HOST) + " failed. Please check your setup");
    }

    if (!contentLength || !isValidContentType) {
        updateClient.stop();
        return;
    }
    
    bool canBegin = Update.begin(contentLength);
    if (!canBegin) {
        Serial.println("There is no enough space to begin OTA... giving up.");
        updateClient.stop();
        return;
    }

    Serial.println("Begin OTA update...");
    size_t written = Update.writeStream(updateClient);
    if (written == contentLength) {
        Serial.println("Written: " + String(written) + " successfully.");
    } else {
        Serial.println("Written only: " + String(written) + "/" + String(contentLength) + "...");
    }

    if (Update.end()) {
        Serial.println("OTA process ended...");
        if (Update.isFinished()) {
            Serial.println("Update successfully completed. Rebooting.");
            ESP.restart();
        } else {
            Serial.println("Update not finished? Something went wrong! Uh-oh...");
        }
    } else {
        Serial.println("Error Occurred. Error code is " + String(Update.getError()));
    }
}

/**
 * Check for update on the specified server.
 */
void checkForUpdate() {
    if (updateLastChecked == 0 || updateLastChecked + updateCheckDuration < millis()) {
        updateLastChecked = millis();
    } else {
        return;
    }

    HTTPClient http;
    http.useHTTP10(true);
    http.setTimeout(8000);

    http.begin("http://" + String(IOT_BASE_HOST) + "/firmware/check/" + DEVICE_KEY + "/" + String(VERSION) + "/" + WiFi.localIP().toString());
    int code = http.GET();
    http.end();

    Serial.println();
    Serial.println("Checking for new firmware: " + String(code));

    if (code == 200) {
        ESP.restart();
    }
}

/**
 * Returns with the value of the header.
 */
String getHeaderValue(String header, String headerName) {
    return header.substring(strlen(headerName.c_str()));
}
