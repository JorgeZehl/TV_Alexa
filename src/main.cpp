#include <Arduino.h>
#ifdef ESP32
    #include <WiFi.h>
#else
    #include <ESP8266WiFi.h>
#endif
#include "fauxmoESP.h"

// Rename the credentials.sample.h file to credentials.h and 
// edit it according to your router configuration
#include "secrets.h"  // Include the secrets header file


fauxmoESP fauxmo;

// -----------------------------------------------------------------------------

#define SERIAL_BAUDRATE     115200


#define ID_ON_OFF_TV           "La Tele"
#define ID_VOLUM            "Volumen de la tele"
#define ID_HDMI             "ArduinoHDMI"

// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Wifi
// -----------------------------------------------------------------------------


// IRMP Library
/*
 * Set library modifiers first to set output pin etc.
 */
#include "PinDefinitionsAndMore.h"
//#define IR_OUTPUT_IS_ACTIVE_LOW
#define IRSND_IR_FREQUENCY          38000

#define IRSND_PROTOCOL_NAMES        1 // Enable protocol number mapping to protocol strings - requires some FLASH.

#include <irsndSelectMain15Protocols.h>
// or use only one protocol to save programming space
//#define IRSND_SUPPORT_NEC_PROTOCOL        1

/*
 * After setting the definitions we can include the code and compile it.
 */
#include <irsnd.hpp>


union WordUnion
{
    struct
    {
        uint8_t LowByte;
        uint8_t HighByte;
    } UByte;
    struct
    {
        int8_t LowByte;
        int8_t HighByte;
    } Byte;
    uint8_t UBytes[2];
    int8_t Bytes[2];
    uint16_t UWord;
    int16_t Word;
    uint8_t *BytePointer;
};

IRMP_DATA irsnd_data;



void wifiSetup() {

    // Set WIFI module to STA mode
    WiFi.mode(WIFI_STA);

    // Connect
    Serial.printf("[WIFI] Connecting to %s ", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    // Wait
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(100);
    }
    Serial.println();

    // Connected!
    Serial.printf("[WIFI] STATION Mode, SSID: %s, IP address: %s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());

}

void setup() {

    // Init serial port and clean garbage
    Serial.begin(SERIAL_BAUDRATE);

#if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || defined(SERIAL_USB) || defined(SERIALUSB_PID) || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif
#if defined(ESP8266)
    Serial.println(); // to separate it from the internal boot output
#endif

    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRMP));

    irsnd_init();
    irmp_irsnd_LEDFeedback(true); // Enable send signal feedback at LED_BUILTIN

#if defined(ARDUINO_ARCH_STM32)
    Serial.println(F("Ready to send IR signals at pin " IRSND_OUTPUT_PIN_STRING)); // the internal pin numbers are crazy for the STM32 Boards library
#else
    Serial.println(F("Ready to send IR signals at pin " STR(IRSND_OUTPUT_PIN)));
#endif
    /*
     * Send Samsung32
     */
    irsnd_data.protocol = IRMP_SAMSUNG32_PROTOCOL;
    irsnd_data.address = 0x0707;
    irsnd_data.flags = 0; // repeat frame 0 time
    Serial.println();
    Serial.println();

    // Wifi
    wifiSetup();

    // By default, fauxmoESP creates it's own webserver on the defined port
    // The TCP port must be 80 for gen3 devices (default is 1901)
    // This has to be done before the call to enable()
    fauxmo.createServer(true); // not needed, this is the default value
    fauxmo.setPort(80); // This is required for gen3 devices

    // You have to call enable(true) once you have a WiFi connection
    // You can enable or disable the library at any moment
    // Disabling it will prevent the devices from being discovered and switched
    fauxmo.enable(true);

    // You can use different ways to invoke alexa to modify the devices state:
    // "Alexa, turn yellow lamp on"
    // "Alexa, turn on yellow lamp
    // "Alexa, set yellow lamp to fifty" (50 means 50% of brightness, note, this example does not use this functionality)

    // Add virtual devices
    fauxmo.addDevice(ID_ON_OFF_TV);
    fauxmo.addDevice(ID_VOLUM);
    fauxmo.addDevice(ID_HDMI);


    fauxmo.onSetState([](unsigned char device_id, const char * device_name, bool state, unsigned char value) {
        
        // Callback when a command from Alexa is received. 
        // You can use device_id or device_name to choose the element to perform an action onto (relay, LED,...)
        // State is a boolean (ON/OFF) and value a number from 0 to 255 (if you say "set kitchen light to 50%" you will receive a 128 here).
        // Just remember not to delay too much here, this is a callback, exit as soon as possible.
        // If you have to do something more involved here set a flag and process it in your main loop.
        
        Serial.printf("[MAIN] Device #%d (%s) state: %s value: %d\n", device_id, device_name, state ? "ON" : "OFF", value);

        // Checking for device_id is simpler if you are certain about the order they are loaded and it does not change.
        // Otherwise comparing the device_name is safer.

        if (strcmp(device_name, ID_ON_OFF_TV)==0) {
            if(state == HIGH)
            {

            }
            else(state == LOW)
            {

            }
            digitalWrite(LED_YELLOW, state ? HIGH : LOW);
        } else if (strcmp(device_name, ID_VOLUM)==0) {
            digitalWrite(LED_GREEN, state ? HIGH : LOW);
        } else if (strcmp(device_name, ID_HDMI)==0) {
            digitalWrite(LED_BLUE, state ? HIGH : LOW);
        } 

    });

}

void loop() {

    // fauxmoESP uses an async TCP server but a sync UDP server
    // Therefore, we have to manually poll for UDP packets
    fauxmo.handle();

    // This is a sample code to output free heap every 5 seconds
    // This is a cheap way to detect memory leaks
    static unsigned long last = millis();
    if (millis() - last > 5000) {
        last = millis();
        Serial.printf("[MAIN] Free heap: %d bytes\n", ESP.getFreeHeap());
    }

    // If your device state is changed by any other means (MQTT, physical button,...)
    // you can instruct the library to report the new state to Alexa on next request:
    // fauxmo.setState(ID_YELLOW, true, 255);

}