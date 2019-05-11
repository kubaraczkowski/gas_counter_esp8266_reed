/*
 */

#include "Arduino.h"
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

ADC_MODE(ADC_VCC);

// #define DEBUG

#ifdef DEBUG
  #define DEBUG_PRINTLN(x) Serial.println(x);
  #define DEBUG_PRINT(x) Serial.print(x);
#else
  #define DEBUG_PRINTLN(x)  ;
  #define DEBUG_PRINT(x) ;
#endif


void increment_gas_counter_in_rtc();
void read_wifi_settings_from_rtc();
void wifi_connect();
void write_wifi_settings_to_rtc();
void connect_to_mqtt();
void sub_callback(const char* topic, byte* payload, unsigned int length);
uint32_t calculateCRC32( const uint8_t *data, size_t length );

// Use PLATFORMIO_BUILD_FLAGS to pass the variables to the build env
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASS;

// The ESP8266 RTC memory is arranged into blocks of 4 bytes. The access methods read and write 4 bytes at a time,
// so the RTC data structure should be padded to a 4-byte multiple.
struct {
  uint32_t crc32;   // 4 bytes
  uint8_t channel;  // 1 byte,   5 in total
  uint8_t ap_mac[6]; // 6 bytes, 11 in total
  uint8_t padding;  // 1 byte,  12 in total
} rtcData;

// save counter_value in rtc. important - value should nominally be false
struct {
  uint32_t crc32;      // 4 bytes
  uint32_t value;      // 4 bytes,   8 in total
  uint8_t valid;       // 1 byte,    9 in total
  uint8_t padding[3];  // 1 byte,   12 in total
} gas_counter;


IPAddress staticIP(192, 168, 88, 102); //ESP static ip
IPAddress gateway(192, 168, 88, 1);   //IP Address of your WiFi Router (Gateway)
IPAddress subnet(255, 255, 255, 0);  //Subnet mask
IPAddress dns(8, 8, 8, 8);  //DNS
 
const char* deviceName = "esp8266_gas_counter";

const char* mqttServer = MQTT_SERVER_IP;
const int mqttPort = 1883;
const char* mqttUser = MQTT_SERVER_USER;
const char* mqttPassword = MQTT_PASS;

WiFiClient espClient;
PubSubClient client(espClient);

unsigned long StartTime;
unsigned long ConnectedTime;

bool rtcValid = false;
bool connected_mqtt = false;
bool counter_valid = false;
int retries = 0;

char payload [12] = {0};

void setup()
{

  #ifdef DEBUG
    Serial.begin(115200);
  #endif

  StartTime = millis(); 

  increment_gas_counter_in_rtc();
  read_wifi_settings_from_rtc();
  wifi_connect();
  write_wifi_settings_to_rtc();

  connect_to_mqtt();

  if (connected_mqtt) {
    DEBUG_PRINTLN("Subscribing to new counter value");
    client.subscribe("esp/gas_counter/new_counter_value");

    DEBUG_PRINTLN("Publishing counter value");
    sprintf(payload, "%u", gas_counter.value);
    client.publish("esp/gas_counter", payload, strlen(payload));

    ConnectedTime = millis();

    sprintf(payload, "%lu", ConnectedTime - StartTime);
    client.publish("esp/gas_counter/startup_time", payload, strlen(payload));

    sprintf(payload, "%u", ESP.getVcc());
    client.publish("esp/gas_counter/vdd", payload, strlen(payload));

    if (!counter_valid) {
      // we should wait (reasonable time) for an updated value of the counter
      DEBUG_PRINT("Waiting for new value");
      retries = 200;
      while ( (retries-->0) && (counter_valid == false) ) {
        client.loop();
        delay(10);
        DEBUG_PRINT(".");
      }
      DEBUG_PRINTLN("");
    }
    else {
      // counter is valid, all good, let's wait a moment to make sure data propagated and go to sleep
      // the length of delay below is quite sensitive - adjust as needed
      delay(80); // wait a tiny bit to push the data to the mqtt server // TODO: what could be a better way to see this finished?
    }

    ConnectedTime = millis();
    #ifdef DEBUG
      Serial.print("Connection time: ");
      Serial.println(ConnectedTime - StartTime);
      Serial.println(payload);
    #endif
  }

  #ifdef DEBUG
    Serial.flush();
  #endif
  ESP.deepSleep(0);

}

void loop()
{

  // should never get here. if we do - go deep sleep again
  ESP.deepSleep(0);
}


void increment_gas_counter_in_rtc() {
  // even before any WIFI action, we increment the gas_counter_value stored in the RTC memory
  // first 12 bytes are for wifi settings, we'll use offset 12 for our counter
  ESP.rtcUserMemoryRead( 12, (uint32_t*)&gas_counter, 12 );
  uint32_t crc = calculateCRC32( ((uint8_t*)&gas_counter) + 4, sizeof( rtcData ) - 4 );
  if (crc == gas_counter.crc32) {
    counter_valid = true;
    gas_counter.value = gas_counter.value + 1;
    gas_counter.crc32 = calculateCRC32( ((uint8_t*)&gas_counter) + 4, sizeof( rtcData ) - 4 );
    DEBUG_PRINTLN("Counter in RTC is valid");
  }
  else {
    counter_valid = false;
    DEBUG_PRINTLN("Counter in RTC is invalid");
  }
  DEBUG_PRINTLN("RTC gas counter value:");
  DEBUG_PRINTLN(gas_counter.value);
  ESP.rtcUserMemoryWrite( 12, (uint32_t*)&gas_counter, 12);
}

void read_wifi_settings_from_rtc() {
  // Try to read WiFi settings from RTC memory
  if( ESP.rtcUserMemoryRead( 0, (uint32_t*)&rtcData, sizeof( rtcData ) ) ) {
    // Calculate the CRC of what we just read from RTC memory, but skip the first 4 bytes as that's the checksum itself.
    uint32_t crc = calculateCRC32( ((uint8_t*)&rtcData) + 4, sizeof( rtcData ) - 4 );
    if( crc == rtcData.crc32 ) {
      rtcValid = true;
    }
  }
}


void wifi_connect() {
  WiFi.hostname(deviceName);      // DHCP Hostname (useful for finding device for static lease)
  WiFi.config(staticIP, subnet, gateway, dns);

  if( rtcValid ) {
    // The RTC data was good, try making a quick connection
    WiFi.begin( ssid, password, rtcData.channel, rtcData.ap_mac, true );
  }
  else {
    // The RTC data was not valid, so make a regular connection
    WiFi.begin( ssid, password );
  }

  WiFi.mode(WIFI_STA); //WiFi mode station (connect to wifi router only
  
  int wifiStatus = WiFi.status();

  while( wifiStatus != WL_CONNECTED ) {
    retries++;
    if( retries == 100 ) {
      // Quick connect is not working, reset WiFi and try regular connection
      WiFi.disconnect();
      delay( 10 );
      WiFi.forceSleepBegin();
      delay( 10 );
      WiFi.forceSleepWake();
      delay( 10 );
      WiFi.begin( ssid, password);
      rtcValid = false;
    }
    if( retries == 600 ) {
      // Giving up after 30 seconds and going back to sleep
      WiFi.disconnect( true );
      delay( 1 );
      WiFi.mode( WIFI_OFF );
      ESP.deepSleep(0); // go to sleep forever. wake up at next reset
    }
    delay( 50 );
    wifiStatus = WiFi.status();
  }
  
  DEBUG_PRINTLN("Connected to the WiFi network");
}


void write_wifi_settings_to_rtc() {
  // Write current connection info back to RTC
  if (rtcValid == false) {
    rtcData.channel = WiFi.channel();
    memcpy( rtcData.ap_mac, WiFi.BSSID(), 6 ); // Copy 6 bytes of BSSID (AP's MAC address)
    rtcData.crc32 = calculateCRC32( ((uint8_t*)&rtcData) + 4, sizeof( rtcData ) - 4 );
    ESP.rtcUserMemoryWrite( 0, (uint32_t*)&rtcData, sizeof( rtcData ) );

    DEBUG_PRINTLN("Saved wifi data to RTC");
  }
}


void connect_to_mqtt() {
  // connect to mqtt and publish our gas counter
  client.setServer(mqttServer, mqttPort);
  client.setCallback(sub_callback);

  retries = 0;
  while (!connected_mqtt) {
    DEBUG_PRINTLN("Connecting to MQTT...");
 
    if (client.connect("ESP8266Client", mqttUser, mqttPassword )) {
      DEBUG_PRINTLN("connected");  
    } else {
        DEBUG_PRINT("failed with state ");
        DEBUG_PRINTLN(client.state());
      delay(20);
    }
    retries++;

    if (retries == 100) {
      // abandon action. well pity, we couldn't connect
      break;
    }
    connected_mqtt = client.connected();
  }

}

void sub_callback(const char* topic, byte* payload, unsigned int length) {
  char counter_string[10] = {0};

  DEBUG_PRINTLN("Received message on: ");
  DEBUG_PRINTLN(topic);

  if (strcmp(topic, "esp/gas_counter/new_counter_value") == 0) {
    memcpy(counter_string, payload, length);
    uint32_t counter_value = strtoul(counter_string, NULL, 10);

    DEBUG_PRINT("New counter value: ");
    DEBUG_PRINTLN(counter_value);

    // write the new value to RTC memory
    gas_counter.value = counter_value;
    gas_counter.valid = 1;
    gas_counter.crc32 = calculateCRC32( ((uint8_t*)&gas_counter) + 4, sizeof( rtcData ) - 4 );
    counter_valid = true;
    ESP.rtcUserMemoryWrite( 12, (uint32_t*)&gas_counter, 12);
  }

}


uint32_t calculateCRC32( const uint8_t *data, size_t length ) {
  uint32_t crc = 0xffffffff;
  while( length-- ) {
    uint8_t c = *data++;
    for( uint32_t i = 0x80; i > 0; i >>= 1 ) {
      bool bit = crc & 0x80000000;
      if( c & i ) {
        bit = !bit;
      }

      crc <<= 1;
      if( bit ) {
        crc ^= 0x04c11db7;
      }
    }
  }

  return crc;
}