// D1M-WX1-ThingSpeak-OTA-WFM.ino
const int    FW_VERSION  = 1007;
const String FW_FILENAME = "D1M-WX1-ThingSpeak-OTA-WFM";

// v1008 2020/06/15 beta release
// v1007 2020/06/12 add drd, change to claws BH1750 library
// v1006 2020/06/12 same as v1005 to force OTA prior to adding drd
// v1005 2020/06/12 make connection subs return bool
// v1004 2020/06/11 expanded json doc capacity, rtc speedup, fixed alarm check
// v1003 uses tzapu WiFiManager
// v1002 2020-05-09 Offset RTC to avoid conflict with OTA
// v1001 2020-05-08 WiFiManager working, ThingSpeak working, OTA working

/*_____________________________________________________________________________
   Copyright(c) 2018-2020 Karl W. Berger dba IoT Kits https://w4krl.com/iot-kits

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in all
   copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
   SOFTWARE.
   _____________________________________________________________________________
*/

// *******************************************************
// ******************* INCLUDES **************************
// *******************************************************

// ESP8266 core version 2.7.1
// Arduino IDE version 1.8.12
// For WiFiManager
#include <FS.h>                       // [builtin] SPIFFS file system for custom parameters
#include <ESP8266WebServer.h>         // [builtin] for captive portal
#include <WiFiManager.h>              // [manager] v0.15.0 by tzapu https://github.com/tzapu/WiFiManager
#include <ESP8266WiFi.h>              // [builtin] Wi-Fi
#include <ArduinoJson.h>              // [manager] v6.15.2 by BenoÃ®t Blanchon https://github.com/bblanchon/ArduinoJson
#include <DoubleResetDetector.h>      // [manager] v1.0.3 by Stephen Denne https://github.com/datacute/DoubleResetDetector

// For general sketch
#include <Wire.h>                     // [builtin] I2C bus
#include <BME280I2C.h>                // [ZIP]     v2.3.0 by Tyler Glenn https://github.com/finitespace/BME280
#include <BH1750.h>                   // [builtin] v1.1.4 by Christopher Laws https://github.com/claws/BH1750

// For HTTP OTA
#include <ESP8266HTTPClient.h>        // [builtin] http
#include <WiFiClientSecureBearSSL.h>  // [builtin] https
#include <ESP8266httpUpdate.h>        // [builtin] OTA

// *******************************************************
// ******* Set true to erase config data *****************
// *******************************************************
// Use only if all else fails.
// If your device is confused or constantly rebooting:
// 1. Set RESET_WIFI = true;
// 2. Upload the firmware:
//    - DO NOT DO ANYTHING ELSE
//    - DO NOT CONFIGURE YOUR DEVICE
//    - Let it run ONCE
// 3. Change RESET_WFI = false; and reupload.
// 4. Now you can do the normal configuration process.
const boolean RESET_WIFI = false;     // erases WiFI & SPIFFS settings

// *******************************************************
// ******************* DEFAULTS **************************
// *******************************************************
//           CHANGEABLE DEFAULTS
const float  DMM_VOLTAGE = 1.0;  // voltage displayed on your digital multimeter
const float  ADC_VOLTAGE = 1.0;  // voltage reported by the Analog to Digital Converter
const long   SLEEP_INTERVAL = 1 * 60;    // must be 15 seconds or more
const int    OTA_SPAN = 30 * 60; // seconds between OTA checks
const long   MIN_RSSI = -85;     // warning level for low WiFi signal strength
const float  MIN_VCELL = 3.0;    // warning level for low cell voltage

//#define DEBUG

//           DO NOT CHANGE THESE DEFAULTS
const String THINGSPEAK_SERVER = "api.thingspeak.com";    // ThingSpeak Server
const char   AP_PORTAL_NAME[] = "IoT Kits";
const String CONFIG_FILENAME = "/config.json";            // SPIFFS filename
const int    OTA_OFFSET = 32;                             // OTA uses first 128 bytes

// *******************************************************
// ******************* GLOBALS ***************************
// *******************************************************
bool saveConfigFlag = false;
String unitStatus = "";          // holds device error messages
long startTime = millis();       // record time at start of sketch
const float HPA_TO_INHG = 0.0295299830714;  // hPa (mb) to inHg pressure
bool rtcValid = false;           // RTC check of validity

String wifiSSID;
String wifiPASS;

// user parameter values entered through WiFiManager
char wm_ts_channel[10];          // ThingSpeak channel ID
char wm_ts_write_key[20];        // ThingSpeak API Write Key
char wm_elevation[10];           // elevation in meters

// structure to hold sensor measurements & calculated values
struct
{
  float stationPressure;         // station pressure (hPa) (mb)
  float seaLevelPressure;        // calculated SLP (hPa)
  float celsius;                 // temperature (Â°C)
  float fahrenheit;              // calculated temperature (Â°F)
  float humidity;                // relative humidity (%)
  float lightLevel;              // light intensity (lux)
  float cellVoltage;             // LiPo cell volts
  long  wifiRSSI;                // WiFi signal strength (dBm)
} sensorData;

// The ESP8266 Real Time Clock memory is arranged into blocks of 4 bytes.
// The RTC data structure MUST be padded to a 4-byte multiple.
// Maximum 512 bytes available.
// https://arduino-esp8266.readthedocs.io/en/latest/libraries.html#esp-specific-apis
// Use fixed width types to avoid variations between devices, for example,
// int is two bytes in Arduino UNO and four bytes in ESP8266.
struct
{
  uint32_t  crc32;               // 4 bytes    4 total
  uint8_t   wifiChannel;         // 1 byte,    5 total
  uint8_t   bssid[6];            // 6 bytes,  11 total
  uint8_t   sequence;            // 1 byte,   12 total
  bool      bme280Fail;          // 1 byte,   13 total
  bool      bh1750Fail;          // 1 byte,   14 total
  bool      lowVcell;            // 1 byte,   15 total
  bool      lowRSSI;             // 1 byte,   16 total
  float     timeAwake;           // 4 bytes,  20 total (5 blocks)
  //  uint8_t   pad[x];              // x bytes,  xx total
} rtcData;

// *******************************************************
// ********* INSTANTIATE DEVICES *************************
// *******************************************************
BME280I2C myBME280;              // barometric pressure / temperature / humidity sensor
BH1750 myBH1750;                 // light level sensor
WiFiClient client;               // ThingSpeak
WiFiManager myWFM;               // WiFiManager
DoubleResetDetector drd( 10, OTA_OFFSET + (sizeof(rtcData) / 4) );

// *******************************************************
// ******************** SETUP ****************************
// *******************************************************
void setup()
{
  Wire.begin();                  // required for BME280 library
  Serial.begin( 115200 );
  Serial.println();

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  WiFi.mode( WIFI_STA );        // ESP defaults to AP+STA
  
  bool configFlag = false;
  if ( drd.detectDoubleReset() )
  {
    configFlag = true;
    Serial.println(F("User requests config portal." ));
  }
  // erase user config data - used ONLY for testing
  if ( RESET_WIFI )
  {
    SPIFFS.format();
    myWFM.resetSettings();
  }

  readRTCmemory();               // get sequence number & WiFi channel info

  // read configuration from FS json
  readConfig();

  wifiSSID = myWFM.getWiFiSSID();
  wifiPASS = myWFM.getWiFiPass();
  if ( wifiSSID == "" )
  {
    configFlag = true;
    Serial.println("No stored credentials");
  }

#ifdef DEBUG
  Serial.print( wifiSSID ); Serial.print("/"); Serial.println( wifiPASS );
  myWFM.setDebugOutput(true);
#else
  myWFM.setDebugOutput(false);
#endif

  if ( configFlag )
  {
    if ( !openConfigPortal() )
    {
      enterSleep( SLEEP_INTERVAL );
    }
  }
  else
  {
    Serial.println("Try logon with stored credentials");
    if ( !logonToRouter() )
    {
      Serial.println("Normal connection failed");
      if ( !openConfigPortal() )
      {
        enterSleep( SLEEP_INTERVAL );
      }
    }
  }
  drd.stop();

  // ************** Weather Station Program **************************

  readSensors();                 // read data into sensorData struct

  printToSerialPort();           // display data on local serial monitor

  // periodically check for an OTA
  // Note: if an OTA update is performed, program will be reset
  if ( rtcData.sequence % ( OTA_SPAN / SLEEP_INTERVAL ) == 0 )
  {
    checkOTAupdate();            // check for OTA update
  }

  postToThingSpeak();            // send data to ThingSpeak

  // update the sequence number - rollover at 255
  rtcData.sequence = rtcData.sequence + 1;

  writeRTCmemory();              // save sequence & sensor status

  enterSleep( SLEEP_INTERVAL );  // go to low power sleep mode
} //setup()

// *******************************************************
// ******************** LOOP *****************************
// *******************************************************
void loop()
{
  // everything is done in setup()
} // loop()


// *******************************************************
// ******************* readSensors ***********************
// *******************************************************
void readSensors()
{
  if ( myBME280.begin() == true )          // device is OK
  {
    // read pressure, temperature and humidity in one command
    myBME280.read( sensorData.stationPressure, sensorData.celsius,
                   sensorData.humidity, BME280::TempUnit_Celsius, BME280::PresUnit_hPa );

    // calculate fahrenheit
    sensorData.fahrenheit = 1.8 * sensorData.celsius + 32.0;

    // calculate the Sea Level Pressure from the station pressure and temperature
    sensorData.seaLevelPressure = calculateSeaLevelPressure( sensorData.celsius,
                                  sensorData.stationPressure, atof( wm_elevation ) );

    if ( rtcData.bme280Fail == true )     // device was failed
    {
      unitStatus += "BME280 cleared. ";
      rtcData.bme280Fail = false;         // now cleared
    }
  }
  else                                    // device is failed
  {
    if ( rtcData.bme280Fail == false )    // device was OK
    {
      rtcData.bme280Fail = true;
      unitStatus += "BME280 failed. ";    // now failed
    }
  }

  // initialize BH1750 light sensor
  if (   myBH1750.begin(BH1750::ONE_TIME_HIGH_RES_MODE))  // device is ok
  {
    // read light level in lux
    sensorData.lightLevel = myBH1750.readLightLevel();

    if ( rtcData.bh1750Fail == true )     // device was failed
    {
      unitStatus += "BH1750 cleared. ";
      rtcData.bh1750Fail = false;         // now ok
    }
  }
  else                                    // device is failed
  {
    if ( rtcData.bh1750Fail == false )    // device was good
    {
      unitStatus += "BH1750 failed. ";
      rtcData.bh1750Fail = true;          // now failed
    }
  }

  // read analog voltage from the Analog to Digital Converter
  // on D1 Mini this is 0 - 1023 for voltages 0 to 3.2V
  // the D1M-WX1 has an external resistor to extend the range to 5.0 Volts
  // a fudgeFactor corrects for voltage divider component variation
  // as measured by the user in the calbration step

  float fudgeFactor = DMM_VOLTAGE / ADC_VOLTAGE;
  sensorData.cellVoltage = 5.0 * analogRead( A0 ) * fudgeFactor / 1023.0;
  if ( sensorData.cellVoltage > MIN_VCELL ) // unit is OK
  {
    if ( rtcData.lowVcell == true )         // was failed
    {
      unitStatus += "Vcell cleared. ";
      rtcData.lowVcell = false;             // now cleared
    }
  }
  else                                      // unit is bad
  {
    if ( rtcData.lowVcell == false )        // was good
    {
      unitStatus += "Vcell low. ";
      rtcData.lowVcell = true;              // now failed
    }
  }

  // read WiFi Received Signal Strength Indicator (RSSI)
  sensorData.wifiRSSI = WiFi.RSSI();
  if ( sensorData.wifiRSSI > MIN_RSSI )   // device is OK
  {
    if ( rtcData.lowRSSI == true )        // device was failed
    {
      unitStatus += "RSSI cleared. ";
      rtcData.lowRSSI = false;            // now cleared
    }
  }
  else                                    // device is failed
  {
    if ( rtcData.lowRSSI == false )       // device was good
    {
      unitStatus += "RSSI low. ";
      rtcData.lowRSSI = true;            // now failed
    }
  }
} // readSensors()

// *******************************************************
// ******************* readRTCmemory *********************
// *******************************************************
/* RTC Memory Functions: The ESP8266 internal Real Time Clock has unused memory
  that remains active during the Deep Sleep mode. This sketch stores WiFi connection
  information in RTC memory to speed up connection time.
*/
void readRTCmemory()
{
  rtcValid = false;
  // offset data 32 bytes to avoid OTA area
  if ( ESP.rtcUserMemoryRead( OTA_OFFSET, (uint32_t*)&rtcData, sizeof( rtcData ) ) )
  {
    // Calculate the CRC of what we just read from RTC memory,
    // but skip the first 4 bytes as that's the checksum itself.
    uint32_t crc = calculateCRC32(((uint8_t*)&rtcData ) + 4, sizeof( rtcData ) - 4 );
    if ( crc == rtcData.crc32 )
    {
      rtcValid = true;
      rtcData.timeAwake = constrain( rtcData.timeAwake, 0, 90 );
    }
  }
} // readRTCmemory()

// *******************************************************
// ****************** writeRTCmemory *********************
// *******************************************************
void writeRTCmemory()
{
  rtcData.wifiChannel = WiFi.channel();         // WiFi channel
  // memcpy explanation: http://arduino.land/FAQ/content/6/30/en/how-to-copy-an-array.html
  memcpy( rtcData.bssid, WiFi.BSSID(), 6 ); // Copy 6 bytes of BSSID (AP's MAC address)
  rtcData.timeAwake  = ( millis() - startTime ) / 1000.0;  // total awake time in seconds
  rtcData.crc32      = calculateCRC32( ((uint8_t*)&rtcData) + 4, sizeof( rtcData ) - 4 );

  // offset data 32 bytes to avoid OTA area
  ESP.rtcUserMemoryWrite( OTA_OFFSET, (uint32_t*)&rtcData, sizeof( rtcData ) );
} // writeRTCmemory()

// *******************************************************
// ***************** calculateCRC32 **********************
// *******************************************************
// Cribbed from Bakke. Originated by others.
uint32_t calculateCRC32( const uint8_t *data, size_t length )
{
  uint32_t crc = 0xffffffff;
  while ( length-- )
  {
    uint8_t c = *data++;
    for ( uint32_t i = 0x80; i > 0; i >>= 1 )
    {
      bool bit = crc & 0x80000000;
      if ( c & i )
      {
        bit = !bit;
      }
      crc <<= 1;
      if ( bit )
      {
        crc ^= 0x04c11db7;
      }
    }
  }
  return crc;
} // calculateCRC32()

// *******************************************************
// **************** checkOTAupdate ***********************
// *******************************************************
void checkOTAupdate()
{
  const String FW_URL_BASE = "https://w4krl.com/fota/";
  const String FW_PATH = FW_FILENAME + "/";
  const String FW_VERSION_URL = FW_URL_BASE + FW_PATH + FW_FILENAME + ".version";
  const String FW_IMAGE_URL   = FW_URL_BASE + FW_PATH + FW_FILENAME + ".ino.d1_mini.bin";

  std::unique_ptr<BearSSL::WiFiClientSecure>client(new BearSSL::WiFiClientSecure);
  client->setInsecure();  // doesn't need fingerprint!!!!

  HTTPClient https;

  if ( https.begin( *client, FW_VERSION_URL ) )
  {
    unitStatus += "OTA check @ seq. " + String(rtcData.sequence) + " ";
    // start connection and send HTTP header
    int httpCode = https.GET();
    Serial.printf("[HTTPS] GET code: %d\n", httpCode);
    if ( httpCode > 0 )
    {
      // HTTP header has been sent and Server response header has been handled
      if ( httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY )
      {
        String newFWVersion = https.getString();
        int newVersion = newFWVersion.toInt();
        Serial.print(F("Using version: ")); Serial.println( FW_VERSION );
        Serial.print(F("Found version: ")); Serial.println( newVersion );
        if ( newVersion > FW_VERSION )
        {
          unitStatus += " Update to version " + newFWVersion + ".";
          Serial.println(F("Updating bin: "));
          Serial.println( FW_IMAGE_URL );

          // report status to ThingSpeak, update sequence and save to RTC
          postToThingSpeak();
          rtcData.sequence++;
          writeRTCmemory();

          ESPhttpUpdate.update( *client, FW_IMAGE_URL );  // must be *client

          // if OTA is sucessful processor will reboot here
        }
        else
        {
          unitStatus += " OTA OK. ";
          Serial.println(F( "Already on latest version" ));
        }
      }
    }
    else
    {
      Serial.printf("[HTTPS] GET... failed, error: %s\n", https.errorToString(httpCode).c_str());
    }
    https.end();
  }
  else
  {
    Serial.printf("[HTTPS] Unable to connect\n");
  }
} // checkOTAupdate()

// *******************************************************
// ************** postToThingSpeak ***********************
// *******************************************************
void postToThingSpeak()
{
  // assemble and post the data
  if ( client.connect( THINGSPEAK_SERVER, 80 ) == true )
  {
    Serial.println(F("ThingSpeak Server connected."));
    // declare dataString as a String and initialize with the API_WRITE_KEY
    String dataString = wm_ts_write_key;
    // cocatenate each field onto the end of dataString
    if ( rtcData.bme280Fail == false )
    {
      dataString += "&field1=" + String( sensorData.celsius );
      dataString += "&field2=" + String( sensorData.humidity );
      dataString += "&field8=" + String( sensorData.fahrenheit );
    }
    dataString += "&field3=" + String( rtcData.timeAwake );  // from last cycle
    dataString += "&field4=" + String( sensorData.seaLevelPressure );
    if ( rtcData.bh1750Fail == false )
    {
      dataString += "&field5=" + String( sensorData.lightLevel );
    }
    dataString += "&field6=" + String( sensorData.cellVoltage );
    dataString += "&field7=" + String( sensorData.wifiRSSI );

    dataString += "&status=" + unitStatus;

    Serial.println( dataString );   // show ThingSpeak payload on serial monitor

    // find the number of characters in dataString
    String dataStringLength = String(dataString.length());

    // post the data to ThingSpeak
    client.println("POST /update HTTP/1.1");
    client.println("Host: " + THINGSPEAK_SERVER);
    client.println("Connection: close");
    client.println("X-THINGSPEAKAPIKEY: " + (String)wm_ts_write_key);
    client.println("Content-Type: application/x-www-form-urlencoded");
    client.println("Content-Length: " + dataStringLength);
    client.println("");
    client.print( dataString );
  }
  client.stop();
  Serial.println(F("ThingSpeak disconnected."));
} // postToThingSpeak()

// *******************************************************
// *************** printToSerialPort *********************
// *******************************************************
void printToSerialPort()
{
  char buffer[70];  // must be >1 longer than the printed string

  float si = sensorData.seaLevelPressure * HPA_TO_INHG;

  // buffer to store formatted print string
  // header line
  Serial.println();
  Serial.println(F("   Â°C    (Â°F)    RH%  SLP mb    (in)     Lux   Volt   dBm    sec    seq"));

  // data line (EXTENDED!!!)
  sprintf( buffer, "%5.1f (%5.1f)  %5.1f  %6.1f (%5.2f)  %6.0f   %4.2f   %d   %4.2f    %d",
           sensorData.celsius, sensorData.fahrenheit, sensorData.humidity, sensorData.seaLevelPressure, si,
           sensorData.lightLevel, sensorData.cellVoltage, sensorData.wifiRSSI, rtcData.timeAwake, rtcData.sequence );

  Serial.println( buffer );

  Serial.println( unitStatus );

  // print a dashed line footer
  for ( int i = 0; i < 70; i++ )
  {
    Serial.print("-");
  }
  Serial.println("-");
} // printToSerialPort()

// *******************************************************
// ***************** enterSleep **************************
// *******************************************************
void enterSleep(long sleep)
{
  Serial.print("Sleeping for ");
  Serial.print( sleep );
  Serial.println(" seconds.");
  delay( 2 );                       // delay to let things settle
  // WAKE_RF_DEFAULT wakes the ESP8266 with Wi-Fi enabled
  ESP.deepSleep(sleep * 1000000L, WAKE_RF_DEFAULT);
} // enterSleep()

// *******************************************************
// *********** WIFI MANAGER FUNCTIONS ********************
// *******************************************************

// *******************************************************
// *********** WiFi Manager Callbacks ********************
// *******************************************************
void configModeCallback( WiFiManager * myWFM )
{
  Serial.println(F("Entered config mode"));
  Serial.println( WiFi.softAPIP() );
} // configModeCallback()

// callback when there is need to save config
void saveConfigCallback()
{
  saveConfigFlag = true;
} // saveConfigCallback()

// *******************************************************
// **************** readConfig ***************************
// *******************************************************
void readConfig()
{
  // SPIFFS = Serial Peripheral Inteface Flash File System
  // http://esp8266.github.io/Arduino/versions/2.0.0/doc/filesystem.html
  // from wifiManager example AutoConnectWithFSParameters
  // https://github.com/tzapu/WiFiManager/tree/master/examples/AutoConnectWithFSParameters
  if ( SPIFFS.begin() )
  {
    if ( SPIFFS.exists( CONFIG_FILENAME ) )
    {
      // file exists - read and copy to globals
      File configFile = SPIFFS.open( CONFIG_FILENAME, "r" );
      if ( configFile )
      {
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size + 1]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonDocument doc( 1024 );

        deserializeJson( doc, buf.get() );
        serializeJsonPretty(doc, Serial);
        Serial.println("");

        strncpy( wm_ts_channel,   doc["ts_channel"],   sizeof(wm_ts_channel) );
        strncpy( wm_ts_write_key, doc["ts_write_key"], sizeof(wm_ts_write_key) );
        strncpy( wm_elevation,    doc["elevation"],    sizeof(wm_elevation) );
      }
#ifdef DEBUG
      else
      {
        Serial.println(F("Failed to load json config"));
      }
#endif
      configFile.close();
    }
  }
#ifdef DEBUG
  else
  {
    Serial.println(F("Failed to mount FS"));
  }
#endif
} // readConfig()

// *******************************************************
// ***************** writeConfig *************************
// *******************************************************
// saves user parameters in SPIFFS
void writeConfig()
{
  DynamicJsonDocument doc( 1024 );
  // copy globals to json
  doc["ts_channel"]    = wm_ts_channel;
  doc["ts_write_key"]  = wm_ts_write_key;
  doc["elevation"]     = wm_elevation;

  File configFile = SPIFFS.open( CONFIG_FILENAME, "w" );
  if ( !configFile )
  {
    Serial.println(F("Failed to write config"));
  }
  serializeJsonPretty( doc, Serial );
  Serial.println("");
  serializeJson( doc, configFile );

  configFile.close();
} // writeConfig()

// *******************************************************
// ***************** openConfigPortal ********************
// *******************************************************
bool openConfigPortal()
{
  bool connectFlag = false;
  digitalWrite( LED_BUILTIN, LOW );
  drd.stop();
  // ***********************************************
  // define & add user parameters to config web page
  // ***********************************************
  myWFM.setCustomHeadElement("<h1 style=\"color:red;\">D1M-WX1 by W4KRL</h1>");
  // parameter name (json id, Prompt, user input variable, length)
  WiFiManagerParameter custom_text_ts("<p style=\"color:red; font-weight: bold\">Enter your ThingSpeak parameters:</p>");
  myWFM.addParameter( &custom_text_ts );
  WiFiManagerParameter custom_ts_channel("ts_channel", "Channel ID", wm_ts_channel, 10);
  myWFM.addParameter( &custom_ts_channel );
  WiFiManagerParameter custom_ts_write_key("ts_write_key", "Write Key", wm_ts_write_key, 20);
  myWFM.addParameter( &custom_ts_write_key );
  WiFiManagerParameter custom_elevation("elevation", "Elevation", wm_elevation, 10);
  myWFM.addParameter( &custom_elevation );

  myWFM.setTimeout( 240 );    // in seconds
  myWFM.setAPCallback( configModeCallback );
  myWFM.setSaveConfigCallback( saveConfigCallback );

  Serial.println("Open Config Portal");
  myWFM.startConfigPortal( AP_PORTAL_NAME );

  if ( WiFi.status() == WL_CONNECTED )
  {
    connectFlag = true;
    // if you get here you have connected to the WiFi
    Serial.println("WiFi connected.");

    // get the user's parameters from WiFiManager
    strncpy( wm_ts_channel,   custom_ts_channel.getValue(),   sizeof(wm_ts_channel) );
    strncpy( wm_ts_write_key, custom_ts_write_key.getValue(), sizeof(wm_ts_write_key) );
    strncpy( wm_elevation,    custom_elevation.getValue(),    sizeof(wm_elevation) );

    if ( saveConfigFlag )
    {
      Serial.println("Save new config");
      writeConfig();
      saveConfigFlag = false;
    }
  }
  digitalWrite( LED_BUILTIN, HIGH );
  return connectFlag;
} //openConfigPortal()

// *******************************************************
// ***************** logonToRouter ***********************
// *******************************************************
bool logonToRouter()
{
  bool connectFlag = false;
  WiFi.forceSleepWake();       // Bakke
  delay( 1 );
  WiFi.persistent( false );    // prevent it from writing logon to flash memory

  if ( rtcValid )
  { // The RTC data was good, make a fast connection
    Serial.println(F("RTC ok: try fast connection"));
    WiFi.begin( wifiSSID, wifiPASS, rtcData.wifiChannel, rtcData.bssid, true );
  }
  else
  { // The RTC data was not valid, so make a regular connection
    Serial.println(F("RTC bad: try normal connection"));
    WiFi.begin( wifiSSID, wifiPASS );
  }

  int count = 0;
  while ( WiFi.status() != WL_CONNECTED && count < 100 )
  {
    count++;
    delay( 100 );                     // ms delay between reports
    Serial.print(".");
  } // loop while not connected or counted out
  // WiFi is connected or counted out
  if ( WiFi.status() == WL_CONNECTED )
  {
    connectFlag = true;
  }
  return connectFlag;
} // logonToRouter()

// *******************************************************
// *********** calculateSeaLevelPressure *****************
// *******************************************************

/*
   Calculate relative sea level pressure from absolute station pressure in hPa
   temperature in Â°C, elevation in m
   http://www.sandhurstweather.org.uk/barometric.pdf
   http://keisan.casio.com/exec/system/1224575267
*/

float calculateSeaLevelPressure(float celsius, float stationPressure, float elevation)
{
  float slP = stationPressure / pow(2.718281828, -(elevation / ((273.15 + celsius) * 29.263)));
  return slP;
} // calculateSeaLevelPressure()
