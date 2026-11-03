// Bench test for an ELRS 4.x receiver connected as a flight controller would
// be. Prints channels, link stats, and all the newer telemetry sensors the
// receiver can produce, and joins the CRSF network as a discoverable device:
// with a 4.0 TX/RX pair this sketch should show up in the ExpressLRS Lua
// under Other Devices, and a VBAT-sensing receiver should produce a CELLS
// frame (source id 128) with millivolt precision.

#include <AlfredoDShot.h>
#include <AlfredoCRSF.h>
#include <HardwareSerial.h>
#include <esp_task_wdt.h>
#include <WiFi.h>
#include <WiFiAP.h>
#include <ArduinoOTA.h>
#include <Arduino.h>
#include <EEPROM.h>



//-------Wifi config---------
const char *ssid = "Combat-Robot-Motherboard"; //wifi ssid
const char *password = ""; //wifi password

//------Reciever config-----------
#define PIN_RX 16
#define PIN_TX 5
HardwareSerial crsfSerial(1);
AlfredoCRSF crsf;
const int NUM_CHANNELS = 10; //number of reciever channels to use

//------ESC config--------------
const int PIN_ESC_1 = 9;
const int PIN_ESC_2 = 17;
const int PIN_ESC_3 = 18;
const int PIN_ESC_4 = 8;
const uint8_t MOTOR_POLES = 14;
const int PIN_ESC_L = PIN_ESC_1; //defines pins attached to right and left drive motors
const int PIN_ESC_R = PIN_ESC_2;
const int PIN_ESC_W = PIN_ESC_3;
AlfredoDShot esc_R;
AlfredoDShot esc_L;
AlfredoDShot esc_W;

//=============GLOBAL VARIABLES==================
//--------reciever----------
int pwm[NUM_CHANNELS+1]; //list values from 1000-2000
int duty[NUM_CHANNELS+1]; //list values from 0 - 100
bool rc_status = false; //whether getting rc signal, used for triggering failsafe
//----------other----------
float volts; //variable to store current battery voltage
uint32_t lastHeartbeatMs = 0;
uint32_t lastPrintMs = 0;
int motor_L; //throttle for sending to esc
int motor_R;
int motor_W;
bool wep_reversed = false;
int esc_R_rpm;
int esc_L_rpm;
int16_t throttle_print;
unsigned long long motor_update = 0; //tracks how long since last motor update

void setup()
{
  AlfredoDShot::releaseBootloader(PIN_ESC_L);  // must be first, see AlfredoDShot.h pulls signal low to reset esc
  AlfredoDShot::releaseBootloader(PIN_ESC_R);  // must be first, see AlfredoDShot.h
  AlfredoDShot::releaseBootloader(PIN_ESC_W);  // must be first, see AlfredoDShot.h
  Serial.begin(115200);
  Serial.println("Boot");
  ArduinoOTA.setHostname("esp32-ap"); //wifi ota config
  ArduinoOTA.setPassword(password); //enter this if window opens in arduino IDE asking for pswrd
  WiFi.softAP(ssid, password);
  ArduinoOTA.begin();  //starts ota

  crsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_RX, PIN_TX);
  if (!crsfSerial) while (1) Serial.println("Invalid crsfSerial configuration");

  crsf.begin(crsfSerial); // defaults to CRSF_ADDRESS_FLIGHT_CONTROLLER
  crsf.setDeviceName("AlfredoCRSF"); // answer device discovery pings
  esc_R.begin(PIN_ESC_R, DSHOT600, true, MOTOR_POLES); //initialized dshot to a defined pin (pin 1 = M1)
  esc_L.begin(PIN_ESC_L, DSHOT600, true, MOTOR_POLES);
  esc_W.begin(PIN_ESC_W, DSHOT600, true, MOTOR_POLES);
}

void loop()
{
  ArduinoOTA.handle();
  update_channels();
  update_motors();
}
