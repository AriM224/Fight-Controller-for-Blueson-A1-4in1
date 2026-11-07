// Bench test for an ELRS 4.x receiver connected as a flight controller would
// be. Prints channels, link stats, and all the newer telemetry sensors the
// receiver can produce, and joins the CRSF network as a discoverable device:
// with a 4.0 TX/RX pair this sketch should show up in the ExpressLRS Lua
// under Other Devices, and a VBAT-sensing receiver should produce a CELLS
// frame (source id 128) with millivolt precision.

/* To connect and recieve Serial data over wifi. 
1. Connect to Combat-Robot-Motherboard network
2. Open powershell
3. Run these commands (Note the first two commands have no output and the 
                      third will not print anything if no packets are recieved):
$u = New-Object Net.Sockets.UdpClient(4210)
$ep = New-Object Net.IPEndPoint([Net.IPAddress]::Any, 0)
while ($true) { [Text.Encoding]::ASCII.GetString($u.Receive([ref]$ep)) }
*/
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
const char *password = "youarewhatyouyeet"; //wifi password

//-------Wifi serial (log over UDP)---------
#include <WiFiUdp.h>
const uint16_t LOG_PORT = 4210;
IPAddress logDest(192, 168, 4, 255);  // default guess; replaced once the laptop says hello
WiFiUDP logUdp;

// Prints to USB serial AND buffers for UDP. Nothing is sent until flush().
class TeePrint : public Print {
  char buf[1024];
  size_t len = 0;
public:
  size_t write(uint8_t c) override { return write(&c, 1); }
  size_t write(const uint8_t *data, size_t n) override {
    Serial.write(data, n);
    for (size_t i = 0; i < n; i++) {
      buf[len++] = data[i];
      if (len == sizeof(buf)) flush();   // safety only
    }
    return n;
  }
  void flush() override {
    if (logUdp.parsePacket() > 0) {      // laptop said hello: learn its IP
      logDest = logUdp.remoteIP();
      logUdp.clear();                     // use logUdp.flush() on older cores
    }
    if (len > 0 && WiFi.softAPgetStationNum() > 0) {
      logUdp.beginPacket(logDest, LOG_PORT);
      logUdp.write((const uint8_t *)buf, len);
      logUdp.endPacket();
    }
    len = 0;
  }
};
TeePrint Log;
//------Reciever config-----------
#define PIN_RX 16
#define PIN_TX 5
#define TELEM_INTERVAL_MS 200   // how often telemetry goes to the TX (ELRS has little bandwidth, 100-500 is sensible)
HardwareSerial crsfSerial(1);
AlfredoCRSF crsf;
const int NUM_CHANNELS = 10; //number of reciever channels to use

//------ESC config--------------
const int PIN_ESC_1 = 9;
const int PIN_ESC_2 = 17;
const int PIN_ESC_3 = 18;
const int PIN_ESC_4 = 8;
const uint8_t MOTOR_POLES = 14;
// AM32 2.20 and older send ESC current in 0.5 A steps, but the library reads it as
// 1 A steps, so esc.current() comes out DOUBLE the real value. 0.5 corrects that.
// If you update the ESCs to AM32 2.21 or newer, change this to 1.0.
const float CURRENT_SCALE = 1.0;
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
uint32_t lastTelemMs = 0;
//----------esc telemetry----------
// One slot per ESC. Use these index names instead of 0/1/2.
const int IDX_R = 0;
const int IDX_L = 1;
const int IDX_W = 2;
float esc_rpm[3];    // motor RPM, NAN if the ESC isn't replying
float esc_temp[3];   // degrees C, NAN until EDT is working
float esc_volts[3];  // volts, NAN until EDT is working
float esc_amps[3];   // amps, NAN until EDT is working
float pack_volts = NAN; // battery voltage (first ESC that reports it)
float pack_amps = 0;    // total current from all ESCs that report it
uint32_t edt_sent[3] = {0, 0, 0}; // millis() when we last sent the EDT enable command to each ESC
//----------other----------
uint32_t lastHeartbeatMs = 0;
uint32_t lastPrintMs = 0;
int motor_L; //throttle for sending to esc
int motor_R;
int motor_W;
bool wep_reversed = false;
int16_t throttle_print;
unsigned long long motor_update = 0; //tracks how long since last motor update
int last_print = 0;

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
  logUdp.begin(LOG_PORT);
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
  get_telem();    // read the latest values the ESCs sent back
  send_telem();   // forward them to the TX over ELRS
  if (millis() - last_print > 10)
  {
    print_status();
    last_print = millis();
  }
}

// Prints one status block: connection state + current motor/ESC state
void print_status()
{
  // --- Connection status ---
  Log.println("===== STATUS =====");
  Log.printf("WiFi AP clients: %d | UDP log -> port %u\n",
            WiFi.softAPgetStationNum(), LOG_PORT);
  esp_reset_reason_t r = esp_reset_reason();
  Log.printf("Reset reason: %d%s\n", (int)r, r == ESP_RST_BROWNOUT ? " (BROWNOUT)" : "");
  Log.printf("RC link: %s\n", rc_status ? "UP" : "DOWN (failsafe)");

  if (rc_status) {
    const crsfLinkStatistics_t *ls = crsf.getLinkStatistics();
    Log.printf("  RSSI1:%d dBm  LQ:%u%%  SNR:%d  RFmode:%u  TXpower:%u\n",
               -(int)ls->uplink_RSSI_1, ls->uplink_Link_quality,
               ls->uplink_SNR, ls->rf_Mode, ls->uplink_TX_Power);
  }

  // --- Current status ---
  Log.printf("Throttle  L:%d  R:%d  W:%d%s\n",
             motor_L, motor_R, motor_W, wep_reversed ? " (W reversed)" : "");

  const char *names[3] = {"R", "L", "W"};
  for (int i = 0; i < 3; i++) {
    if (isnan(esc_rpm[i])) {
      Log.printf("ESC %s: no reply\n", names[i]);
    } else {
      Log.printf("ESC %s: %.0f rpm", names[i], esc_rpm[i]);
      if (!isnan(esc_temp[i]))  Log.printf("  %.0fC", esc_temp[i]);
      if (!isnan(esc_volts[i])) Log.printf("  %.2fV", esc_volts[i]);
      if (!isnan(esc_amps[i]))  Log.printf("  %.1fA", esc_amps[i]);
      Log.println();
    }
  }

  if (!isnan(pack_volts)) Log.printf("Pack: %.2fV  %.1fA\n", pack_volts, pack_amps);
  else                    Log.println("Pack: no voltage data");
  Log.flush();   // send the whole block as one packet
}
