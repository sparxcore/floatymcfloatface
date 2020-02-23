#include <SPI.h>
#include <Adafruit_BME280.h>
#include <DS18B20.h>
#include <dummy.h>
#include <HardwareSerial.h>

#include <TinyGPS++.h>
#include <lmic.h>
#include <hal/hal.h>

const int led = 25;

TinyGPSPlus gps;
DS18B20 thermo(4);

uint8_t airSensor[] = {40, 170, 000, 000, 00, 0, 0, 000};
uint8_t waterSensor[] = {40, 170, 000, 000, 00, 0, 0, 000};
uint8_t selected;

const int solarADC = 36;
const int battADC = 37;
int solarA = 0;
int battA = 0;
float battV;
float solarV;
int sampCount = 80;    

const int gpsen = 13;

//const int i2c_sda = 21;
//const int i2c_scl = 22;
const int bmeaddress = 0x76;
bool status;

float latitude;
float longitude;
float acc;
float alt;
float waterT;
float airT;
float intT;
float intH;

const int payloadsize = 43;
const int payload = 21;

const int uS_TO_S_FACTOR=1000000;
const int TIME_TO_SLEEP=60;

const int GPS_RX=33;
const int GPS_TX=23;
HardwareSerial GPSSerial(2);
Adafruit_BME280 bme;

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3, 0x70.
// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const PROGMEM u1_t NWKSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const u1_t PROGMEM APPSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x00000000 ;
// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 30;
// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 14,
  .dio = {26, 34,35},
};


typedef struct sensorData_t{
  float latitude;
  float longitude;
  uint8_t acc;
  uint8_t alt;
  int16_t solarV;
  int16_t battV;
  int16_t airT;
  int16_t waterT;
  int16_t intT;
  uint8_t intH;
};

typedef union loraPacket_t{
sensorData_t sensor;
byte payload[sizeof(sensorData_t)];
};

loraPacket_t getinfo;

void volts()
{
  for (int i = 0; i < sampCount; i++){
    battA = battA + analogRead(battADC);
    solarA = solarA + analogRead(solarADC);
    // 1ms pause adds more stability between reads.
    delay(1);
  }
  battA = battA/sampCount;
  solarA = solarA/sampCount;
  delay(10);
  solarV = (solarA * 8.29)/2047*.98;
  battV = (battA * 4.59)/2047*.98;    
  Serial.println();
  Serial.print("battT = ");
  Serial.print(battV);
  Serial.print(" solarV = ");
  Serial.print(solarV);
  
  delay(10);
}

void temphumid(){
  intT = bme.readTemperature();
  intH = bme.readHumidity();
  
  Serial.println(" ");
  Serial.print("intT = ");
  Serial.print(intT);
  Serial.print(" ℃ ");
  Serial.print("intH = ");
  Serial.print(intH);
  Serial.println(" %");
}

void readthermo(){
    thermo.select(airSensor);
    airT = thermo.getTempC();
    Serial.print("airT = ");
    Serial.print(airT);
    Serial.print(" ℃ ");
    thermo.select(waterSensor);
    waterT = thermo.getTempC();
    Serial.print("waterT = ");
    Serial.print(waterT);
    Serial.println(" ℃");
    float aT = airT + .05;
    float wT = waterT + .05;
}

void get_gps () {
  while (GPSSerial.available())
  gps.encode(GPSSerial.read());
  latitude  = gps.location.lat();
  longitude = gps.location.lng();
  acc = gps.hdop.value();
  alt = gps.altitude.meters();
}

void readsensors(){
  get_gps();
  volts();
  temphumid();
  readthermo();
      
  getinfo.sensor.latitude = latitude;
  getinfo.sensor.longitude = longitude;
  getinfo.sensor.acc = acc;
  getinfo.sensor.alt = alt;
  getinfo.sensor.solarV = solarV*100;
  getinfo.sensor.battV = battV*100;
  getinfo.sensor.airT = airT*100;
  getinfo.sensor.waterT = waterT*100;
  getinfo.sensor.intT = intT*100;
  getinfo.sensor.intH = intH;
  
  Serial.print(getinfo.sensor.latitude);
  Serial.print("\t");
  Serial.print(getinfo.sensor.longitude);
  Serial.print("\t");
  Serial.print(getinfo.sensor.acc);
  Serial.print("\t");
  Serial.print(getinfo.sensor.alt);
  Serial.print("\t");
  Serial.print(getinfo.sensor.solarV);
  Serial.print("\t");
  Serial.print(getinfo.sensor.battV);
  Serial.print("\n");
  Serial.print(getinfo.sensor.airT);
  Serial.print("\t");
  Serial.print(getinfo.sensor.waterT);
  Serial.print("\t");
  Serial.print(getinfo.sensor.intT);
  Serial.print("\t");
  Serial.print(getinfo.sensor.intH);
  Serial.print("\t");
  
  
}



void onEvent (ev_t ev) {
  
    Serial.print(os_getTime());
    Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));

      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));

      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));

      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));

      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));

      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));

      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));

      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));

      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));


      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      digitalWrite(led, LOW);
           
      if (LMIC.txrxFlags & TXRX_ACK) {
        Serial.println(F("Received ack"));

      }
      if (LMIC.dataLen) {
        Serial.println(F("Received "));

        Serial.println(LMIC.dataLen);


        Serial.println(F(" bytes of payload"));

      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      digitalWrite(gpsen, LOW);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}




void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } 
  else {
    // Prepare upstream data transmission at the next possible time.
    readsensors();
    LMIC_setTxData2(1, getinfo.payload, payload, 0);
    Serial.println(F("Packet queued"));

    //digitalWrite(led, HIGH);
    
  }
  
  // Next TX is scheduled after TX_COMPLETE event.
}



void setup() {
  Serial.begin(115200);
  pinMode(gpsen, OUTPUT);
  digitalWrite(gpsen, HIGH);
  
  //LMIC_setClockError(MAX_CLOCK_ERROR * 2 / 100);
  
  delay(100);
  Serial.println("Start GPS");
  GPSSerial.begin(9600, SERIAL_8N1,GPS_RX,GPS_TX);
  GPSSerial.setTimeout(2);
  delay(1000);
  status = bme.begin(bmeaddress);
  Serial.println("Read Sensors");

  analogReadResolution(11); // Sets all reads: 12 = 0-4095, 11 = 0-2047, 10 = 0-1024, 9 = 0-512
  analogSetAttenuation(ADC_11db); // Sets all pins: 11db = 0-3.3v, 6dB range = 0-2.2v, 2.5db = 0-1.5v, 0db = 0-1v

  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR); //Make sure I can wakeup
 
  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  // Start job (sending automatically starts OTAA too)
  #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif
   // LMIC.dn2Dr = DR_SF9;
    LMIC_setDrTxpow(DR_SF9,14);// Set data rate and transmit power for uplink)
  if(latitude != longitude){
  do_send(&sendjob);
  }
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);
}

void loop() {
  
 if (latitude != longitude){
    os_runloop_once();
   }
  else{
    for (int i = 0; i < 60; i++)//runs every 1 seconds for 60times before sleeping again
      {
      readsensors();
      if (latitude != longitude)
      {
  i = 60;
  do_send(&sendjob);
 }
    delay(1000);
      }
    digitalWrite(gpsen, LOW);
    esp_deep_sleep_start(); 
 }
}

void printPayload()
{
  for(int i=0; i<payload; i++)
  {
    Serial.print(getinfo.payload[i], HEX);
    Serial.print("\t");
  }
  Serial.println();  
}
