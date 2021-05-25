#include <OneWire.h>
#include <DallasTemperature.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Arduino_JSON.h>
#include "SIM800L.h"
#include <EEPROM.h>

//status variables
bool gpsAvailable = false;
bool gsmAvailable = false;
bool tempBelowX = false;
bool mainBatBelowY = false;

//gps
const int gpsRxPin = 3;
const int gpsTxPin = 4;

char longitude[15];
char latitude[15];
int accuracy = 0;
byte Y, M, D, H, MN, S;

//gsm
const int gsmRxPin = 8;
const int gsmTxPin = 7;
const int gsmRstPin = 6;

//status rgb
const int R = 11;
const int G = 12;
const int B = A4;

//buzzer
const int buzzer = 9;
//rollerSwitch
const int rollerSwitch = A1;
//relay
const int relay = 10;
//bat level
const int batLevel = A5;

//##configs##

//hardcoded boxid
const char* boxId = "cc79a9b7-b6dd-4e5c-adf5-f2c72b6377ab";

//api
const char apn[] = "";
const char readingsUrl[] = "http://167.86.75.239:9123/api/v1/readings";
const char settingsUrl[] = "http://167.86.75.239:9123/api/v1/settings";
const char contentType[] = "application/json";
//const char payload[] = "{ \"box_id\": \"cc79a9b7-b6dd-4e5c-adf5-f2c72b6377ab\", \"created_at\": \"2021-05-07 07:52:52\", \"updated_at\": \"2021-05-12 11:22:52\", \"charge\": \"50\", \"lat\": \"20\", \"long\": \"30\", \"alt\": \"2\", \"accuracy\": \"1\", \"timestamp\": \"2021-05-12 11:22:52\", \"temp\": \"23\"}";
//const  char payload[] = "{\"box_id\":\"cc79a9b7-b6dd-4e5c-adf5-f2c72b6377ab\",\"charge\":\"96\",\"lat\":\"0\",\"long\":\"0\",\"switch\":\"1\",\"accuracy\":\"0\",\"timestamp\":\"0\",\"temp\":\"23.6\"}";

//settings
int sendIntervalMs = 60000; //read_interval_ms from the settings model default 1 minute
int lowTemp = 2;
int highTemp = 10;
int lowBattPercentage = 20;

//ds18b20
const int oneWireBus = 13;
OneWire oneWire(oneWireBus);
DallasTemperature tempSensor(&oneWire);

//gps
TinyGPSPlus gps;
SoftwareSerial gpsSerial(gpsRxPin, gpsTxPin);
//sim 800
SIM800L* sim800l;

//current eeprom address
int addr = 0;

void setup()
{
  Serial.begin(9600);
  gpsSerial.begin(9600);
  Serial.println("Initializing Fridgedino please wait...");
  pinMode(rollerSwitch, OUTPUT);
  pinMode(relay, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(R, OUTPUT);
  pinMode(G, OUTPUT);
  pinMode(B, OUTPUT);
  //initialise gsm software serial
  SoftwareSerial* gsmSerial = new SoftwareSerial(gsmRxPin, gsmTxPin);
  gsmSerial->begin(9600);
  delay(1000);
  // Initialize SIM800L driver with an internal buffer of 200 bytes and a reception buffer of 512 bytes, debug disabled
  sim800l = new SIM800L((Stream *)gsmSerial, gsmRstPin, 200, 512);
  //setup gsm module
  setupModule();
}

void loop()
{
  //testRelay();
  //getGpsData();
  //getcurrTemperature();
  //testBeep(5);
  //statusLedTest();
  //getSettings();
  //sendData();
  //getBatLevel();
  //getRollerState();
  //get settings
  Serial.println("Geting device settings");
  getSettings();
  
  //get sensor readings and send
  int batVal = getBatLevel();
  int switchVal = getRollerState();
  float tempVal = getcurrTemperature();
  float latitude = getLat();
  float longitude = getLon();
  String timeStamp = getTimestamp();
  
  Serial.print("The data to be sent: BatLevel");
  Serial.print(batVal);
  Serial.print("SwitchVal");
  Serial.print(switchVal);
  Serial.print("TempVal");
  Serial.print(tempVal);
  Serial.print("Latitude");
  Serial.print(latitude);
  Serial.print("Timestamp");
  Serial.print(timeStamp);
  Serial.print("Longitude");
  Serial.println(longitude);
  sendData(batVal,longitude,latitude,switchVal,accuracy,timeStamp, tempVal);
  //sendSensorData();
  //getSettings();
  //sendPseudoData();
  delay(sendIntervalMs);
}

float getcurrTemperature() {
  tempSensor.requestTemperatures();
  float temperatureC = tempSensor.getTempCByIndex(0);
  return temperatureC;
}

int getRollerState() {
  int state = digitalRead(rollerSwitch);
  return state;
}
float getLat() {
  if (gps.encode(gpsSerial.read())) {
    if (gps.location.isValid())
    {
      accuracy = 1;
      //dtostrf(gps.location.lat(), 10, 7, latitude);
      float latitude = gps.location.lat();
      return latitude;
    }
    else {
      accuracy = 0;
      float latitude = 0;
      return latitude;
    }
  }
}
float getLon() {
  if (gps.encode(gpsSerial.read())) {
    if (gps.location.isValid())
    {
      accuracy = 1;
      float longitude = gps.location.lng();
      return longitude;
    }
    else {
      accuracy = 0;
      float longitude = 0;
      return longitude;
    }
  }
}

String getTimestamp() {
  if (gps.time.isValid())
  {
    H = gps.time.hour();
    MN = gps.time.minute();
    S = gps.time.second();

  }
  else {
    H = "00";
    MN = "00";
    S = "00";

  }
  if (gps.date.isValid())
  {
    Y = gps.date.year();
    M = gps.date.month();
    D = gps.date.day();

  }
  else
  {
    Y = 0;
    M = 0;
    D = 0;
  }
  //the full timestamp
  String timeStamp = String(Y) + String(M) + String(D) + String(H) + String(MN) + String(S);
  return timeStamp;
}

void switchRelay(int state) {
  digitalWrite(relay, state);
}

int getBatLevel() {
  int maxVal = 800;
  int level = analogRead(batLevel);
  int batLevel = map(level, 0, maxVal, 0, 100);
  Serial.println(batLevel);
  return batLevel;
}

void lightStatus(bool gsmAvailable, bool gpsAvailable) {
  //green if gsm and gps are okay
  if (gsmAvailable && gpsAvailable) {
    lightStatus('g');
  }
  else if (!gsmAvailable) {
    //light red if gsm is unavailable
    lightStatus('r');
  }
  else if (!gpsAvailable) {
    //light blue if gps is unavailable
    lightStatus('b');
  }
}


void testBeep(int pulses) {
  for (int i = 0; i < pulses; i++) {
    tone(buzzer, 400, 500);
    noTone(buzzer);
  }
}


void statusLedTest() {
  digitalWrite(R, 1);
  delay(1000);
  digitalWrite(R, 0);
  delay(1000);
  digitalWrite(G, 1);
  delay(1000);
  digitalWrite(G, 0);
  delay(1000);
  digitalWrite(B, 1);
  delay(1000);
  digitalWrite(B, 0);
  delay(1000);
}
void lightStatus(char colour) {
  if (colour == 'r') {
    digitalWrite(R, 1);
    delay(1000);
    digitalWrite(R, 0);
    delay(1000);
  }
  else if (colour == 'g') {
    digitalWrite(G, 1);
    delay(1000);
    digitalWrite(G, 0);
    delay(1000);
  }
  else if (colour == 'b') {
    digitalWrite(B, 1);
    delay(1000);
    digitalWrite(B, 0);
    delay(1000);
  }
  else {
    digitalWrite(R, 0);
    digitalWrite(G, 0);
    digitalWrite(B, 0);
  }
}

void setupModule() {
  // Wait until the module is ready to accept AT commands
  while (!sim800l->isReady()) {
    Serial.println(F("Problem to initialize AT command, retry in 1 sec"));
    delay(1000);
  }
  Serial.println(F("Setup Complete!"));

  // Wait for the GSM signal
  uint8_t signal = sim800l->getSignal();
  while (signal <= 0) {
    delay(1000);
    signal = sim800l->getSignal();
  }
  Serial.print(F("Signal OK (strenght: "));
  Serial.print(signal);
  Serial.println(F(")"));
  delay(1000);

  // Wait for operator network registration (national or roaming network)
  NetworkRegistration network = sim800l->getRegistrationStatus();
  while (network != REGISTERED_HOME && network != REGISTERED_ROAMING) {
    delay(1000);
    network = sim800l->getRegistrationStatus();
  }
  Serial.println(F("Network registration OK"));
  delay(1000);

  // Setup APN for GPRS configuration
  bool success = sim800l->setupGPRS(apn);
  while (!success) {
    success = sim800l->setupGPRS(apn);
    delay(5000);
  }
  Serial.println(F("GPRS config OK"));
}
void sendData(int batVal, float latitude,float longitude,int switchVal,int accuracy,String timeStamp,float tempVal) {
  String data ="{\"box_id\":\"cc79a9b7-b6dd-4e5c-adf5-f2c72b6377ab\",\"charge\":\""+String(batVal)+"\",\"lat\":\""+String()+"\",\"long\":\""+String()+"\",\"switch\":\""+String(switchVal)+"\",\"accuracy\":\""+String(accuracy)+"\",\"timestamp\":\""+timeStamp+"\",\"temp\":\""+String(tempVal)+"\"}";
  const char* payload = data.c_str();
  //const  char payload[] = "{\"box_id\":\"cc79a9b7-b6dd-4e5c-adf5-f2c72b6377ab\",\"charge\":\"96\",\"lat\":\"0\",\"long\":\"0\",\"switch\":\"1\",\"accuracy\":\"0\",\"timestamp\":\"0\",\"temp\":\"23.6\"}";
  //const char payload[] = "{ \"box_id\": \"cc79a9b7-b6dd-4e5c-adf5-f2c72b6377ab\", \"created_at\": \"2021-05-07 07:52:52\", \"updated_at\": \"2021-05-12 11:22:52\", \"charge\": \"50\", \"lat\": \"20\", \"long\": \"30\", \"alt\": \"2\", \"accuracy\": \"1\", \"timestamp\": \"2021-05-12 11:22:52\", \"temp\": \"23\"}";
  //char data[] ="{\"box_id\":\"cc79a9b7-b6dd-4e5c-adf5-f2c72b6377ab\",\"charge\":\""+String(batVal)+"\",\"lat\":\"00\",\"long\":\"00\",\"switch\":\""+String(switchVal)+"\",\"accuracy\":\"0\",\"timestamp\":\"0\",\"temp\":\""+String(tempVal)+"\"}";
  //int n = data.length();
  //char payload[n+1];
  //strcpy(payload,data.c_str());

  // Establish GPRS connectivity (5 trials)
  bool connected = false;
  for (uint8_t i = 0; i < 5 && !connected; i++) {
    delay(1000);
    connected = sim800l->connectGPRS();
  }

  // Check if connected, if not reset the module and setup the config again
  if (connected) {
    Serial.println(F("GPRS connected !"));
  } else {
    Serial.println(F("GPRS not connected !"));
    Serial.println(F("Reset the module."));
    sim800l->reset();
    setupModule();
    return;
  }

  Serial.println(F("Start HTTP POST..."));

  // Do HTTP POST communication with 10s for the timeout (read and write)
  uint16_t rc = sim800l->doPost(readingsUrl, contentType, payload, 10000, 10000);
  if (rc == 200) {
    // Success, output the data received on the serial
    Serial.print(F("HTTP POST successful ("));
    Serial.print(sim800l->getDataSizeReceived());
    Serial.println(F(" bytes)"));
    Serial.print(F("Received : "));
    Serial.println(sim800l->getDataReceived());
  } else {
    // Failed...
    Serial.print(F("HTTP POST error "));
    Serial.println(rc);
  }

  // Close GPRS connectivity (5 trials)
  bool disconnected = sim800l->disconnectGPRS();
  for (uint8_t i = 0; i < 5 && !connected; i++) {
    delay(1000);
    disconnected = sim800l->disconnectGPRS();
  }

  if (disconnected) {
    Serial.println(F("GPRS disconnected !"));
  } else {
    Serial.println(F("GPRS still connected !"));
  }

  // Go into low power mode
  bool lowPowerMode = sim800l->setPowerMode(MINIMUM);
  if (lowPowerMode) {
    Serial.println(F("Module in low power mode"));
  } else {
    Serial.println(F("Failed to switch module to low power mode"));
  }

  // End of program... wait...
  while (1);
}

void sendPseudoData() {
  // Establish GPRS connectivity (5 trials)
  bool connected = false;
  for (uint8_t i = 0; i < 5 && !connected; i++) {
    delay(1000);
    connected = sim800l->connectGPRS();
  }

  // Check if connected, if not reset the module and setup the config again
  if (connected) {
    Serial.println(F("GPRS connected !"));
  } else {
    Serial.println(F("GPRS not connected !"));
    Serial.println(F("Reset the module."));
    sim800l->reset();
    setupModule();
    return;
  }

  Serial.println(F("Start HTTP POST..."));
  const  char payload[] = "{\"box_id\":\"cc79a9b7-b6dd-4e5c-adf5-f2c72b6377ab\",\"charge\":\"96\",\"lat\":\"0\",\"long\":\"0\",\"switch\":\"1\",\"accuracy\":\"0\",\"timestamp\":\"0\",\"temp\":\"23.6\"}";
  // Do HTTP POST communication with 10s for the timeout (read and write)
  uint16_t rc = sim800l->doPost(readingsUrl, contentType, payload, 10000, 10000);
  if (rc == 200) {
    // Success, output the data received on the serial
    Serial.print(F("HTTP POST successful ("));
    Serial.print(sim800l->getDataSizeReceived());
    Serial.println(F(" bytes)"));
    Serial.print(F("Received : "));
    Serial.println(sim800l->getDataReceived());
  } else {
    // Failed...
    Serial.print(F("HTTP POST error "));
    Serial.println(rc);
  }

  // Close GPRS connectivity (5 trials)
  bool disconnected = sim800l->disconnectGPRS();
  for (uint8_t i = 0; i < 5 && !connected; i++) {
    delay(1000);
    disconnected = sim800l->disconnectGPRS();
  }

  if (disconnected) {
    Serial.println(F("GPRS disconnected !"));
  } else {
    Serial.println(F("GPRS still connected !"));
  }

  // Go into low power mode
  bool lowPowerMode = sim800l->setPowerMode(MINIMUM);
  if (lowPowerMode) {
    Serial.println(F("Module in low power mode"));
  } else {
    Serial.println(F("Failed to switch module to low power mode"));
  }

}
void getSettings() {
  // Establish GPRS connectivity (5 trials)
  bool connected = false;
  for (uint8_t i = 0; i < 5 && !connected; i++) {
    delay(1000);
    connected = sim800l->connectGPRS();
  }

  // Check if connected, if not reset the module and setup the config again
  if (connected) {
    Serial.println(F("GPRS connected !"));
  } else {
    Serial.println(F("GPRS not connected !"));
    Serial.println(F("Reset the module."));
    sim800l->reset();
    setupModule();
    return;
  }

  Serial.println(F("Start HTTP GET..."));

  // Do HTTP GET communication with 10s for the timeout (read)
  uint16_t rc = sim800l->doGet(settingsUrl, 10000);
  if (rc == 200) {
    // Success, store and output the data received on the serial
    String settings = sim800l->getDataReceived();
    String settingsPayload = "{\"id\":1,\"title\":null,\"created_at\":\"2021-05-20T11:54:35.000Z\",\"updated_at\":\"2021-05-20T11:54:35.000Z\",\"read_interval_ms\":12000,\"low_temp_celsius\":30000,\"high_temp_celsius\":180000,\"low_batt_percentage\":900000,\"title8\":null}";
    JSONVar settingsObject = JSON.parse(settingsPayload);
    //settings
    JSONVar sendIntervalMs = settingsObject["temp_interval_ms"];
    JSONVar highTemp = settingsObject["low_temp_celsius"];
    JSONVar lowTemp = settingsObject["high_temp_celsius"];
    JSONVar lowBattPercentage = settingsObject["low_batt_percentage"];
    Serial.print(F("HTTP GET successful ("));
    Serial.print(sim800l->getDataSizeReceived());
    Serial.println(F(" bytes)"));
    Serial.print(F("Received : "));
    Serial.println(sim800l->getDataReceived());
    //Serial.print("The decoded json response is: ");
    //Serial.println(settingsObject);

    //Serial.print("The temp interval is: ");
    //Serial.println(tempInterval);
  } else {
    // Failed...
    Serial.print(F("HTTP GET error "));
    Serial.println(rc);
  }

  // Close GPRS connectivity (5 trials)
  bool disconnected = sim800l->disconnectGPRS();
  for (uint8_t i = 0; i < 5 && !connected; i++) {
    delay(1000);
    disconnected = sim800l->disconnectGPRS();
  }

  if (disconnected) {
    Serial.println(F("GPRS disconnected !"));
  } else {
    Serial.println(F("GPRS still connected !"));
  }

  // Go into low power mode
  bool lowPowerMode = sim800l->setPowerMode(MINIMUM);
  if (lowPowerMode) {
    Serial.println(F("Module in low power mode"));
  } else {
    Serial.println(F("Failed to switch module to low power mode"));
  }

  
}

void writeEeprom(int address,int val) {
   EEPROM.write(address, val);
   address++;
   if (address == EEPROM.length()) {
    address = 0;
  }
  delay(100);
}
