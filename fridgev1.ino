#include <OneWire.h>
#include <DallasTemperature.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Arduino_JSON.h>
#include "SIM800L.h"

//status variables
bool gpsAvailable = false;
bool gsmAvailable = false;

//gps
const int gpsRxPin = 3;
const int gpsTxPin = 4;

char longitude[15];
char latitude[15];
int accuracy = 0;
byte H, M, S;
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
const int rollerSwitch = A4;
//relay
const int relay = 10;
//bat level
const int batLevel = A1;

//##configs##
//api configs
const char* apn = "";
const char* contentType = "application/json";
const char* payload = "{ \"box_id\": \"b004\", \"created_at\": \"2021-05-07 07:52:52\", \"updated_at\": \"2021-05-12 11:22:52\", \"charge\": \"50\", \"lat\": \"20\", \"long\": \"30\", \"alt\": \"2\", \"accuracy\": \"1\", \"timestamp\": \"2021-05-12 11:22:52\", \"temp\": \"23\"}";
const char* readingsUrl = "http://167.86.75.239:9123/api/v1/readings/";
const char* configUrl = "";

//ds18b20
const int oneWireBus = 13;
OneWire oneWire(oneWireBus);
DallasTemperature tempSensor(&oneWire);

//gps
TinyGPSPlus gps;
SoftwareSerial gpsSerial(gpsRxPin, gpsTxPin);
//sim 800
SIM800L* sim800l;
SoftwareSerial* gsmSerial = new SoftwareSerial(gsmRxPin, gsmTxPin);
void setup()
{
  Serial.begin(9600);
  gpsSerial.begin(9600);
  gsmSerial->begin(9600);
  pinMode(rollerSwitch, OUTPUT);
  pinMode(relay, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(R, OUTPUT);
  pinMode(G, OUTPUT);
  pinMode(B, OUTPUT);
}

void loop()
{
  //testRelay();
  //getGpsData();
  getcurrTemperature();
  //testBeep();
  //statusLedTest();
}



float getcurrTemperature() {
  tempSensor.requestTemperatures();
  float temperatureC = tempSensor.getTempCByIndex(0);
  Serial.println(temperatureC);
  return temperatureC;
}

int getRollerState() {
  int state = digitalRead(rollerSwitch);
  Serial.println(state);
  return state;
}

void displayInfo()
{
  if (gps.location.isValid())
  {
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
    Serial.print("Altitude: ");
    Serial.println(gps.altitude.meters());
  }
  else
  {
    Serial.println("Location: Not Available");
  }
  Serial.print("Date: ");
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print("/");
    Serial.print(gps.date.day());
    Serial.print("/");
    Serial.println(gps.date.year());
  }
  else
  {
    Serial.println("Not Available");
  }

  Serial.print("Time: ");
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(":");
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(":");
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(".");
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.println(gps.time.centisecond());
  }
  else
  {
    Serial.println("Not Available");
  }
  Serial.println();
  Serial.println();
  delay(500);
}



void getGpsData() {
  while (gpsSerial.available() > 0) 
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid())
      {
        accuracy = 1;
        dtostrf(gps.location.lat(), 10, 7, latitude);
        dtostrf(gps.location.lat(), 10, 7, longitude);
      }
      else {
        accuracy = 0;
        dtostrf(0.0000, 10, 7, latitude);
        dtostrf(0.0000, 10, 7, longitude);
      }

      //gps time
      if (gps.time.isValid())
      {
        H = gps.time.hour();
        M = gps.time.minute();
        S = gps.time.second();
      }
      else {
        H = "00";
        M = "00";
        S = "00";
      }


      Serial.print(latitude);
      Serial.print(F(","));
      Serial.print(longitude);
      Serial.print(F(","));
      Serial.print(H);
      Serial.print(F(","));
      Serial.print(M);
      Serial.print(F(","));
      Serial.print(S);
      Serial.print(F(","));
      Serial.print("Accuracy");
      Serial.print(F(","));
      Serial.println(accuracy);
    }
  


  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println("No GPS detected");
    //lightStatus('b');
    while (true);
  }
}



void testRelay() {
  digitalWrite(relay, 1);
  delay(1000);
  digitalWrite(relay, 0);
  delay(1000);
}

int getBatLevel() {
  int level = analogRead(batLevel);
  return level;
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

//function to beep the buzzer
void beep(float currTemp, int batLevel) {
  //beep a 2 pulse warning if the temperature is below x

  //beep 3 pulse warning if the main battery is below y
}

void testBeep() {
  for (int i = 0; i < 3; i++) {
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
  while(!sim800l->isReady()) {
    Serial.println(F("Problem to initialize AT command, retry in 1 sec"));
    delay(1000);
  }
  Serial.println(F("Setup Complete!"));

  // Wait for the GSM signal
  uint8_t signal = sim800l->getSignal();
  while(signal <= 0) {
    delay(1000);
    signal = sim800l->getSignal();
  }
  Serial.print(F("Signal OK (strenght: "));
  Serial.print(signal);
  Serial.println(F(")"));
  delay(1000);

  // Wait for operator network registration (national or roaming network)
  NetworkRegistration network = sim800l->getRegistrationStatus();
  while(network != REGISTERED_HOME && network != REGISTERED_ROAMING) {
    delay(1000);
    network = sim800l->getRegistrationStatus();
  }
  Serial.println(F("Network registration OK"));
  delay(1000);

  // Setup APN for GPRS configuration
  bool success = sim800l->setupGPRS(apn);
  while(!success) {
    success = sim800l->setupGPRS(apn);
    delay(5000);
  }
  Serial.println(F("GPRS config OK"));
}
void sendData(){
  // Establish GPRS connectivity (5 trials)
  bool connected = false;
  for(uint8_t i = 0; i < 5 && !connected; i++) {
    delay(1000);
    connected = sim800l->connectGPRS();
  }

  // Check if connected, if not reset the module and setup the config again
  if(connected) {
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
   if(rc == 200) {
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
  for(uint8_t i = 0; i < 5 && !connected; i++) {
    delay(1000);
    disconnected = sim800l->disconnectGPRS();
  }
  
  if(disconnected) {
    Serial.println(F("GPRS disconnected !"));
  } else {
    Serial.println(F("GPRS still connected !"));
  }

  // Go into low power mode
  bool lowPowerMode = sim800l->setPowerMode(MINIMUM);
  if(lowPowerMode) {
    Serial.println(F("Module in low power mode"));
  } else {
    Serial.println(F("Failed to switch module to low power mode"));
  }

  // End of program... wait...
  while(1);
}
