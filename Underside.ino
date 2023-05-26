#include <Adafruit_DPS310.h>
#include <math.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <TORICA_SD.h>
#include <Adafruit_NeoPixel.h>

#define SerialIN  Serial1
#define SerialOUT Serial1

int cs_SD = 28;
TORICA_SD sd(cs_SD);
char SD_BUF[256];

Adafruit_DPS310 dps;
sensors_event_t temp_event, pressure_event;

int URECHO = 26; // PWM Output 0-50000US,Every 50US represent 1cm
int URTRIG = 27; // trigger pin

volatile float dps_pressure_m = 0;
volatile float dps_temperature_deg = 0;
volatile float dps_altitude_m = 0;
volatile float urm_altitude_m = 0;

int Power = 11;
int PIN  = 12;
#define NUMPIXELS 1

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  Serial1.setFIFOSize(1024);
  Serial.begin(460800);
  Serial1.begin(460800);

  pinMode(16, OUTPUT);
  pinMode(25, OUTPUT);
  pinMode(17, OUTPUT);

  delay(100);

  Wire.setClock(400000);
  Wire.begin();

  Serial.println("DPS310");
  while (! dps.begin_I2C()) {             // Can pass in I2C address here
    //if (! dps.begin_SPI(DPS310_CS)) {  // If you want to use SPI
    Serial.println("Failed to find DPS");
    delay(100);
  }
  Serial.println("DPS OK!");
  dps.configurePressure(DPS310_32HZ, DPS310_16SAMPLES);
  dps.configureTemperature(DPS310_32HZ, DPS310_2SAMPLES);

  pinMode(Power, OUTPUT);
  digitalWrite(Power, HIGH);
  pixels.begin();

  while (SerialIN.available()) {
    SerialIN.read();
  }

}

void setup1() {
  pinMode(URTRIG, OUTPUT);    // A low pull on pin COMP/TRIG
  digitalWrite(URTRIG, HIGH); // Set to HIGH
  pinMode(URECHO, INPUT);     // Sending Enable PWM mode command
  
  sd.begin();
}

const int readUART_BUF_SIZE = 256;
char readUART_BUF[256];
char sendUART_BUF[256];
void loop() {
  while (SerialIN.available()) {
    pixels.clear();
    pixels.setPixelColor(0, pixels.Color(255, 0, 0));
    pixels.show();
    int read_length = SerialIN.available();
    if (read_length >= readUART_BUF_SIZE - 1) {
      read_length = readUART_BUF_SIZE - 1;
    }
    SerialIN.readBytes(readUART_BUF, read_length);
    readUART_BUF[read_length] = '\0';
    sd.add_str(readUART_BUF);
    if (!SerialIN.available()) {
      delay(1);
    }

    pixels.clear();
    pixels.show();
  }

  if (dps.temperatureAvailable() && dps.pressureAvailable()) {
    pixels.clear();
    pixels.setPixelColor(0, pixels.Color(0, 0, 255));
    pixels.show();
    dps.getEvents(&temp_event, &pressure_event);

    dps_pressure_m = pressure_event.pressure;
    dps_temperature_deg = temp_event.temperature;
    dps_altitude_m = (pow(1013.25 / dps_pressure_m, 1 / 5.257) - 1) * (dps_temperature_deg + 273.15) / 0.0065;
    sprintf(sendUART_BUF, "%.2f,%.2f,%.2f,%.2f\n", dps_pressure_m, dps_temperature_deg, dps_altitude_m, urm_altitude_m);
    SerialOUT.print(sendUART_BUF);

    pixels.clear();
    pixels.show();
  }
  sd.flash();
}

void LEDwrite(int status) {
  digitalWrite(25, status);
  digitalWrite(17, status);
  digitalWrite(16, status);
}

void loop1() {
  LEDwrite(LOW);
  digitalWrite(URTRIG, LOW);
  delay(1);
  LEDwrite(HIGH);
  digitalWrite(URTRIG, HIGH);
  unsigned long LowLevelTime = pulseIn(URECHO, LOW);
  if (LowLevelTime <= 25000) { //50*500cm
    unsigned int DistanceMeasured = LowLevelTime / 50; // every 50us low level stands for 1cm
    urm_altitude_m = (float)DistanceMeasured / 100.0;
  }
}
