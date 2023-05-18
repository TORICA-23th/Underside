// RP2040でいけるか確認するために藤田がガッと書いちゃったので，新入生に一からUnderside.inoに書いてもらう

#include <Adafruit_DPS310.h>
#include <math.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <TORICA_SD.h>

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
volatile float data_under_urm_altitude_m = 0;


void setup() {
  Serial1.setFIFOSize(1024);
  Serial.begin(460800);
  Serial1.begin(460800);

  pinMode(16, OUTPUT);
  pinMode(25, OUTPUT);

  Wire.setClock(100000);
  Wire.begin();

  Serial.println("DPS310");
  if (! dps.begin_I2C()) {             // Can pass in I2C address here
    //if (! dps.begin_SPI(DPS310_CS)) {  // If you want to use SPI
    Serial.println("Failed to find DPS");
    while (1) yield();
  }
  Serial.println("DPS OK!");
  dps.configurePressure(DPS310_32HZ, DPS310_16SAMPLES);
  dps.configureTemperature(DPS310_32HZ, DPS310_2SAMPLES);
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
    pinMode(16, LOW);
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
    pinMode(16, HIGH);
  }

  if (dps.temperatureAvailable() && dps.pressureAvailable()) {
    pinMode(25, LOW);
    dps.getEvents(&temp_event, &pressure_event);

    dps_pressure_m = pressure_event.pressure;
    dps_temperature_deg = temp_event.temperature;
    dps_altitude_m = (pow(1013.25 / dps_pressure_m, 1 / 5.257) - 1) * (dps_temperature_deg + 273.15) / 0.0065;
    sprintf(sendUART_BUF, "%.2f,%.2f,%.2f,%.2f\n", dps_pressure_m, dps_temperature_deg, dps_altitude_m, data_under_urm_altitude_m);
    SerialOUT.print(sendUART_BUF);

    pinMode(25, HIGH);
  }
}

void loop1() {
  digitalWrite(URTRIG, LOW);
  delay(1);
  digitalWrite(URTRIG, HIGH);
  unsigned long LowLevelTime = pulseIn(URECHO, LOW, 150000); //timeout=6m*100cm*50us
  if (LowLevelTime != 0)
  {
    unsigned int DistanceMeasured = LowLevelTime / 50; // every 50us low level stands for 1cm
    data_under_urm_altitude_m = (float)DistanceMeasured / 100.0;
  }
  for (int i = 0; i++; i < 10) {
    sd.flash();
    delay(10);
  }
}
