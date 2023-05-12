#include <TimerTCC0.h>

#include <Adafruit_DPS310.h>
Adafruit_DPS310 dps;
sensors_event_t temp_event, pressure_event;

int URECHO = 0; // PWM Output 0-50000US,Every 50US represent 1cm
int URTRIG = 1; // trigger pin

float pressure_hPa = 0;
float temperature_deg = 0;
float altitude_pressure_m = 0;
float altitude_ultrasonic_m = 0;

char Serial_BUF[256];

void setup()
{
  Serial.begin(115200);

  pinMode(URTRIG, OUTPUT);    // A low pull on pin COMP/TRIG
  digitalWrite(URTRIG, HIGH); // Set to HIGH
  pinMode(URECHO, INPUT);     // Sending Enable PWM mode command

  Wire.setClock(400000);
  if (!dps.begin_I2C())
  { // Can pass in I2C address here
    // if (! dps.begin_SPI(DPS310_CS)) {  // If you want to use SPI
    Serial.println("Failed to find DPS");
    while (1)
      yield();
  }
  dps.configurePressure(DPS310_32HZ, DPS310_16SAMPLES);
  dps.configureTemperature(DPS310_32HZ, DPS310_2SAMPLES);

  TimerTcc0.initialize(10000);
  TimerTcc0.attachInterrupt(timerIsr);
}

void loop()
{
  digitalWrite(URTRIG, LOW);
  delay(1);
  digitalWrite(URTRIG, HIGH);
  unsigned long LowLevelTime = pulseIn(URECHO, LOW);
  if (LowLevelTime <= 50000)
  {
    unsigned int DistanceMeasured = LowLevelTime / 50; // every 50us low level stands for 1cm
    altitude_ultrasonic_m = (float)DistanceMeasured / 100.0;
  }

  // delay(200);
}
void timerIsr()
{
  if (dps.temperatureAvailable() && dps.pressureAvailable())
  {
    dps.getEvents(&temp_event, &pressure_event);
    temperature_deg = temp_event.temperature;
    pressure_hPa = pressure_event.pressure;
    // ToDo calc altitude
  }

  sprintf(Serial_BUF, "%f,%f,%f,%f\n", pressure_hPa, temperature_deg, altitude_pressure_m, altitude_ultrasonic_m);
  Serial.print(Serial_BUF);
}
