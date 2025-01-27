#include <Arduino.h>
#include <Wire.h>
#include <ArduinoCelestialCompass.h>

#define MUX_ADDR 0x70
#define N_UNITS 8
#define ADC_ADDR_1 0x40
#define ADC_ADDR_2 0x41

POL_OP units[N_UNITS];

void mux_change_channel(byte channel){
  // Form command byte
  if (channel > 7) return;
  byte writeval = 1 << channel;

  // Transmit channel-change byte
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(writeval);
  Wire.endTransmission();

  // Give a millisecond for channel change (think this can be smaller).
  delay(1);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial);

  //Start I2C interface
  Wire.begin();

  mux_change_channel(0);

  // Init POL_OP units
  for (int i = 0; i < N_UNITS; i++){
    units[i].initialise(Wire, ADC_ADDR_1, ADC_ADDR_2, true, i, Serial);
    delay(500);
  }

  // POL_OP test_unit;
  // test_unit.initialise(Wire, ADC_ADDR_1, ADC_ADDR_2, true, 0, Serial);

  // int readings[2];

  // size_t buffer_s = 50;
  // char buffer[buffer_s];
  // test_unit.read_sensor_half(readings, 25);
  // snprintf(buffer, buffer_s, "Unit %d: [ %d, %d ]", 0, readings[0], readings[1]);
  // Serial.println(buffer);
}

void loop() {
  // For each MUX Channel
  int readings[2];

  size_t buffer_s = 50;
  char buffer[buffer_s];

  for (int i = 0; i < 8; i++) {
    mux_change_channel(i);
    units[i].read_sensor_half(readings, 100);
    snprintf(buffer, buffer_s, "Unit %d: [ %d, %d ]", i, readings[0], readings[1]);
    Serial.println(buffer);
    
  }
  delay(1000);
}
