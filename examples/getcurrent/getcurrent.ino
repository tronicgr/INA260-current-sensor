// This example was writen for Arduino Zero and uses two serial ports, the USB and the SERIAL1. 
// Most of the messages will appear on the USB port fine.

#define Serial Serial1
#include "Wire.h"
#include <Thanos_INA260.h>

Thanos_INA260 ina260;

void setup() {
  byte error, address;
  int nDevices;

  Serial.begin(115200);  // Initialize hardware serial port, pins 0/1
  SerialUSB.begin(115200); // Initialize the USB serial port
  Wire.begin();

#ifndef ESP8266
    while (!SerialUSB);     // will pause Zero, Leonardo, etc until serial console opens
  #endif
  
  SerialUSB.println("Scanning...");
  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0)
    {
      if (address < 127) {
        SerialUSB.print("I2C device found at address 0x");
        SerialUSB.print(address, HEX);
        SerialUSB.println("  !");
        Serial.print(" 0x");
        Serial.print(address, HEX);
        Serial.print(",  ");
        nDevices++;
      }
    }
    else if (error == 4)
    {
      if (address < 127) {
        SerialUSB.print("Unknow error at address 0x");
        SerialUSB.println(address, HEX);
      }
    }
  }
  if (nDevices == 0) {
    SerialUSB.println("No I2C devices found\n");
    Serial.println("No I2C devices found\n");
  }
  else
  { SerialUSB.println("done\n");
    Serial.print(" done\r");
  }

 // Initialize the INA260.
   ina260.begin();
  SerialUSB.print(" INA260 started\r");
  SerialUSB.println("Measuring voltage and current with INA260 ...");
}

void loop() {
  float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;

  shuntvoltage = ina260.getShuntVoltage_mV();
  busvoltage = ina260.getBusVoltage_V();
  current_mA = ina260.getCurrent_mA();
   
  SerialUSB.print("Bus Voltage:   "); SerialUSB.print(busvoltage); SerialUSB.print(" V , \t");
  SerialUSB.print("Shunt Voltage: \t"); SerialUSB.print(shuntvoltage); SerialUSB.print(" mV \t");
  SerialUSB.print("Current:       \t"); SerialUSB.print(current_mA); SerialUSB.println(" mA");

  delay(10); //small delay for the terminal
}
