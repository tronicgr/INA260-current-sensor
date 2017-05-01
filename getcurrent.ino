#define Serial Serial1

#include "Wire.h"

#include <Thanos_INA260.h>

Thanos_INA260 ina260;

void setup() {
  // put your setup code here, to run once:

  byte error, address;
  int nDevices;

  Serial.begin(115200);
  SerialUSB.begin(115200); // Initialize hardware serial port, pins 0/1
  Wire.begin();

#ifndef ESP8266
    while (!SerialUSB);     // will pause Zero, Leonardo, etc until serial console opens
  #endif
  //delay(5000);


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


 // Initialize the INA219.
  // By default the initialization will use the largest range (32V, 2A).  However
  // you can call a setCalibration function to change this range (see comments).
  ina260.begin();
  SerialUSB.print(" INA260 started\r");
  // To use a slightly lower 32V, 1A range (higher precision on amps):
  //ina219.setCalibration_32V_1A();
  // Or to use a lower 16V, 400mA range (higher precision on volts and amps):
  //ina219.setCalibration_16V_400mA();

  SerialUSB.println("Measuring voltage and current with INA219 ...");


  

}

void loop() {
  // put your main code here, to run repeatedly:



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

  delay(10);


  

}