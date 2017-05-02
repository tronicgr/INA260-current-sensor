/**************************************************************************/
/*! 
    @file     Thanos_INA260.cpp
    @author   Thanos Kontogiannis 
	@license  BSD (see license.txt)
	
	Driver for the INA260 current sensor
	This is a library for the INA260 
		
	@section  HISTORY

    v1.0 - First release
*/
/**************************************************************************/
#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Wire.h>

#include "Thanos_INA260.h"

/**************************************************************************/
/*! 
    @brief  Sends a single command byte over I2C
*/
/**************************************************************************/
void Thanos_INA260::wireWriteRegister (uint8_t reg, uint16_t value)
{
  Wire.beginTransmission(ina260_i2caddr);
  #if ARDUINO >= 100
    Wire.write(reg);                       // Register
    Wire.write((value >> 8) & 0xFF);       // Upper 8-bits
    Wire.write(value & 0xFF);              // Lower 8-bits
  #else
    Wire.send(reg);                        // Register
    Wire.send(value >> 8);                 // Upper 8-bits
    Wire.send(value & 0xFF);               // Lower 8-bits
  #endif
  Wire.endTransmission();
}

/**************************************************************************/
/*! 
    @brief  Reads a 16 bit values over I2C
*/
/**************************************************************************/
void Thanos_INA260::wireReadRegister(uint8_t reg, uint16_t *value)
{

  Wire.beginTransmission(ina260_i2caddr);
  #if ARDUINO >= 100
    Wire.write(reg);                       // Register
  #else
    Wire.send(reg);                        // Register
  #endif
  Wire.endTransmission();
  
  delay(1); // Max 12-bit conversion time is 586us per sample

  Wire.requestFrom(ina260_i2caddr, (uint8_t)2);  
  #if ARDUINO >= 100
    // Shift values to create properly formed integer
    *value = ((Wire.read() << 8) | Wire.read());
  #else
    // Shift values to create properly formed integer
    *value = ((Wire.receive() << 8) | Wire.receive());
  #endif
}


/**************************************************************************/
/*! 
    @brief  Configures to INA260
			
*/
/**************************************************************************/
void Thanos_INA260::setConfigRegister(void)
{
  // Sets 4 samples average and sampling time for voltage and current to 8.244ms
    
  // Set Config register 
  uint16_t config = INA260_CONFIG_AVGRANGE_4 |
                    INA260_CONFIG_BVOLTAGETIME_8244US |
                    INA260_CONFIG_SCURRENTTIME_8244US |
                    INA260_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
  wireWriteRegister(INA260_REG_CONFIG, config);
}


/**************************************************************************/
/*! 
    @brief  Instantiates a new INA260 class
*/
/**************************************************************************/
Thanos_INA260::Thanos_INA260(uint8_t addr) {
  ina260_i2caddr = addr;
}

/**************************************************************************/
/*! 
    @brief  Setups the HW 
*/
/**************************************************************************/
void Thanos_INA260::begin(uint8_t addr) {
  ina260_i2caddr = addr;
  begin();
}

void Thanos_INA260::begin(void) {
  Wire.begin();    
  // Set chip to large range config values to start
  setConfigRegister();
}

/**************************************************************************/
/*! 
    @brief  Gets the raw bus voltage (16-bit signed integer, so +-32767)
*/
/**************************************************************************/
int16_t Thanos_INA260::getBusVoltage_raw() {
  uint16_t value;
  wireReadRegister(INA260_REG_BUSVOLTAGE, &value);
  return (int16_t)value;
}

/**************************************************************************/
/*! 
    @brief  Gets the raw shunt voltage (16-bit signed integer, so +-32767)
*/
/**************************************************************************/
int16_t Thanos_INA260::getShuntVoltage_raw() {
  uint16_t value;
  wireReadRegister(INA260_REG_SHUNTVOLTAGE, &value);
  return (int16_t)value;
}

/**************************************************************************/
/*! 
    @brief  Gets the raw current value (16-bit signed integer, so +-32767)
*/
/**************************************************************************/
int16_t Thanos_INA260::getCurrent_raw() {
  uint16_t value;
  wireReadRegister(INA260_REG_CURRENT, &value);
  return (int16_t)value;
}
 
/**************************************************************************/
/*! 
    @brief  Gets the shunt voltage in mV
*/
/**************************************************************************/
float Thanos_INA260::getShuntVoltage_mV() {
  int16_t value;
  value = getShuntVoltage_raw();
  return value * 0.007;
}

/**************************************************************************/
/*! 
    @brief  Gets the shunt voltage in volts
*/
/**************************************************************************/
float Thanos_INA260::getBusVoltage_V() {
  int16_t value = getBusVoltage_raw();
  return value * 0.00125;
}

/**************************************************************************/
/*! 
    @brief  Gets the current value in mA
*/
/**************************************************************************/
float Thanos_INA260::getCurrent_mA() {
  float valueDec = getCurrent_raw();
  return valueDec * 1.25;
}
