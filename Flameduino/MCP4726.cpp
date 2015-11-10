
#include "MCP4726.h"
#include <Wire.h>

MCP4726::MCP4726() {
}

void MCP4726::begin(uint8_t addr) {
  _i2caddr = addr;
  Wire.begin();
}

void MCP4726::begin(uint8_t addr, bool retain_default) {
  _i2caddr = addr;
  _retain_default = retain_default;
  Wire.begin();
}

void MCP4726::setRetainDefault(bool retain_default) {
  _retain_default = retain_default;
}

bool MCP4726::setOutput(uint16_t output)
{
  return setOutput(output, _retain_default);
}

bool MCP4726::setOutput(uint16_t output, bool retain)
{
  uint8_t twbrback = TWBR;
  TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz
  Wire.beginTransmission(_i2caddr);
  if (retain)
  {
    Wire.write(MCP4726_RETAIN);
  }
  else
  {
    Wire.write(MCP4726_VOLATILE);
  }
  Wire.write(output / 16);                   // Upper data bits (D11.D10.D9.D8.D7.D6.D5.D4)
  Wire.write((output % 16) << 4);            // Lower data bits (D3. D2. D1.D0.xx.xx.xx.xx)
  Wire.endTransmission();
  TWBR = twbrback;
  return true;
}

bool MCP4726::setVoltage(float voltage)
{
  return setVoltage(voltage, _retain_default);
}

bool MCP4726::setVoltage(float voltage, bool retain)
{
  uint16_t output = voltage / VOLTAGE_REFERENCE * MAX_VALUE;
  if (output > MAX_VALUE) 
  { 
    output = MAX_VALUE; 
  }
  else if (output < 0) 
  { 
    output = 0; 
  }
  return setOutput(output, retain);
}

bool MCP4726::setPercentage(float percentage)
{
  return setVoltage(percentage, _retain_default);
}

bool MCP4726::setPercentage(float percentage, bool retain)
{
  uint16_t output = percentage / 100 * MAX_VALUE;
  if (output > MAX_VALUE) 
  { 
    output = MAX_VALUE; 
  }
  else if (output < 0) 
  { 
    output = 0; 
  }
  return setOutput(output, retain);
}



