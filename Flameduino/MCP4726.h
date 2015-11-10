
#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#ifndef MCP4726_H_
#define MCP4726_H_

#define MCP4726_VOLATILE  (0x40) // Writes data to the DAC, lost on power loss
#define MCP4726_RETAIN    (0x60) // Writes data to the DAC, retain on power loss
#define VOLTAGE_REFERENCE 5.0    // Voltage Reference
#define MAX_VALUE         4095   // 12 bit DAC: 0 - 4095

class MCP4726 {
  
 public:
  MCP4726();
  void begin(uint8_t addr);
  void begin(uint8_t addr, bool retain_default);
  bool setOutput(uint16_t output);
  bool setOutput(uint16_t output, bool retain);
  bool setVoltage(float voltage);
  bool setVoltage(float voltage, bool retain);
  bool setPercentage(float percentage);
  bool setPercentage(float percentage, bool retain);
  void setRetainDefault(bool retain_default);

 private:
  uint8_t _i2caddr;
  bool _retain_default;
  
};

#endif

