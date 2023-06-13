
#ifndef DS18S20_h
#define DS18S20_h

#if defined(ARDUINO) && (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

class DS18S20{
public:
  /**
   * @fn read
   * @brief Read DHT11 data
   * @param pin: Connect the IO port of the DHT11 data port.
   * @return NONE    
   */
  void read(int pin);
  float temperature;
  int pin;
};
#endif
