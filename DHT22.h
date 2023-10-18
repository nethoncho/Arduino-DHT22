#ifndef _DHT22_H_
#define _DHT22_H_

#include <inttypes.h>
#include <Arduino.h>

#define DHT22_ERROR_VALUE -995

typedef enum
{
  DHT_ERROR_NONE = 0,
  DHT_BUS_HUNG,
  DHT_ERROR_NOT_PRESENT,
  DHT_ERROR_ACK_TOO_LONG,
  DHT_ERROR_SYNC_TIMEOUT,
  DHT_ERROR_DATA_TIMEOUT,
  DHT_ERROR_CHECKSUM,
  DHT_ERROR_TOOQUICK
} DHT22_ERROR_t;

class DHT22
{
  private:
    void init();
    uint8_t _bitmask;
    volatile uint8_t *_baseReg;
    unsigned long _lastReadTime;
    short int _lastHumidity;
    short int _lastTemperature;
    
    inline bool levelTime(uint8_t level, unsigned long min, unsigned long max) {
        volatile uint8_t *reg asm("r30") = _baseReg;
        long t;
        
        t= micros();
        for(uint8_t i= 0;  (((*reg) & _bitmask) ? 1 : 0)==level && i<100; i++)
            delayMicroseconds(6);
        
        wrongTiming= micros() - t;
        
        return wrongTiming>min && wrongTiming<max;
    };
    
    
  public:
    unsigned long wrongTiming;
    DHT22();
    DHT22(uint8_t pin);
    void setPin(uint8_t pin);
    DHT22_ERROR_t readData();
    DHT22_ERROR_t readDataNow();
    short int getHumidityInt();
    short int getTemperatureCInt();
    void clockReset();
    // unsigned long getWrongTiming();
#if !defined(DHT22_NO_FLOAT)
    float getHumidity();
    float getTemperatureC();
    float getTemperatureF();
#endif
};

// Report the humidity in .1 percent increments, such that 635 means 63.5% relative humidity
inline short int DHT22::getHumidityInt()
{
  return _lastHumidity;
}

// Get the temperature in decidegrees C, such that 326 means 32.6 degrees C.
// The temperature may be negative, so be careful when handling the fractional part.
inline short int DHT22::getTemperatureCInt()
{
  return _lastTemperature;
}

// unsigned long DHT22::getWrongTiming() {
//     return wrongTiming;
// }


#if !defined(DHT22_NO_FLOAT)
// Return the percentage relative humidity in decimal form
// Converts from the internal integer format on demand, so you might want
// to cache the result.
inline float DHT22::getHumidity()
{
  return float(_lastHumidity) * (float)0.1;
}
#endif

#if !defined(DHT22_NO_FLOAT)
// Return the percentage relative humidity in decimal form
// Converts from the internal integer format on demand, so you might want
// to cache the result.
inline float DHT22::getTemperatureC()
{
  return float(_lastTemperature) * (float)0.1;
}

inline float DHT22::getTemperatureF()
{
  return float(_lastTemperature) * (float)0.1 * 9.0 / 5.0 + 32.0;
}
#endif //DHT22_SUPPORT_FLOAT

#endif /*_DHT22_H_*/
