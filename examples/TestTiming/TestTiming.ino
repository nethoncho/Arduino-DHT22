#include <DHT22.h>
 
 
#define DIRECT_READ(base, mask)		(((*(base)) & (mask)) ? 1 : 0)
#define DIRECT_MODE_INPUT(base, mask)	((*(base+1)) &= ~(mask))
#define DIRECT_MODE_OUTPUT(base, mask)	((*(base+1)) |= (mask))
#define DIRECT_WRITE_LOW(base, mask)	((*(base+2)) &= ~(mask))

const byte pin= 7;
uint8_t _bitmask;
volatile uint8_t * _baseReg;
uint8_t timesHi[45];
uint8_t timesLo[45];
DHT22 dht22;

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

void loop() {
  Serial.println("Start");
  uint8_t res= sense();
  Serial.print("Times Read: ");
  Serial.println(res);
  for(byte i= 0; i<45; i++ ) {
    Serial.print("# ");
    Serial.print(i);
    Serial.print("Time Lo: ");
    Serial.print(timesLo[i]*2);
    Serial.print(" Time Hi: ");
    Serial.println(timesHi[i]*2);
  }
  
  
  delay(3000);
  dht22.setPin(pin);
  DHT22_ERROR_t e= dht22.readData();
  if (e==DHT_ERROR_NONE) {
    Serial.print("T: ");
    Serial.print(dht22.getTemperatureC());
    Serial.print(", H: ");
    Serial.println(dht22.getHumidity());
  } else {
    Serial.print("E: ");
    Serial.println(e);    
  }
  while(1);
}

uint8_t sense() {
  _bitmask = digitalPinToBitMask(pin);
  _baseReg = portInputRegister(digitalPinToPort(pin));
  uint8_t bitmask = _bitmask;
  volatile uint8_t *reg asm("r30") = _baseReg;

  uint8_t retryCount;
  for(byte i= 0; i<45; i++ ) {
    timesHi[i]= 0;
    timesLo[i]= 0;
  }
  
  cli();
  DIRECT_MODE_INPUT(reg, bitmask);
  sei();
  retryCount = 0;
  do {
    if (retryCount > 125)
    {
      return 0;
    }
    retryCount++;
    delayMicroseconds(2);
  } while(!DIRECT_READ(reg, bitmask)); // while low
  
  // Send the activate pulse
  cli();
  DIRECT_WRITE_LOW(reg, bitmask);
  DIRECT_MODE_OUTPUT(reg, bitmask); // Output Low
  sei();
  delayMicroseconds(1100); // 1.1 ms
  cli();
  DIRECT_MODE_INPUT(reg, bitmask);	// Switch back to input so pin can float
  sei();
  
  for(byte i= 0; i<45; i++ ) {
    retryCount = 0;
    do {
      if (retryCount > 100) {
        return i+1;
      }
      
      retryCount++;
      delayMicroseconds(2);
    } while(!DIRECT_READ(reg, bitmask)); // while lo
    timesLo[i]= retryCount;

    retryCount = 0;
    do {
      if (retryCount > 100) {
        return i+1;
      }
      
      retryCount++;
      delayMicroseconds(2);
    } while(DIRECT_READ(reg, bitmask)); // while hi
    timesHi[i]= retryCount;
  }
  return 46;
}
