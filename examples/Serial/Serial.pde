#include <DHT22.h>

// Data wire is plugged into port 7 on the Arduino
// Connect a 4.7K resistor between VCC and the data pin (strong pullup)
#define DHT22_PIN 7

// Setup a DHT22 instance
DHT22 myDHT22(DHT22_PIN);

void setup(void)
{
  // start serial port
  Serial.begin(9600);
  Serial.println("DHT22 Library Demo");
}

void loop(void)
{ 
  DHT22_ERROR_t errorCode;

  delay(2000);
  Serial.print("Requesting data...");
  errorCode = myDHT22.readData();
  if(errorCode == DHT_ERROR_NONE)
  {
    Serial.print("Got Data ");
    Serial.print(myDHT22.getTemperatureC());
    Serial.print("C ");
    Serial.print(myDHT22.getHumidity());
    Serial.println("%");
  }
  else
  {
    Serial.print("Error Code ");
    Serial.print(errorCode);
    Serial.println(" readData Failed");
  }
}
