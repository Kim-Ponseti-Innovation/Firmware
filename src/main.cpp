#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* Defines IC adresses */
#define BNO055_ADDRESS (55)

/* Assign a unique ID to BNO055 */
Adafruit_BNO055 bno = Adafruit_BNO055(BNO055_ADDRESS);

void setup() 
{
  Wire.begin();
  Serial.begin(115200);
  Serial.flush();

  /* Checks if BNO055 Sensor is detected */
  if(!bno.begin())
  {
    Serial.print("0");
    while(1);
  }
  delayMicroseconds(10);

  bno.setExtCrystalUse(true);
}

void loop() 
{ 
  /* Included to avoid buffer issues with the GUI */
  Serial.print("0,0,0,0,0,0,0,0,");

  /* Reads absolute orientation from each BNO055 sensor */
  sensors_event_t event; 
  bno.getEvent(&event);

  Serial.print(event.orientation.x, 4);
  Serial.print(", ");
  Serial.print(event.orientation.y, 4);
  Serial.print(", ");
  Serial.print(event.orientation.z, 4);
  Serial.print(", ");

  /* Reads accleration vector from each BNO055 sensor */
  imu::Vector<3> accl = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  Serial.print(accl.x(), 4);
  Serial.print(", ");
  Serial.print(accl.y(), 4);
  Serial.print(", ");
  Serial.print(accl.z(), 4);
  Serial.println("");

  delayMicroseconds(3);
}