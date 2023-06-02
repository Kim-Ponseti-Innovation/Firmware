/* Set Serial to 57600 */
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* Defines IC adresses */
#define BNO055_ADDRESS (55)
#define FORCE_TACT_ADDRESS_1 (0x0A)
#define FORCE_TACT_ADDRESS_2 (0x0C)
#define FORCE_TACT_ADDRESS_3 (0x0B)
#define FORCE_TACT_ADDRESS_4 (0x0D)
#define FORCE_TACT_ADDRESS_5 (0x08)
#define FORCE_TACT_ADDRESS_6 (0x06)
#define FORCE_TACT_ADDRESS_7 (0x09)
#define FORCE_TACT_ADDRESS_8 (0x07)

/* Creates a list of Force Sensors to make sure they are working */
int IC_address_list[8] = {FORCE_TACT_ADDRESS_1, FORCE_TACT_ADDRESS_2, FORCE_TACT_ADDRESS_3, FORCE_TACT_ADDRESS_4, FORCE_TACT_ADDRESS_5, FORCE_TACT_ADDRESS_6, FORCE_TACT_ADDRESS_7, FORCE_TACT_ADDRESS_8};

/* Assign a unique ID to BNO055 */
Adafruit_BNO055 bno = Adafruit_BNO055(BNO055_ADDRESS);

/* Defines a function to read from Force Sensors */
short readDataFromSensor(short address)
{
  byte i2cPacketLength = 6; //i2c packet length. Just need 6 bytes from each slave
  byte outgoingI2CBuffer[3]; //outgoing array buffer
  byte incomingI2CBuffer[6]; //incoming array buffer

  outgoingI2CBuffer[0] = 0x01; //I2c read command
  outgoingI2CBuffer[1] = 128; //Slave data offset
  outgoingI2CBuffer[2] = i2cPacketLength; //require 6 bytes

  Wire.beginTransmission(address); // transmit to device 
  Wire.write(outgoingI2CBuffer, 3); // send out command
  byte error = Wire.endTransmission(); // stop transmitting and check slave status

  /* Checks for a conection if it does not work it returns -1 */
  if (error != 0) return -1; //if slave not exists or has error, return -1
  Wire.requestFrom(address, 6); //require 6 bytes from slave

  byte incomeCount = 0;
  while (incomeCount < i2cPacketLength) // slave may send less than requested
  {
    if (Wire.available())
    {
      incomingI2CBuffer[incomeCount] = Wire.read(); // receive a byte as character
      incomeCount++;
    }
    else
    {
      delayMicroseconds(10); //Wait 10us 
    }
  }

  short rawData = (incomingI2CBuffer[4] << 8) + incomingI2CBuffer[5]; //get the raw data

  return rawData;
}

void setup(void) 
{
  Wire.begin();
  Serial.begin(57600);
  Serial.flush();

   while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  /* Checks if BNO055 Sensor is detected */
  if(!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);

  bno.setExtCrystalUse(true);

  /* Checks to see if Singletact Sensors are working */
  for (int i = 0; i <= 7; i++) {
    while (readDataFromSensor(IC_address_list[i]) <= 0){
      Serial.print("Ooops, no SingleTact detected ... Check your wiring! Check sensor: ");
      Serial.println(IC_address_list[i]);
      delay(10000);
    }
  }
}

void loop(void) 
{ 
  /* Declares each variable for force data */
  short data1;
  short data2;
  short data3;
  short data4;
  short data5;
  short data6;
  short data7;
  short data8;

  /* Reads force from each SingleTact sensor */
  
  data1 = readDataFromSensor(FORCE_TACT_ADDRESS_1);
  Serial.print(data1);    
  Serial.print(", ");
  
  data2 = readDataFromSensor(FORCE_TACT_ADDRESS_2);
  Serial.print(data2);    
  Serial.print(", ");
    
  data3 = readDataFromSensor(FORCE_TACT_ADDRESS_3);
  Serial.print(data3);    
  Serial.print(", ");

  data4 = readDataFromSensor(FORCE_TACT_ADDRESS_4);
  Serial.print(data4);
  Serial.print(", ");

  data5 = readDataFromSensor(FORCE_TACT_ADDRESS_5);
  Serial.print(data5);    
  Serial.print(", ");

  data6 = readDataFromSensor(FORCE_TACT_ADDRESS_6);
  Serial.print(data6);    
  Serial.print(", ");

  data7 = readDataFromSensor(FORCE_TACT_ADDRESS_7);
  Serial.print(data7);    
  Serial.print(", ");

  data8 = readDataFromSensor(FORCE_TACT_ADDRESS_8);
  Serial.print(data8);    
  Serial.print(", ");

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

  delay(100);
  
}