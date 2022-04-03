#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
  
Adafruit_BNO055 bno = Adafruit_BNO055(55);
void setup(void) 
{
  Serial.begin(9600);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
    
  bno.setExtCrystalUse(true);
}

void loop() {
  // put your main code here, to run repeatedly:
 sensors_event_t event; 
  bno.getEvent(&event);
  
  /* Display the floating point data */
  Serial.print("X: ");
  Serial.print(event.orientation.x);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z);
  Serial.print("\t timestamp: ");
  Serial.println(event.timestamp);

  imu::Vector<3> accelormeter = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    /* Display the floating point data */
  Serial.print("accelormeter X: ");
  Serial.print(accelormeter.x());
  Serial.print("accelormeter Y: ");
  Serial.print(accelormeter.y());
  Serial.print("accelormeter Z: ");
  Serial.print(accelormeter.z());
  Serial.println("");
  //Serial.print(accelormeter);
  
  
  delay(100);
}
