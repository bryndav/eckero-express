#include <Wire.h>
#include <SoftwareSerial.h>
#include "NineAxesMotion.h"

NineAxesMotion gyroSensor;

float magX, magY, magZ, heading, e_heading;
float mag_avrage;
unsigned long last_sampel;
unsigned long last_debug_print;

void setup (){
  float mag_readings;
  
  Serial.begin(115200);
  I2C.begin ();
  
  gyroSensor.initSensor ();
  gyroSensor.setOperationMode (OPERATION_MODE_NDOF);
  gyroSensor.setUpdateMode (MANUAL);
  delay(5000);
  Serial.println("Sensor initialized..");

  for(int i = 0; i < 10; i++){
    gyroSensor.updateEuler();
    gyroSensor.updateMag();
    gyroSensor.updateCalibStatus ();

    magX = gyroSensor.readMagX();
    magY = gyroSensor.readMagY();
    magZ = gyroSensor.readMagZ();

    mag_readings += (atan2(magY, magX) * (180/M_PI));
    delay(1000);
  }

  mag_avrage = mag_readings/10.0;
  Serial.print("Mag avrage: ");
  Serial.println(mag_avrage);
}

void loop () {
  const int debug_rate = 1000;
  const int sampel_period = 20;
  unsigned long now = millis ();

  // Read sensor values
  if ((now - last_sampel) >= sampel_period) {  
    gyroSensor.updateEuler();
    gyroSensor.updateMag();
    gyroSensor.updateCalibStatus ();

    e_heading = gyroSensor.readEulerHeading() + mag_avrage;

    if (e_heading > 360){
      e_heading = e_heading - 360;
    }else if(e_heading < 0){
      e_heading = e_heading + 360;
    }
    
    magX = gyroSensor.readMagX();
    magY = gyroSensor.readMagY();
    magZ = gyroSensor.readMagZ();

    heading = atan2(magY, magX) * (180/M_PI);

    last_sampel = now;
  }

  if (now - last_debug_print > debug_rate) {
    debugPrint();
    
    last_debug_print = now;
  } 
  
}

void debugPrint(){
  Serial.print("Heading: ");
  Serial.print(heading);
  Serial.print("\t\t");
  Serial.print("Euler heading: ");
  Serial.print(e_heading);
  Serial.print("\t\t");
  Serial.print("Mag X: ");
  Serial.print(magX);
  Serial.print("\t\t");
  Serial.print("Mag Y: ");
  Serial.print(magY);
  Serial.print("\t\t");
  Serial.print("Mag Z: ");
  Serial.print(magZ);
  Serial.println("\t\t");
}
