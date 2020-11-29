/*** Needed libraries ***/
#include <Wire.h>
#include "NineAxesMotion.h"
#include "MS5837.h"

#define batteryPin A0

/*** Object initialization ***/
NineAxesMotion gyroSensor;
MS5837 depthSensor;

/*** Global variables ***/
unsigned long now = 0;
unsigned long lastSampel = 0;
unsigned long lastTransmission = 0;
const int sampelPeriod = 20;
const int transmissionPeriod = 100;

int pitch = 0;
int pitch_offset = 0;
float heading = 0.0;
float depth = 0.0;
float depth_offset = 46.0;
float voltage_level = 0.0;

char pitch_id = 'P';
char heading_id = 'H';
char depth_id = 'D';

const int fresh_water = 997;

void 
setup ()
{
  Serial.begin (115200);
  I2C.begin ();
  Wire.begin ();

  gyroSensor.initSensor ();
  gyroSensor.setOperationMode (OPERATION_MODE_NDOF);
  gyroSensor.setUpdateMode (MANUAL);

  depthSensor.init ();
  depthSensor.setModel (MS5837::MS5837_30BA);
  depthSensor.setFluidDensity (fresh_water);
  delay (5000);
  Serial.println ("Sensors initialized");
}

void 
loop ()
{
  int return_code;
  
  now = millis ();

  if ((now - lastSampel) >= sampelPeriod) {
      lastSampel = now;
      updateSensors ();
      readSensors (&pitch, &depth, &heading, &voltage_level);
  }

  if (now - lastTransmission > transmissionPeriod){
    return_code = sendInt (pitch_id, &pitch);
    return_code = sendFloat (depth_id, &depth);
    return_code = sendFloat (heading_id, &heading);
  }
}

void 
updateSensors ()
{
  gyroSensor.updateEuler ();
  gyroSensor.updateCalibStatus ();
  depthSensor.read ();
}

void
readSensors (int*     pitch, 
             float*   depth, 
             float*   heading,
             float*   voltage_level)
{
  int battery_level = analogRead(batteryPin);
  *voltage_level = battery_level * (5.0 / 1023.0); // multiply with 11.02 ???
  
  if (*pitch != (int) gyroSensor.readEulerRoll ())
      *pitch = ((int) gyroSensor.readEulerRoll () - pitch_offset);

  if (*depth != depthSensor.depth ())
    *depth = (depthSensor.depth () - depth_offset);

  if (*heading != gyroSensor.readEulerHeading ())
    *heading = gyroSensor.readEulerHeading ();
}
