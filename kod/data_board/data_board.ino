/*** Needed libraries ***/
#include <Wire.h>
#include "NineAxesMotion.h"
#include "MS5837.h"

/*** Object initialization ***/
NineAxesMotion gyroSensor;
MS5837 depthSensor;

/*** Global variables ***/
unsigned long now = 0;
unsigned long lastSampel = 0;
unsigned long lastTransmission = 0;
const int sampelPeriod = 20;
const int transmissionPeriod = 100;
unsigned long lastDebugPrint = 0;

int pitch = 0;
int pitch_offset = 0;
float heading = 0.0;
float depth = 0.0;
float depth_offset = 0.0;
float acceleration = 0.0;
float current_speed = 0.0;

float distance_traveled = 0.0;

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
  const int debugTime = 1000;
  
  now = millis ();

  if ((now - lastSampel) >= sampelPeriod) {
      lastSampel = now;
      updateSensors ();
      readSensors (&pitch, &depth, &heading, &acceleration);
      current_speed = calcVelocity(current_speed, acceleration);
      distance_traveled = calcDistance(distance_traveled, current_speed, acceleration);
  }
  /*
  if (now - lastTransmission > transmissionPeriod){
    return_code = sendInt (pitch_id, &pitch);
    return_code = sendFloat (depth_id, &depth);
    return_code = sendFloat (heading_id, &heading);
  }
  */
  if ((now - lastDebugPrint) >= debugTime) {
    Serial.print("Distance traveled: ");
    Serial.println(distance_traveled);
    lastDebugPrint = now;
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
             float*   acceleration)
{
  *acceleration = gyroSensor.readAccelerometer(X_AXIS);
  
  if (*pitch != (int) gyroSensor.readEulerRoll ())
      *pitch = (int) gyroSensor.readEulerRoll () - pitch_offset;

  if (*depth != depthSensor.depth ())
    *depth = depthSensor.depth () - depth_offset;

  if (*heading != gyroSensor.readEulerHeading ())
    *heading = gyroSensor.readEulerHeading ();
}

float
calcVelocity (float prev_velocity,
              float acceleration)
{
  float return_val;
  float samplePeriod = 0.0020;

  return_val = prev_velocity + (acceleration * samplePeriod);

  return return_val;
}

float
calcDistance (float prev_distance,
              float current_speed,
              float acceleration)
{
  float return_val;
  float samplePeriod = 0.0020;

  return_val = prev_distance + (current_speed * samplePeriod) + 0,5 * (acceleration * (samplePeriod * samplePeriod));

  return return_val;
}
