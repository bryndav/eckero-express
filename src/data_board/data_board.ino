/*** Needed libraries ***/
#include <Wire.h>
#include <SoftwareSerial.h>
#include "NineAxesMotion.h"
#include "MS5837.h"
#include "DataBoardDef.h"

#define GPS_RX_PIN 3
#define GPS_TX_PIN 4

/*** Object initialization ***/
NineAxesMotion gyroSensor;
SoftwareSerial gps_serial(GPS_RX_PIN, GPS_TX_PIN);
MS5837 depthSensor;

/*** Global variables ***/
unsigned long now = 0;
unsigned long last_sample = 0;
unsigned long last_transmission = 0;
unsigned long last_debug_print = 0;

int pitch = 0;
int pressure = 0;
int depth = 0;
float heading = 0.0;
float temperature = 0.0;
float acceleration = 0.0;
float current_speed = 0.0;

int depth_offset = 540;
float distance_traveled = 0.0;

void 
setup ()
{
  const int fresh_water = 997;
  
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
  const int sample_period = 20;
  const int transmission_period = 100;
  const int debug_period = 1000;
  now = millis ();

  if ((now - last_sample) >= sample_period) {
      updateSensors ();
      readSensors (&pitch, &pressure, &depth, &heading, &temperature, &acceleration);
      depth = correctDepth(depth, depth_offset);
      current_speed = calcVelocity(current_speed, acceleration);
      distance_traveled = calcDistance(distance_traveled, current_speed, acceleration);
      last_sample = now;
  }

  if (now - last_transmission > transmission_period){
    Serial.print("Sending pitch: ");
    return_code = sendInt (PITCH_TRANS, &pitch);
    return_code = sendInt (DEPTH_TRANS, &depth);
    return_code = sendFloat (HEADING_TRANS, &heading);
    last_transmission = now;
  }

  if (now - last_debug_print > debug_period) {
    debugPrint();
    last_debug_print = now;
  }
}

void
debugPrint()
{
//  Serial.print("Milliseconds since startup: ");
//  Serial.println(now);
  Serial.print("Depth: ");
  Serial.print(depth);
  Serial.print("\t\t\t");
  Serial.print("Pitch: ");
  Serial.print(pitch);
  Serial.print("\t\t\t");
  Serial.print("Heading: ");
  Serial.print(heading);
  Serial.print("\t\t\t");
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println();
  Serial.println();
}
