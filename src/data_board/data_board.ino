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
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
MS5837 depthSensor;

/*** Global variables ***/
unsigned long last_sampel = 0;
unsigned long last_transmission = 0;

int pitch = 0;
int pressure = 0;
int depth = 0.0;
float heading = 0.0;
float temperature = 0.0;
float acceleration = 0.0;
float current_speed = 0.0;
int depth_offset = 540;
float distance_traveled = 0.0;
bool gps_requested = false;

void 
setup ()
{
  const int i2c_address = 20;
  const int fresh_water = 997;
  
  Serial.begin (115200);
  I2C.begin ();
  Wire.begin (i2c_address);
  Wire.onReceive (receiveEvent);
  
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
  const int sampel_period = 20;
  const int transmission_period = 100;
  const unsigned int position_timeout = 120000;
  unsigned long now = millis ();

  if ((now - last_sampel) >= sampel_period) {
      updateSensors ();
      readSensors (&pitch, &pressure, &depth, &heading, &temperature, &acceleration);
      depth = correctDepth(depth, depth_offset);
      current_speed = calcVelocity(current_speed, acceleration);
      distance_traveled = calcDistance(distance_traveled, current_speed, acceleration);
      last_sampel = now;
  }

  if (now - last_transmission > transmission_period){
    return_code = sendInt (PITCH_TRANS, &pitch);
    return_code = sendInt (DEPTH_TRANS, &depth);
    return_code = sendFloat (HEADING_TRANS, &heading);
    last_transmission = now;
  }

  if (gps_requested){
    unsigned long req_time_stamp = now;
    Position curr_pos = {" ", 0.0, 'F', 0.0, 'F', 0, 0, 0.0, 0.0, false};

    // Wait for a valid position
    while (curr_pos.valid == false && (now - req_time_stamp >= position_timeout)){
      curr_pos = getPos();
      now = millis();
    }

    // Send position if available
    if (curr_pos.valid){
      return_code = sendFloat(LONG_TRANS, &curr_pos.longitude);
      return_code = sendFloat(LAT_TRANS, &curr_pos.latitude);
    } else {
      return_code = sendByte(GPS_UNAVAILABLE);
    }

    gps_requested = false;
  }
}
