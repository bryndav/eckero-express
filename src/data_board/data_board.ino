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
const uint16_t t1_load = 0;
const uint16_t t1_comp = 39999; // Gives 20 ms interrupts 

unsigned long last_transmission = 0;
unsigned long last_debug_print = 0;

Sensors sensor_values = {0, 0, 0, 0.0, 0.0, 0.0};

float current_speed = 0.0;

int depth_offset = 540;
float distance_traveled = 0.0;

void 
setup ()
{
  const int fresh_water = 997;

  // Reset Timer1 control register A
  TCCR1A = 0;

  // Set prescaler to 256
  TCCR1B &= ~(1 << CS12);
  TCCR1B |= (1 << CS11);
  TCCR1B &= ~(1 << CS10);

  // Reset Timer1 and set compare value
  TCNT1 = t1_load;
  OCR1A = t1_comp;

  // Enable Timer1 compare interrupt
  TIMSK1 = (1 << OCIE1A);

  // Enable global interrupts
  sei();
  
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
  const int sampel_period = 20;
  const int transmission_period = 100;
  const int debug_period = 1000;
  unsigned long now = millis ();

  if (now - last_transmission > transmission_period){
    calcVelocity(&current_speed, sensor_values.acceleration);
    calcDistance(&distance_traveled, current_speed, sensor_values.acceleration);
    correctDepth(&sensor_values.depth, depth_offset);
    
    return_code = sendInt (PITCH_TRANS, &sensor_values.pitch);
    return_code = sendInt (DEPTH_TRANS, &sensor_values.depth);
    return_code = sendFloat (HEADING_TRANS, &sensor_values.heading);
    
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
  Serial.print("Depth: ");
  Serial.print(sensor_values.depth);
  Serial.print("\t\t\t");
  Serial.print("Pitch: ");
  Serial.print(sensor_values.pitch);
  Serial.print("\t\t\t");
  Serial.print("Heading: ");
  Serial.print(sensor_values.heading);
  Serial.print("\t\t\t");
  Serial.print("Temprature: ");
  Serial.print(sensor_values.temperature);
  Serial.println();
  Serial.println();
}

ISR(TIMER1_COMPA_vect){
  TCNT1 = t1_load;
  
  updateSensors ();
  readSensors (&sensor_values);
}
