/*** Needed libraries ***/
#include <Wire.h>
#include <SoftwareSerial.h>
#include "NineAxesMotion.h"
#include "MS5837.h"
#include "AlenDef.h"

#define GPS_RX_PIN 3
#define GPS_TX_PIN 4
#define RIGHT_MOTOR_PIN 6
#define LEFT_MOTOR_PIN 10
#define FRONT_MOTOR_PIN 9
#define REAR_MOTOR_PIN 11
#define RIGHT_MOTOR_DIR 7
#define LEFT_MOTOR_DIR 8
#define FRONT_MOTOR_DIR 13
#define REAR_MOTOR_DIR 12

/*** Object initialization ***/
NineAxesMotion gyroSensor;
SoftwareSerial gps_serial(GPS_RX_PIN, GPS_TX_PIN);
MS5837 depthSensor;

/*** Global variables ***/
const uint16_t t1_load = 0;
const uint16_t t1_comp = 39999; // Gives 20 ms interrupts 

// Timestamps
unsigned long rc_update = 0;
unsigned long last_motor_writing, last_debug_print, last_sampel;

// Operations
int right_motor_speed = 0;
int left_motor_speed = 0;
int rear_motor_speed;
int front_motor_speed;

float current_speed = 0.0;
float distance_traveled = 0.0;

const int pid_calc_rate = 100;
pidData pid_balance = {0, 0.0, 0.0, 1.0, 0.0, 0.0, pid_calc_rate, 0, 0.0, 90, -90};
pidData pid_dive = {8, 0.0, 0.0, 30.0, 0.0, 0.0, pid_calc_rate, 0, 0.0, 255, -255};

Sensors sensor_values = {0, 0, 0, 0.0, 0.0, 0.0};

void 
setup ()
{
  const int fresh_water = 997;

  Serial.begin (115200);
  I2C.begin ();

  pinMode (RIGHT_MOTOR_PIN, OUTPUT);
  pinMode (LEFT_MOTOR_PIN, OUTPUT);
  pinMode (FRONT_MOTOR_PIN, OUTPUT);
  pinMode (REAR_MOTOR_PIN, OUTPUT);
  pinMode (RIGHT_MOTOR_DIR, OUTPUT);
  pinMode (LEFT_MOTOR_DIR, OUTPUT);
  pinMode (FRONT_MOTOR_DIR, OUTPUT);
  pinMode (REAR_MOTOR_DIR, OUTPUT);

  digitalWrite(RIGHT_MOTOR_DIR, HIGH);
  digitalWrite(LEFT_MOTOR_DIR, HIGH);
  digitalWrite(FRONT_MOTOR_DIR, LOW);
  digitalWrite(REAR_MOTOR_DIR, HIGH);
  
  gyroSensor.initSensor ();
  gyroSensor.setOperationMode (OPERATION_MODE_MAGONLY);
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
  const int debug_rate = 1000;
  const int motor_write_rate = 1000;
  const int sampel_period = 20;
  unsigned long now = millis ();

  // Read sensor values
  if ((now - last_sampel) >= sampel_period) {  
    updateSensors ();
    readSensors (&sensor_values);

    last_sampel = now;
  }

  //PID control signal calculations
  if (now - pid_balance.last_time >= pid_calc_rate){
    pidControl (&pid_balance, sensor_values.pitch, now);
    pidControl (&pid_dive, sensor_values.depth, now);
  }

  // Calculate and write motor signals
  if (now - last_motor_writing > motor_write_rate){
    getDiveOutput (&rear_motor_speed, &front_motor_speed, pid_dive.control_signal);     
    getBalanceReduction (&rear_motor_speed, &front_motor_speed, pid_balance.control_signal);

    checkDiveMotorOutput (pid_dive.setpoint, &front_motor_speed, &rear_motor_speed);
    
    setMotorSpeed (RIGHT_MOTOR_PIN, right_motor_speed);
    setMotorSpeed (LEFT_MOTOR_PIN, left_motor_speed);
    setMotorSpeed (REAR_MOTOR_PIN, rear_motor_speed);
    setMotorSpeed (FRONT_MOTOR_PIN, front_motor_speed);

    last_motor_writing = now;
  }

  if (now - last_debug_print > debug_rate) {
    readSerial();
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
  Serial.print("Set depth: ");
  Serial.print(pid_dive.setpoint);
  Serial.print("\t\t\t");
  Serial.print("Angle: ");
  Serial.print(sensor_values.pitch);
  Serial.print("\t\t\t");
  Serial.print("Heading: ");
  Serial.print(sensor_values.heading);
  Serial.print("\t\t\t");
  Serial.print("Temprature: ");
  Serial.println(sensor_values.temperature);
  Serial.print("Rear motor speed: ");
  Serial.print(rear_motor_speed); 
  Serial.print("\t\t");
  Serial.print("Front motor speed: ");
  Serial.print(front_motor_speed);
  Serial.print("\t\t");
  Serial.print("Left motor speed: ");
  Serial.print(left_motor_speed); 
  Serial.print("\t\t");
  Serial.print("Right motor speed: ");
  Serial.print(right_motor_speed);
  Serial.print("\t\t");
  Serial.print("Control signal: ");
  Serial.print(pid_dive.control_signal);
  Serial.println();
  Serial.println();
}
