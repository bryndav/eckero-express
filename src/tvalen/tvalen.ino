#include <Arduino_LSM9DS1.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "TvalenDef.h"

#define GPS_RX_PIN 2
#define GPS_TX_PIN 3
#define SERVO_PIN 4
#define RELAY_PIN 5
#define RED 22     
#define BLUE 24     
#define GREEN 23

UART gpsSerial (digitalPinToPinName(GPS_RX_PIN), digitalPinToPinName(GPS_TX_PIN), NC, NC);
Servo steeringServo;
Adafruit_BNO055 bno = Adafruit_BNO055(55);

int STATE = WAIT_FOR_GPS;

//Navigation related variables
const double pi = 3.141592653589793;
const int earth_radius = 6371000;

struct GPSData gps_data = {" ", 0.0, 'F', 0.0, 'F', 0, 0, 0.0, 0.0, false};

struct pos travel_plan[] = {
  {59.299278, 18.032740},
  {59.299318, 18.031827},
  {59.299083, 18.031806},
  {59.299009, 18.032702},
  {59.299278, 18.032740}
};

struct pos curr_pos = {0.0, 0.0};
struct pos destination = {0.0, 0.0};
const int number_of_positions = sizeof travel_plan / sizeof travel_plan[0];
int pos_index = 0;

double distance_to_target = 99999.0;
int degree_diff;

//Motor related variables
pidData pid_steering = {0, 0.0, 0.0, 0.5, 0.0, 0.0, 200, 0, 0, 22, 0};

int servo_signal;
int steering;

//Sensor related variables
float magY, magX, magZ;
float imu_heading, imu_pitch, imu_roll;
float heading;

//Time related variables
unsigned long last_gps_reading, last_heading_reading, last_debug_print, current_time;
const int heading_reading_rate = 100;
const int gps_reading_rate = 1000;
const int debug_rate = 200;

void setup() {
  
  Serial.begin(9600);
  gpsSerial.begin(9600);
  
  steeringServo.attach(SERVO_PIN);
  steeringServo.write(90);

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(RED, OUTPUT);
  pinMode(BLUE, OUTPUT);
  pinMode(GREEN, OUTPUT);
  
  digitalWrite(GREEN, LOW);
  digitalWrite(BLUE, LOW);
  digitalWrite(RED, HIGH);
  digitalWrite(RELAY_PIN, LOW);
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    while(1) {
      Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
      delay(1000);
    }
  }
  
  delay(1000);
  bno.setExtCrystalUse(true);
}


void loop() {
  current_time = millis();

  if (current_time - last_heading_reading > heading_reading_rate){
    readIMUData();
    
    degree_diff = calcAngle(imu_heading, pid_steering.setpoint);
    steering = findTurnSide(imu_heading, pid_steering.setpoint);
    
    pidControl(&pid_steering, degree_diff, current_time);
    setSteering(pid_steering.control_signal, steering);
  
    last_heading_reading = current_time;
  }

  switch(STATE)
  {
    case WAIT_FOR_GPS:
      gps_data = getPos();

      if (gps_data.valid){
        updatePosition(gps_data);
        last_gps_reading = current_time;
        
        STATE = PLAN_COURSE;
      }

      break;

    case PLAN_COURSE:
      int NEXT_STATE;

      digitalWrite(RELAY_PIN, HIGH);
      digitalWrite(GREEN, LOW);
      digitalWrite(BLUE, HIGH);
      digitalWrite(RED, LOW);
      
      if (destination.longitude == 0.0 && destination.latitude == 0.0){
        destination = travel_plan[pos_index];

        NEXT_STATE = NORMAL_OPERATIONS;
        
      }else if (pos_index < (number_of_positions - 1)) {
        pos_index++;
        destination = travel_plan[pos_index];

        NEXT_STATE = NORMAL_OPERATIONS;
        
      }else {
        Serial.println("Reached final destination");
        
        NEXT_STATE = TARGET_REACHED;
      }
      
      pid_steering.setpoint = calcBearing(curr_pos, destination);
      distance_to_target = calcDistance(curr_pos, destination);

      STATE = NEXT_STATE;

      break;

    case NORMAL_OPERATIONS:

      if (current_time - last_gps_reading > gps_reading_rate) {
        gps_data = getPos();
        updatePosition(gps_data);
        last_gps_reading = current_time;
    
        distance_to_target = calcDistance(curr_pos, destination);
        pid_steering.setpoint = calcBearing(curr_pos, destination);
      }

      if (distance_to_target < 2.5) {
        STATE = PLAN_COURSE;
      }

      break;

    case TARGET_REACHED:

      digitalWrite(RELAY_PIN, LOW);

      // Reached final destination, stop
      while(1){
        digitalWrite(GREEN, LOW);
        digitalWrite(BLUE, LOW);
        digitalWrite(RED, LOW);

        delay(1000);

        digitalWrite(GREEN, HIGH);
        digitalWrite(BLUE, LOW);
        digitalWrite(RED, LOW);

        delay(1000);
      }

      break;
      
    default:
      digitalWrite(RELAY_PIN, LOW);
  }

  if (current_time - last_debug_print > debug_rate){
    debugPrint();

    last_debug_print = current_time;
  }
}
