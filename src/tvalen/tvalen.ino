#include <Arduino_LSM9DS1.h>
#include <Servo.h>
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

int STATE = WAIT_FOR_GPS;

//Navigation related variables
const double pi = 3.141592653589793;
const int earth_radius = 6371000;

struct GPSData gps_data = {" ", 0.0, 'F', 0.0, 'F', 0, 0, 0.0, 0.0, false};

struct pos pos_1 = {59.301336, 18.037656, NULL};
struct pos pos_2 = {59.301481, 18.037543, NULL};
struct pos curr_pos = {0.0, 0.0, NULL};
struct pos* destination = NULL;

double distance_to_target = 99999.0;
int degree_diff;

//Motor related variables
pidData pid_steering = {0, 0.0, 0.0, 0.5, 0.0, 0.0, 200, 0, 0, 22, 0};

int servo_signal;
int steering;

//Sensor related variables
float magY, magX, magZ;
float imu_heading, inclanation;
float heading;

//Time related variables
unsigned long last_gps_reading, last_heading_reading, last_debug_print;
unsigned long current_time;
const int heading_reading_rate = 250;
const int gps_reading_rate = 1000;
const int debug_rate = 1000;

void setup() {
  
  Serial.begin(9600);
  gpsSerial.begin(9600);
  
  steeringServo.attach(SERVO_PIN);
  steeringServo.write(90);

  pinMode (RELAY_PIN, OUTPUT);
  pinMode(RED, OUTPUT);
  pinMode(BLUE, OUTPUT);
  pinMode(GREEN, OUTPUT);
  
  digitalWrite(GREEN, LOW);
  digitalWrite(BLUE, LOW);
  digitalWrite(RED, HIGH);
  digitalWrite(RELAY_PIN, LOW);
  
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  
  IMU.setMagnetFS(0);
  IMU.setMagnetODR(8);
  IMU.setMagnetOffset(-8.621420, 11.906535, 1.370036);
  IMU.setMagnetSlope (1.230131, 1.603933, 1.566816); 
  IMU.magnetUnit = MICROTESLA;  //   GAUSS   MICROTESLA   NANOTESLA

  //Connect gps points
  pos_1.next = &pos_2;
}


void loop() {
  current_time = millis();

  if (current_time - last_debug_print > debug_rate){
    debugPrint();

    last_debug_print = current_time;
  }

  switch(STATE)
  {
    case WAIT_FOR_GPS:
      gps_data = getPos();

      if (gps_data.valid){
        updatePosition(gps_data);
        
        STATE = PLAN_COURSE;
      }

      break;

    case PLAN_COURSE:

      if (destination != NULL) {
        destination = destination->next;
      }else {
        Serial.println("Set position!");
        destination = &pos_1;
      }

      pid_steering.setpoint = calcBearing(curr_pos, *destination);
      last_gps_reading = current_time;

      digitalWrite(RELAY_PIN, HIGH);
      
      digitalWrite(GREEN, LOW);
      digitalWrite(BLUE, HIGH);
      digitalWrite(RED, LOW);
      
      STATE = NORMAL_OPERATIONS;

      break;

    case NORMAL_OPERATIONS:

      
      if (current_time - last_heading_reading > heading_reading_rate){
        readHeading();
        
        degree_diff = calcAngle(imu_heading, pid_steering.setpoint);
        steering = findTurnSide(imu_heading, pid_steering.setpoint);
        
        pidControl(&pid_steering, degree_diff, current_time);
        setSteering(pid_steering.control_signal, steering);

        last_heading_reading = current_time;
      }
    

      if (current_time - last_gps_reading > gps_reading_rate) {
        gps_data = getPos();
        updatePosition(gps_data);
        last_gps_reading = current_time;
    
        distance_to_target = calcDistance(curr_pos, *destination);
        pid_steering.setpoint = calcBearing(curr_pos, *destination);
      }

      if (distance_to_target < 3.0) {
        STATE = TARGET_REACHED;
      }

      break;

    case TARGET_REACHED:
 
      if(destination->next){
        STATE = PLAN_COURSE;
      }else {
          digitalWrite(RELAY_PIN, LOW);

          // Reached final destination, stop
          while(1){
            Serial.println("Reached final destination....");
            digitalWrite(GREEN, LOW);
            digitalWrite(BLUE, LOW);
            digitalWrite(RED, LOW);

            delay(1000);

            digitalWrite(GREEN, HIGH);
            digitalWrite(BLUE, LOW);
            digitalWrite(RED, LOW);

            delay(1000);
          }
      }

      break;
      
    default:
      digitalWrite(RELAY_PIN, LOW);
  }
}
