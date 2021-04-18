#include <Arduino_LSM9DS1.h>
#include <Servo.h>
//#include <MadgwickAHRS.h>
#include "SensorFusion.h"
#include "TvalenDef.h"

#define GPS_RX_PIN 2
#define GPS_TX_PIN 3
#define SERVO_PIN 4
#define RELAY_PIN 5
#define RED 22     
#define BLUE 24     
#define GREEN 23

UART gpsSerial (digitalPinToPinName(GPS_RX_PIN), digitalPinToPinName(GPS_TX_PIN), NC, NC);  // create a hardware serial port named mySerial with RX: pin 13 and TX: pin 8
SF filter;
//Madgwick filter;
Servo steeringServo;

int STATE = WAIT_FOR_GPS;

//Navigation related variables
const double pi = 3.141592653589793;
const int earth_radius = 6371000;

struct GPSData gps_data = {" ", 0.0, 'F', 0.0, 'F', 0, 0, 0.0, 0.0, false};

struct pos pos_1 = {59.302723, 18.037233, NULL};
struct pos pos_2 = {59.303305, 18.038316, NULL};
struct pos curr_pos = {0.0, 0.0, NULL};
struct pos* destination = NULL;

double og_distance_to_target;
double distance_to_target;
int degree_diff;

//Motor related variables
pidData pid_steering = {0, 0.0, 0.0, 0.5, 0.0, 0.0, 200, 0, 0, 22, 0};

int servo_signal;
int steering;

//Sensor related variables
const float sensorRate = 119;

float igx, igy, igz, iax, iay, iaz, imx, imy, imz;
float gx, gy, gz, ax, ay, az, mx, my, mz;
float pitch, roll, imu_heading, deltat;
float heading;

//Time related variables
unsigned long last_gps_reading, last_debug_print;
unsigned long current_time;
unsigned long micros_per_reading;
unsigned long micros_previous;
const int gps_reading_rate = 1000;
const int debug_rate = 1000;

void setup() {
  
  Serial.begin(9600);
  gpsSerial.begin(9600);

//  filter.begin(sensorRate);
//  micros_per_reading = 1000000 / 119;
  
  
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

  IMU.setAccelFS(3);
  IMU.setAccelODR(5);
  IMU.setAccelOffset(0.005054, -0.005799, -0.012964);
  IMU.setAccelSlope(0.998262, 1.000600, 1.000083);
  
  IMU.setGyroFS(2);
  IMU.setGyroODR(5);
  IMU.setGyroOffset (-0.289093, 0.215729, -0.071218);
  IMU.setGyroSlope (1.159825, 1.136199, 1.143236);
  
  IMU.setMagnetFS(0);
  IMU.setMagnetODR(8);
  IMU.setMagnetOffset(-8.621420, 11.906535, 1.370036);
  IMU.setMagnetSlope (1.230131, 1.603933, 1.566816); 

  //Connect gps points
  pos_1.next = &pos_2;
}


void loop() {
  current_time = millis();

  readIMU();

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
        destination = &pos_1;
      }

      pid_steering.setpoint = calcBearing(curr_pos, *destination);
      
      //Add five seconds delay for reading GPS position
      last_gps_reading = current_time + 5000;

      digitalWrite(RELAY_PIN, HIGH);
      
      digitalWrite(GREEN, LOW);
      digitalWrite(BLUE, HIGH);
      digitalWrite(RED, LOW);
      
      STATE = NORMAL_OPERATIONS;

      break;

    case NORMAL_OPERATIONS:

      if (current_time - last_gps_reading > gps_reading_rate) {
        gps_data = getPos();
        updatePosition(gps_data);
        last_gps_reading = current_time;
    
        distance_to_target = calcDistance(curr_pos, *destination);
        pid_steering.setpoint = calcBearing(curr_pos, *destination);
        
        degree_diff = calcAngle(heading, pid_steering.setpoint);
        steering = findTurnSide(heading, pid_steering.setpoint);
        pidControl(&pid_steering, degree_diff, current_time);
        setSteering(pid_steering.control_signal, steering);
      }

      if (distance_to_target < 3.0) {
        STATE = TARGET_REACHED;
      }

      break;

    case TARGET_REACHED:
 
      if(destination->next != NULL){
        STATE = PLAN_COURSE;
      }else {
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

void readSerial() {
  while(Serial.available()) {
    char val[12] = "";
    Serial.readBytesUntil(0x0a, val, 12);
    heading = atoi(val);
  }
}

void
debugPrint()
{
  Serial.print("State: ");
  switch(STATE) {
    case WAIT_FOR_GPS:
      Serial.println("WAIT_FOR_GPS");
      break;
    case MAG_OPERATIONS:
      Serial.println("MAG_OPERATIONS");
      break;
    case PLAN_COURSE:
      Serial.println("PLAN_COURSE");
      break;
    case NORMAL_OPERATIONS:
      Serial.println("NORMAL_OPERATIONS");
      break;
    case TARGET_REACHED:
      Serial.println("TARGET_REACHED");
      break;
  }
  
  Serial.print("Longitude: ");
  Serial.print(curr_pos.longitude, 6);
  Serial.print("\t\tLatitude: ");
  Serial.print(curr_pos.latitude, 6);
  Serial.print("\tHeading: ");
  Serial.println(heading);  

  Serial.print("Servo signal: ");
  Serial.print(pid_steering.control_signal);
  Serial.print("\t\tDirection: ");
  if(steering > 0) {
    Serial.println("Right");
  }else {
    Serial.println("Left");
  }

  Serial.print("Target longitude: ");
  Serial.print(destination->longitude, 6);
  Serial.print("\tTarget latitude: ");
  Serial.println(destination->latitude, 6);
  Serial.print("Distance to target: ");
  Serial.print(distance_to_target);
  Serial.print("\tBearing to target: ");
  Serial.print(pid_steering.setpoint);
  Serial.print("\tDegree diff: ");
  Serial.println(abs(pid_steering.setpoint - heading));

  Serial.println();
  Serial.println();
}
