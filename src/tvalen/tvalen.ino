#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "TvalenDef.h"

#define GPS_RX_PIN 2
#define GPS_TX_PIN 3
#define SERVO_PIN 4
#define RELAY_PIN 5
#define RED 22     
#define BLUE 24     
#define GREEN 23

Adafruit_BNO055 bno = Adafruit_BNO055(55);
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
pidData pid_steering = {0, 0.0, 0.0, 0.5, 0.0, 0.0, 200, 0, 0, 35, 0};

int servo_signal;
int steering;

//Sensor related variables
uint8_t sys, gyro, accel, mag;
float imu_heading, inclanation;
float heading;

//Time related variables
unsigned long last_gps_reading, last_heading_reading, last_debug_print, last_radio_poll;
unsigned long current_time;
const int heading_reading_rate = 250;
const int gps_reading_rate = 1000;
const int debug_rate = 1000;
const int radio_poll_rate = 1000;

//Radio com variables
bool radio_state_switch = false;

void setup() {
  
  Serial.begin(9600);
  Serial1.begin(9600);
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

  while(Serial1.available()){
    Serial1.read();
  }

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);
  bno.setExtCrystalUse(true);
  sys = gyro = accel = mag = 0;

  //Connect gps points
  pos_1.next = &pos_2;
}


void loop() {
  current_time = millis();

  if (current_time - last_debug_print > debug_rate){
    int resp = 0;

    //debugPrint();
    radioCom();

    if (STATE != RADIO_CTRL) {
      resp = pollRadioRec();
  
      // If there is data in the serial recieve buffer, turn off motor and enter radio ctrl mode
      if(resp){
        Serial.println("Entering manual mode.");
        digitalWrite(RELAY_PIN, LOW);
        steeringServo.write(90);
        STATE = RADIO_CTRL;
      }
    }
    
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

      if (current_time - last_gps_reading > gps_reading_rate) {
        gps_data = getPos();
        updatePosition(gps_data);
    
        distance_to_target = calcDistance(curr_pos, *destination);
        pid_steering.setpoint = calcBearing(curr_pos, *destination);

        last_gps_reading = current_time;
      }

      if (current_time - last_heading_reading > heading_reading_rate){
        readHeading();
        
        degree_diff = calcAngle((int)imu_heading, (int)pid_steering.setpoint);
        steering = findTurnSide((int)imu_heading, (int)pid_steering.setpoint);
        
        pidControl(&pid_steering, degree_diff, current_time);
        setSteering(pid_steering.control_signal, steering);

        last_heading_reading = current_time;
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
          Serial1.println("Reached final destination....");
          Serial.println("Reached final destination....");

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

    case RADIO_CTRL:
      char instruction;

      if (current_time - last_heading_reading > heading_reading_rate){
        readHeading();
        
        last_heading_reading = current_time;
      }

      if (current_time - last_gps_reading > gps_reading_rate) {
        gps_data = getPos();
        updatePosition(gps_data);
    
        distance_to_target = calcDistance(curr_pos, *destination);
        pid_steering.setpoint = calcBearing(curr_pos, *destination);

        last_gps_reading = current_time;        
      }
      
      if (Serial1.available()){
        instruction = recieveInstruction();
        actOnInstruction(instruction);
      }

      if(radio_state_switch){
        STATE = NORMAL_OPERATIONS;
        digitalWrite(RELAY_PIN, HIGH);
        radio_state_switch = false;
      }

      break;
      
    default:
      digitalWrite(RELAY_PIN, LOW);
  }
}
