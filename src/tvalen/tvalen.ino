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

struct pos travel_plan[] = {
  {59.309092, 17.978828},
  {59.309294, 17.978061},
  {59.309569, 17.977337},
  {59.309830, 17.976662},
  {59.310199, 17.975841},
  {59.310559, 17.975032},
  {59.310928, 17.974224},
  {59.311298, 17.973417},
  {59.311674, 17.972976},
  {59.312075, 17.972505},
  {59.312386, 17.972599},
  {59.312690, 17.972685},
  {59.312858, 17.973651},
  {59.313017, 17.974606},
  {59.313135, 17.975546},
  {59.313236, 17.976366},
  {59.313164, 17.976981},
  {59.313094, 17.977546},
  {59.312930, 17.977983},
  {59.312767, 17.978388},
  {59.312429, 17.978770},
  {59.312088, 17.979161},
  {59.311649, 17.979021},
  {59.311245, 17.978882},
  {59.310870, 17.978794},
  {59.310500, 17.978710},
  {59.309999, 17.978820},
  {59.309504, 17.978918},
  {59.309161, 17.979273},
  {59.308595, 17.979850}
};

struct pos curr_pos = {0.0, 0.0};
struct pos destination = {0.0, 0.0};
const int number_of_positions = sizeof travel_plan / sizeof travel_plan[0];
int pos_index = 0;

double distance_to_target = 99999.0;
int degree_diff;

//Motor related variables
pidData pid_steering = {0, 0.0, 0.0, 1.5, 0.0, 0.0, 200, 0, 0, 35, 0};

int servo_signal;
int steering;

//Sensor related variables
uint8_t sys, gyro, accel, mag;
float imu_heading, inclanation;
float heading;

//Time related variables
unsigned long last_gps_reading, last_heading_reading, last_debug_print, last_radio_poll, current_time;
const int heading_reading_rate = 250;
const int gps_reading_rate = 1000;
const int debug_rate = 2000;
const int radio_poll_rate = 50;

//Radio com variables
bool radio_state_switch = false;

void setup() {
  
  Serial.begin(9600);
  Serial1.begin(9600);
  gpsSerial.begin(9600);
  
  steeringServo.attach(SERVO_PIN);
  setSteering(0, 1);

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
}


void loop() {
  current_time = millis();

  if (current_time - last_debug_print > debug_rate){
    //debugPrint();
    radioCom();
    
    last_debug_print = current_time;
  }

  if (current_time - last_radio_poll > radio_poll_rate){
    byte resp = 0;
    byte instruction = 0;

    if (STATE != RADIO_CTRL) {
      resp = pollRadioRec();
      instruction = Serial1.read();
  
      // If there is data in the serial recieve buffer, turn off motor and enter radio ctrl mode
    if((instruction == 71) || (instruction == 83) || (instruction == 76) || (instruction == 82) || (instruction == 67) || (instruction == 65) || (instruction == 77)){
      Serial1.println("OK, Entering manual mode.");
      digitalWrite(RELAY_PIN, LOW);
      setSteering(0, 1);
      STATE = RADIO_CTRL;
    }else{
      while(Serial1.available()){
        Serial1.read();
      }
    }
  }
    
  last_radio_poll = current_time;
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
      int next_state;

      digitalWrite(RELAY_PIN, HIGH);
      digitalWrite(GREEN, LOW);
      digitalWrite(BLUE, HIGH);
      digitalWrite(RED, LOW);

      if (destination.longitude == 0.0 && destination.latitude == 0.0) {
        destination = travel_plan[pos_index];

        next_state = NORMAL_OPERATIONS;
      }else if (pos_index < (number_of_positions - 1)) {
        pos_index++;
        destination.longitude = travel_plan[pos_index].longitude;
        destination.latitude = travel_plan[pos_index].latitude;

        next_state = NORMAL_OPERATIONS;
      }else {
        Serial.println("Reached final destination");
        digitalWrite(RELAY_PIN, LOW);
        setSteering(0, 1);
        next_state = RADIO_CTRL;
      }

      distance_to_target = calcDistance(curr_pos, destination);
      pid_steering.setpoint = calcBearing(curr_pos, destination);

      last_gps_reading = current_time;
      
      STATE = next_state;

      break;

    case NORMAL_OPERATIONS:

      if (current_time - last_gps_reading > gps_reading_rate) {
        gps_data = getPos();
        updatePosition(gps_data);
    
        distance_to_target = calcDistance(curr_pos, destination);
        pid_steering.setpoint = calcBearing(curr_pos, destination);

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

      if (distance_to_target < 5.0) {
        STATE = PLAN_COURSE;
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
    
        distance_to_target = calcDistance(curr_pos, destination);
        pid_steering.setpoint = calcBearing(curr_pos, destination);

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
