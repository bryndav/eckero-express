/*** Needed libraries ***/
#include <Wire.h>
#include "LogicBoardDef.h"

/*** Defenitions ***/

#define RIGHT_MOTOR_PIN 6
#define LEFT_MOTOR_PIN 10
#define FRONT_MOTOR_PIN 9
#define REAR_MOTOR_PIN 11
#define RIGHT_MOTOR_DIR 7
#define LEFT_MOTOR_DIR 8
#define FRONT_MOTOR_DIR 13
#define REAR_MOTOR_DIR 12

/*** Global variables ***/

//Variables related to radio transmitter
const boolean servo_dir[] = {0,0,1,0};
const float servo_rates[] = {1,1,1,1};
const float servo_subtrim[] = {0,0,0,0};
const boolean servo_mix_on = false;

const byte channels = 4;
float rc_in[channels];
int servo_us[channels];

// Timestamps
unsigned long rc_update = 0;
unsigned long last_motor_writing = 0;
unsigned long last_debug_print = 0;

//Variables related to operations of the SUV
int right_motor_speed;
int left_motor_speed;
int rear_motor_speed;
int front_motor_speed;

float set_depth;
int depth;
int angle;
float heading;

const int pid_calc_rate = 100;
pidData pid_balance = {0, 0.0, 0.0, 1.0, 0.0, 0.0, pid_calc_rate, 0, 0.0, 90, -90};
pidData pid_dive = {8, 0.0, 0.0, 30.0, 0.0, 0.0, pid_calc_rate, 0, 0.0, 255, -255};

void
setup () 
{
  const byte i2c_address = 15;
  
  Serial.begin (115200);
  Wire.begin (i2c_address);
  Wire.onReceive (receiveEvent);
    
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

  set_depth = pid_dive.setpoint;

  // Starts the radio controller readings
  //setup_pwmRead (); 
}

void 
loop() 
{
  const int debug_rate = 1000;
  const int motor_write_rate = 1000;
  unsigned long now = millis ();

//  // If RC data is available or 25ms has passed since last update (adjust to > frame rate of receiver)
//  if (RC_avail() || now - rc_update > 22){
//    readRCInput (channels, rc_in, servo_us);
//    calcWantedDepth (&set_depth, servo_us[1]);
//    pid_dive.setpoint = set_depth;
//    
//    rc_update = now;
//  }

  //PID controller signals//
  if (now - pid_balance.last_time >= pid_calc_rate){
    pid_balance = pidControl (pid_balance, angle, now);
    pid_dive = pidControl (pid_dive, depth, now);
  }

  // Calculate and write motor signals
  if (now - last_motor_writing > motor_write_rate){
    getDiveOutput (&rear_motor_speed, &front_motor_speed, pid_dive.control_signal);     
    getBalanceReduction (&rear_motor_speed, &front_motor_speed, pid_balance.control_signal);

    checkDiveMotorOutput (set_depth, &front_motor_speed, &rear_motor_speed);
    
    getSteeringOutput (servo_us[0], RIGHT_MOTOR_DIR, &right_motor_speed);
    getSteeringOutput (servo_us[2], LEFT_MOTOR_DIR, &left_motor_speed);
    
    setMotorSpeed (RIGHT_MOTOR_PIN, right_motor_speed);
    setMotorSpeed (LEFT_MOTOR_PIN, left_motor_speed);
    setMotorSpeed (REAR_MOTOR_PIN, rear_motor_speed);
    setMotorSpeed (FRONT_MOTOR_PIN, front_motor_speed);

    last_motor_writing = now;
  }

  if (now - last_debug_print > debug_rate){
    debugPrint();  
    last_debug_print = now;
  }
}

void 
debugPrint ()
{ 
  Serial.print("Depth: "); 
  Serial.print(depth);
  Serial.print(" dm");
  Serial.print("\t\t");
  Serial.print("Angle: ");
  Serial.print(angle); 
  Serial.print("\t\t");
  Serial.print("Heading: ");
  Serial.print(heading); 
  Serial.print("\t\t");
  Serial.print("Set depth: ");
  Serial.print(pid_dive.setpoint); 
  Serial.print("\t\t");
  Serial.print("Rear motor speed: ");
  Serial.print(rear_motor_speed); 
  Serial.print("\t\t");
  Serial.print("Front motor speed: ");
  Serial.print(front_motor_speed);
  Serial.print("\t\t");
  Serial.print("Control signal: ");
  Serial.print(pid_dive.control_signal);  
  
  Serial.println();
  Serial.println();
  
//  Serial.print("Servo 1: ");
//  Serial.print(servo_us[0]);
//  //Serial.print("\t");
//  //Serial.print(rc_in[0]);
//  Serial.print("\t\t");
//  Serial.print("Servo 2: ");
//  Serial.print(servo_us[1]);
//  //Serial.print("\t");
//  //Serial.print(rc_in[1]);
//  Serial.print("\t\t");
//  Serial.print("Servo 3: ");
//  Serial.print(servo_us[2]);
//  //Serial.print("\t");
//  //Serial.print(rc_in[2]);
//  Serial.print("\t\t");
//  Serial.print("Servo 4: ");
//  Serial.print(servo_us[3]);
//  //Serial.print("\t");
//  //Serial.print(rc_in[3]);
//  //Serial.print("\t\t");
//  Serial.println();
//  Serial.print("Right motor speed: ");
//  Serial.print(right_motor_speed);
//  Serial.print("\t\t");
//  Serial.print("Left motor speed: ");
//  Serial.print(left_motor_speed);
//  Serial.println();
//  Serial.println();
}
