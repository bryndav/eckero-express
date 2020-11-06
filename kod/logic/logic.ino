/*** Needed libraries ***/
#include <Wire.h>

#define right_motor_pin 11
#define left_motor_pin 6
#define front_motor_pin 9
#define rear_motor_pin 10
#define in3 13
#define in4 12

/*** Global variables ***/

//Variables related to radio transmitter

// Select Servo Direction, Rates and Sub-trim (the size of each array must match the number of servos)
boolean servo_dir[] = {0,0,0,0};        // Direction: 0 is normal, 1 is reverse
float servo_rates[] = {1,1,1,1};        // Rates: range 0 to 2 (1 = +-500us (NORMAL), 2 = +-1000us (MAX)): The amount of servo deflection in both directions
float servo_subtrim[] = {0,0,0,0};      // Subtrimrange -1 to +1 (-1 = 1000us, 0 = 1500us, 1 = 2000us): The neutral position of the servo
boolean servo_mix_on = false;

const int channels = 4;                 // Specify the number of receiver channels
float rc_in[channels];                  // An array to store the calibrated input from receiver 

int servo1_us;                          // Variables to store the pulse widths to be sent to the servo
int servo2_us;      
int servo3_us;
int servo4_us;

// Time related variables (timestamps)

unsigned long now;
unsigned long rc_update = 0;
unsigned long last_motor_writing = 0;
unsigned long last_debug_print = 0;
int debug_rate = 1000;
int motor_write_rate = 1000;

//Variables related to operations of the SUV

int right_motor_speed;
int left_motor_speed;
int rear_motor_speed;
int front_motor_speed;

float set_depth;
int depth;
int angle;
float heading;
byte f_byteArray[4];

struct pidData{
  double control_signal;
  int setpoint;
  double Kp; //proportional gain
  double Ki; //integral gain
  double Kd; //derivative gain
  int T; //sample time in milliseconds (ms)
  unsigned long last_time;
  double total_error;
  double last_error;
  int max_control;
  int min_control;
};

struct pidData pid_balance = {0.0, 0, 3.0, 0.0, 0.0, 1000, 0, 0.0, 0.0, 90, -90};
struct pidData pid_dive = {0.0, 0, 3, 0, 0, 1000, 0, 0.0, 0.0, 1000, 0};

void setup() {
  Serial.begin(115200);
  Wire.begin(15);                       // Initializes I2C as slave with adress 15
  Wire.onReceive(receiveEvent);         // Connect incoming messages with a function
  
  pinMode(right_motor_pin, OUTPUT);                 // Define outoutpins to controll PWMN input on L298N
  pinMode(left_motor_pin, OUTPUT);

  pinMode(front_motor_pin, OUTPUT);                 // Define outoutpins to controll PWMN input on L298N
  pinMode(rear_motor_pin, OUTPUT);
  
  // Starts the radio controller readings
  setup_pwmRead(); 
}

void loop() {

  now = millis();

  // If RC data is available or 25ms has passed since last update (adjust to > frame rate of receiver)
  if(RC_avail() || now - rc_update > 22){
    readRCInput();
    calcWantedDepth(&set_depth, servo3_us);
    rc_update = now;
  }

  //PID controller signals//
  if (now - pid_balance.last_time >= pid_balance.T){
    pid_balance = pidControl(pid_balance, angle);
  }

  if (now - pid_dive.last_time >= pid_dive.T){
    pid_dive.setpoint = set_depth;
    pid_dive = pidControl(pid_dive, depth);
  }

  // Calculate and write motor signals
  if(now - last_motor_writing > motor_write_rate){
    dive(&rear_motor_speed, &front_motor_speed, set_depth); 
    balance(&rear_motor_speed, &front_motor_speed, pid_balance.control_signal);
    steering(&right_motor_speed, &left_motor_speed);
    last_motor_writing = now;
  }

  // System debug prints
  if(now - last_debug_print > debug_rate){
    printInfo();  
    last_debug_print = now;
  }
}

void readRCInput(){
                              
  //print_RCpwm();                        // Uncommment to print raw data from receiver to serial
  
  for (int i = 0; i < channels; i++){     // Run through each RC channel
    int CH = i+1;
    
    rc_in[i] = RC_decode(CH);             // Decode receiver channel and apply failsafe
    
    //print_decimal2percentage(rc_in[i]); // Uncomment to print calibrated receiver input (+-100%) to serial       
    //Serial.println();  
  }
                                 
  if (servo_mix_on == true){              // MIXING ON
    
    float mix1 = rc_in[0] - rc_in[1];     // Channel 1 (ELV) - Channel 2 (AIL)
    float mix2 = rc_in[0] + rc_in[1];     // Channel 1 (ELV) + Channel 2 (AIL)

    if(mix1 > 1){
      mix1 = 1;                           // Limit mixer output to +-1
    }else if(mix1 < -1){
      mix1 = -1;
    }

    if(mix2 > 1){
      mix2 = 1;                           // Limit mixer output to +-1
    }
    else if(mix2 < -1){
      mix2 = -1;  
    }

    // Calculate the pulse widths for the servos
    servo1_us = calc_uS(mix1, 1);         // Apply the servo rates, direction and sub_trim for servo 1, and convert to a RC pulsewidth (microseconds, uS)
    servo2_us = calc_uS(mix2, 2);         // Apply the servo rates, direction and sub_trim for servo 2, and convert to a RC pulsewidth (microseconds, uS)          
    }
    else{                                 // MIXING OFF
    servo1_us = calc_uS(rc_in[0],1);      // Apply the servo rates, direction and sub_trim for servo 1, and convert to a RC pulsewidth (microseconds, uS)
    servo2_us = calc_uS(rc_in[1],2);      // Apply the servo rates, direction and sub_trim for servo 2, and convert to a RC pulsewidth (microseconds, uS)
    servo3_us = calc_uS(rc_in[2],3);      // Apply the servo rates, direction and sub_trim for servo 3, and convert to a RC pulsewidth (microseconds, uS)
    servo4_us = calc_uS(rc_in[3],4);      // Apply the servo rates, direction and sub_trim for servo 4, and convert to a RC pulsewidth (microseconds, uS)
  }
}

void calcWantedDepth(float* set_depth, int servo_input){

  // Scale away fluctuating values
  if(servo_input < 1000){
    servo_input = 1000;
  }

  if(servo_input > 2000){
    servo_input = 2000;
  }

  *set_depth = map(servo_input, 2000, 1000, 0, 1000);  
}

struct pidData pidControl(struct pidData data, int input){
  struct pidData temp = data;

  double error = temp.setpoint - input;
  temp.total_error += error;                                       // Accumalates the error - integral term
  
  if (temp.total_error >= temp.max_control){
    temp.total_error = temp.max_control;
  }else if (temp.total_error <= temp.min_control){ 
    temp.total_error = temp.min_control;
  }
  
  double delta_error = error - temp.last_error;                    // Difference of error for derivative term
  temp.control_signal = temp.Kp*error + (temp.Ki*temp.T)*temp.total_error + (temp.Kd/temp.T)*delta_error; 

  if (temp.control_signal >= temp.max_control) {
      temp.control_signal = temp.max_control;
    }
    else if (temp.control_signal <= temp.min_control){
      temp.control_signal = temp.min_control;
    }
    
    temp.last_error = error;
    temp.last_time = now; 

  return temp;
}

int calc_uS(float cmd, int servo){                                // cmd = commanded position +-100% 
                                                                  // servo = servo num (to apply correct direction, rates and trim)
  int i = servo-1;
  float dir;
  if(servo_dir[i] == 0) dir = -1; else dir = 1;                   // set the direction of servo travel
  
  cmd = 1500 + (cmd*servo_rates[i]*dir + servo_subtrim[i])*500;   // apply servo rates and sub trim, then convert to a uS value

  if(cmd > 2500) cmd = 2500;                                      // limit pulsewidth to the range 500 to 2500us
  else if(cmd < 500) cmd = 500;

  return cmd;
}

void receiveEvent (int length){
  char data_type;
  byte temp_byte;

  data_type = Wire.read();

  switch(data_type) {
    case 'P':
      int message;
      byte high_byte, low_byte;

      high_byte = Wire.read();
      low_byte = Wire.read();

      message = low_byte | (high_byte << 8);

      if (message >  90) {
        message = message - 256;
      }
      angle = message;

      break;
    
    case 'D':
      float temp_depth;
      
      for(int i = 0; i < 4; i++){
        temp_byte = Wire.read();
        f_byteArray[i] = temp_byte;
      }

      temp_depth = *(float *)&f_byteArray;
      depth = (int) (temp_depth * 100);

      break;

    case 'H':
      for(int i = 0; i < 4; i++){
        temp_byte = Wire.read();
        f_byteArray[i] = temp_byte;
      }

      heading = *(float *)&f_byteArray;

      break;

    default:
      byte val;
    
      Serial.print("Wrong value recieved: ");
      Serial.print(data_type);

      while (Wire.available() > 0){
        val = Wire.read();
        Serial.println(val);
      }
  }

  while (Wire.available() > 0){
    Wire.read();  
  }
}

void steering(int* left_motor_speed, int* right_motor_speed){
  int x, y, z;
  
  if (servo1_us < 1500) {
    servo1_us = 1500;
  }
  
  y = map(servo1_us, 1500, 2000, 0, 255);
  x = map(servo2_us, 1500, 1000, 0, 255);
  z = abs(x) + y;
  
  if (z > 255) {
    z = 255;
  }
  
  if (x < 0) {
    *left_motor_speed = z;
    *right_motor_speed == 0;
  }else{
    *right_motor_speed = z;
    *left_motor_speed == 0;
  }
}

void dive(int* rear_motor_speed, int* front_motor_speed, int wanted_depth){
  int motor_speed;

  motor_speed = map(pid_dive.control_signal, 0, 1000, 0, 255);
  *rear_motor_speed = motor_speed;
  *front_motor_speed = motor_speed;
}

void balance(int* rear_motor_speed, int* front_motor_speed, double control_signal){
  double reduce = control_signal;
  int balance_reduction;

  if(control_signal < 0){
    reduce = control_signal * -1;
  }
 
  balance_reduction = map(reduce, 0, 90, 0, 128);

  if(control_signal < 0){
    *rear_motor_speed = *rear_motor_speed - balance_reduction;
  }else{
    *front_motor_speed = *front_motor_speed - balance_reduction;
  }
}

void printInfo(){

  // Info regarding depth sensor
  
  //Serial.print("Pressure: "); 
  //Serial.print(sensor.pressure()); 
  //Serial.println(" mbar");
    
  //Serial.print("Temperature: "); 
  //Serial.print(sensor.temperature()); 
  //Serial.println(" deg C");
    
  Serial.print("Depth: "); 
  Serial.print(depth);
  Serial.print(" m");
  Serial.print("\t\t\t");
  Serial.print("Angle: ");
  Serial.print(angle); 
  Serial.print("\t\t\t");
  Serial.print("Rear motor speed: ");
  Serial.print(rear_motor_speed); 
  Serial.print("\t\t\t");
  Serial.print("Front motor speed: ");
  Serial.print(front_motor_speed);
  Serial.println();
    
  //Serial.print("Altitude: "); 
  //Serial.print(sensor.altitude()); 
  //Serial.println(" m above mean sea level");

  // Info regarding servo values from RC controller

  //Serial.println();
  //Serial.print("Servo 1: ");
  //Serial.print(servo1_us);
  //Serial.print(rc_in[0]);
  //Serial.print("\t\t\t");
  //Serial.print("Servo 2: ");
  //Serial.print(servo2_us);
  //Serial.print(rc_in[1]);
  //Serial.print("\t\t\t");
  //Serial.print("Servo 3: ");
  //Serial.print(servo3_us);
  //Serial.print(rc_in[2]);
  //Serial.print("\t\t\t");
  //Serial.print("Servo 4: ");
  //Serial.println(servo4_us);
  //Serial.print(rc_in[3]);
  //Serial.println();
}
