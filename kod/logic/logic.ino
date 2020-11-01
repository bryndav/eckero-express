/*** Needed libraries ***/
#include <Wire.h>

#define enA 11 //define PWM input on L298N
#define enB 10


/*** Global variables ***/

//Variables related to radio transmitter

// Select Servo Direction, Rates and Sub-trim (the size of each array must match the number of servos)
boolean servo_dir[] = {0,0,0,0};        // Direction: 0 is normal, 1 is reverse
float servo_rates[] = {1,1,1,1};        // Rates: range 0 to 2 (1 = +-500us (NORMAL), 2 = +-1000us (MAX)): The amount of servo deflection in both directions
float servo_subtrim[] = {0,0,0,0};      // Subtrimrange -1 to +1 (-1 = 1000us, 0 = 1500us, 1 = 2000us): The neutral position of the servo
boolean servo_mix_on = false;

const int channels = 4;                 // Specify the number of receiver channels
float RC_in[channels];                  // An array to store the calibrated input from receiver 

int servo1_uS;                          // Variables to store the pulse widths to be sent to the servo
int servo2_uS;      
int servo3_uS;
int servo4_uS;

// Time related variables (timestamps)

unsigned long now;                      // Timing variables to update data at a regular interval                        
unsigned long rc_update = 0;
unsigned long last_PID_calc = 0;

//Variables related to operations of the SUV

int right_pump_voltage;
int left_pump_voltage;
float set_depth;
float depth;
int dive_voltage;

int i;
float currentAngle = 0.0;
byte f_byteArray[4];

struct PIDdata{
  double control_signal;
  double setpoint = 0;
  double Kp = 1; //proportional gain
  double Ki = 0; //integral gain
  double Kd = 0; //derivative gain
  int T = 50; //sample time in milliseconds (ms)
  unsigned long last_time;
  double total_error;
  double last_error;
  int max_control = 90;
  int min_control = -90;
} PID_balance, PID_dive;


void setup() {

  Serial.begin(115200);
  Wire.begin(15);                       // Initializes I2C as slave with adress 15
  Wire.onReceive(receiveEvent);         // Connect incoming messages with a function
  
  //Serial.setTimeout(50);
  pinMode(enA, OUTPUT);                 // Define outoutpins to controll PWMN input on L298N
  pinMode(enB, OUTPUT);

  // Starts the radio controller readings
  setup_pwmRead(); 
}

void loop() {

  now = millis();

  ///RC TX RX///
  // If RC data is available or 25ms has passed since last update (adjust to > frame rate of receiver)
  if(RC_avail() || now - rc_update > 22){
    readRCInput();
  }

  //PID controller signals//
  if (now - PID_balance.last_time >= PID_balance.T){
    PID_balance = PID_Control(PID_balance, currentAngle);     // Calls the PID function every T interval and outputs a control signal   
  }

  if (now - PID_dive.last_time >= PID_dive.T){
    PID_dive = PID_Control(PID_dive, depth);
  }

  printInfo();
}

void readRCInput(){
   
  rc_update = now;                           
  //print_RCpwm();                        // Uncommment to print raw data from receiver to serial
  
  for (int i = 0; i < channels; i++){     // Run through each RC channel
    int CH = i+1;
    
    RC_in[i] = RC_decode(CH);             // Decode receiver channel and apply failsafe
    
    //print_decimal2percentage(RC_in[i]); // Uncomment to print calibrated receiver input (+-100%) to serial       
    //Serial.println();  
  }
                                 
  if (servo_mix_on == true){              // MIXING ON
    
    float mix1 = RC_in[0] - RC_in[1];     // Channel 1 (ELV) - Channel 2 (AIL)
    float mix2 = RC_in[0] + RC_in[1];     // Channel 1 (ELV) + Channel 2 (AIL)

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
    servo1_uS = calc_uS(mix1, 1);         // Apply the servo rates, direction and sub_trim for servo 1, and convert to a RC pulsewidth (microseconds, uS)
    servo2_uS = calc_uS(mix2, 2);         // Apply the servo rates, direction and sub_trim for servo 2, and convert to a RC pulsewidth (microseconds, uS)          
    }
    else{                                 // MIXING OFF
    servo1_uS = calc_uS(RC_in[2],1);      // Apply the servo rates, direction and sub_trim for servo 1, and convert to a RC pulsewidth (microseconds, uS)
    servo2_uS = calc_uS(RC_in[3],2);      // Apply the servo rates, direction and sub_trim for servo 2, and convert to a RC pulsewidth (microseconds, uS)
    servo3_uS = calc_uS(RC_in[4],3);      // Apply the servo rates, direction and sub_trim for servo 3, and convert to a RC pulsewidth (microseconds, uS)
    servo4_uS = calc_uS(RC_in[5],4);      // Apply the servo rates, direction and sub_trim for servo 4, and convert to a RC pulsewidth (microseconds, uS)
  }
}

struct PIDdata PID_Control(struct PIDdata data, float input){
  struct PIDdata temp = data;

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
  char dataType;

  dataType = Wire.read();

  switch(dataType) {
    case 'P':

      int message;

      for(i = 4; i > 0; i--){
        if(i > 1){
          message |= Wire.read();
          message <<= i * 8;
        }else
          message |= Wire.read();
      }

      if (message >  90) {
        message = message - 256;
      }

      currentAngle = (float)message;

      break;
    
    case 'D':

      for(i = 0; i > 3; i++){
        f_byteArray[i] = Wire.read();
      }

      depth = *(float *)&f_byteArray;

      break;

    default:
      
      byte val;
    
      Serial.print("Wrong value recieved: ");
      Serial.print(dataType);

      while (Wire.available() > 0){
        val = Wire.read();
        Serial.println(val);
      }
  }

  while (Wire.available() > 0){
    Wire.read();  
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
    
  //Serial.print("Depth: "); 
  //Serial.print(sensor.depth());
  //Serial.print(" m");
  //Serial.print("\t\t\t\t");
  //Serial.print(currentAngle); 
  //Serial.println();
    
  //Serial.print("Altitude: "); 
  //Serial.print(sensor.altitude()); 
  //Serial.println(" m above mean sea level");

  // Info regarding servo values from RC controller

  //Serial.println();
  //Serial.print("Servo 1: ");
  //Serial.println(servo1_uS);
  //Serial.print("Servo 2: ");
  //Serial.println(servo2_uS);
  //Serial.print("Servo 3: ");
  //Serial.println(servo3_uS);
  //Serial.print("Servo 4: ");
  //Serial.println(servo4_uS);
  //Serial.println();

  // Info regarding PID values and motor controll

  //Serial.println(sensed_output);
  //Serial.println(control_signal);
  //Serial.print("\t");
  //Serial.println(right_pump_voltage);
  //Serial.print("\t");
  //Serial.println(left_pump_voltage);
  //Serial.println(servo1_uS);
  //Serial.print(rollAngle);
}
