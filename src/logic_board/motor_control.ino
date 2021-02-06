#include "LogicBoardDef.h"

pidData 
pidControl (pidData       data, 
            int           input,
            unsigned long now)
{
  pidData temp = data;

  double error = temp.setpoint - input;
  temp.total_error += error;                                       // Accumalates the error - integral term

  if (temp.total_error >= temp.max_control) {
    temp.total_error = temp.max_control;
  }else if (temp.total_error <= temp.min_control) { 
    temp.total_error = temp.min_control;
  }
  
  double delta_error = error - temp.last_error;                    // Difference of error for derivative term
  temp.control_signal = temp.Kp * error + (temp.Ki * temp.T) * temp.total_error + (temp.Kd / temp.T) * delta_error; 

  if (temp.control_signal >= temp.max_control) {
      temp.control_signal = temp.max_control;
  }else if (temp.control_signal <= temp.min_control){
      temp.control_signal = temp.min_control;
  }
    
  temp.last_error = error;
  temp.last_time = now; 

  return temp;
}

void 
getSteeringOutput (float  rc_in_one,
                   float  rc_in_two,
                   int*   left_motor_speed, 
                   int*   right_motor_speed)
{
  float mix1 = (rc_in_one * -1.0) - rc_in_two;     // Channel 1 (ELV) - Channel 2 (AIL)
  float mix2 = (rc_in_one * -1.0) + rc_in_two;     // Channel 1 (ELV) + Channel 2 (AIL)

  if(mix1 > 1) {
    mix1 = 1;
  }else if (mix1 < -1) {
    mix1 = -1;
  }

  if(mix2 > 1) {
    mix2 = 1;
  }else if (mix2 < -1) {
    mix2 = -1;  
  }

  mix1 = mix1 * 100;
  mix2 = mix2 * 100;

  if (mix1 < 0.0)
    mix1 = 0.0;

  if (mix2 < 0.0)
    mix2 = 0.0;

  *left_motor_speed = map ((int) mix2, 0, 100, 0, 255);
  *right_motor_speed = map ((int) mix1, 0, 100, 0, 255);
}

void 
getDiveOutput (int*  rear_motor_speed, 
               int*  front_motor_speed,
               int   set_point,
               int   control_signal)
{
  int motor_speed;

  if (set_point <= 2)
    motor_speed = 0;
  else if (control_signal < 0)
    motor_speed = map(control_signal, 0, -255, depth_idle, 0);
  else{
    motor_speed = depth_idle + control_signal;

    if (motor_speed > 255)
      motor_speed = 255;
  }

  *rear_motor_speed = motor_speed;
  *front_motor_speed = motor_speed;
}

void 
getBalanceReduction (int*     rear_motor_speed, 
                     int*     front_motor_speed,  
                     double   control_signal)
{
  double reduce = control_signal;
  int balance_reduction;

  if (control_signal < 0)
    reduce = control_signal * -1;
 
  balance_reduction = map (reduce, 0, 90, 0, 128);

  if (control_signal < 0) {
    *rear_motor_speed = *rear_motor_speed - balance_reduction;
  }else {
    *front_motor_speed = *front_motor_speed - balance_reduction;
  }
}

void
setMotorSpeed (int   motor_pin,  
               int   motor_speed)
{

  
  (motor_speed > 15) ? analogWrite (motor_pin, motor_speed) : analogWrite (motor_pin, 0);
}
