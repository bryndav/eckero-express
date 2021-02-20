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
getSteeringOutput (int    rc_input,
                   int    motor_pin,
                   int*   motor_output)
{
  int output;
  
  if (rc_input >= 1500){
    digitalWrite(motor_pin, HIGH);
    output = map (rc_input, 1500, 2000, 0, 255);
  }else {
    digitalWrite(motor_pin, LOW); 
    output = map (rc_input, 1500, 1000, 0, 255);    
  }

  if (output > 255)
    output = 255;

  *motor_output = output;
}

void 
getDiveOutput (int*  rear_motor_speed, 
               int*  front_motor_speed,
               int   control_signal)
{
  const int r_motor_idle_speed = 95;
  const int f_motor_idle_speed = 140;

  *rear_motor_speed = r_motor_idle_speed + control_signal;
  *front_motor_speed = f_motor_idle_speed + control_signal;
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

  // 128 allows us to reduce half of the motor power
  balance_reduction = map (reduce, 0, 90, 0, 128);

  if (control_signal < 0) {
    if ((*rear_motor_speed - balance_reduction) >= 0)
      *front_motor_speed = *front_motor_speed - balance_reduction;
  }else {
    if ((*front_motor_speed - balance_reduction) >= 0)
      *rear_motor_speed = *rear_motor_speed - balance_reduction;
  }
}

void
checkDiveMotorOutput (int   set_depth,
                      int*  front_motor_speed,
                      int*  rear_motor_speed)
{
  int minimum_depth = 2;
  
  if(set_depth <= minimum_depth){
    *front_motor_speed = 0;
    *rear_motor_speed = 0;
  }
}

void
setMotorSpeed (int   motor_pin,  
               int   motor_speed)
{ 
  const int minimum_motor_output = 31;
  
  (motor_speed > minimum_motor_output) ? analogWrite (motor_pin, motor_speed) : analogWrite (motor_pin, 0);
}
