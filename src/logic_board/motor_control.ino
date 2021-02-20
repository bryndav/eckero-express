#include "LogicBoardDef.h"

void 
pidControl (pidData*      data, 
            int           input,
            unsigned long now)
{
  double error = data->setpoint - input;
  
  data->total_error += error;                                       // Accumalates the error - integral term
  data->total_error = checkMaxMin(data->max_control, data->min_control, data->total_error);
  
  double delta_error = error - data->last_error;                    // Difference of error for derivative term
  
  data->control_signal = data->Kp * error + (data->Ki * data->T) * data->total_error + (data->Kd / data->T) * delta_error; 
  data->control_signal = checkMaxMin(data->max_control, data->min_control, data->control_signal);

  data->last_error = error;
  data->last_time = now; 
}

double
checkMaxMin (int    max_val,
             int    min_val,
             double value)
{
  double return_val = value;

  if((int) value >= max_val) {
    return_val = max_val;
  }else if ((int) value <= min_val) {
    return_val = min_val;
  }

  return return_val;
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
  int balance_reduction;
  double reduce = control_signal;

  if (control_signal < 0)
    reduce = control_signal * -1;

  // 128 allows us to reduce half of the motor power
  balance_reduction = map (reduce, 0, 90, 0, 128);

  (control_signal < 0) ? *rear_motor_speed = *rear_motor_speed - balance_reduction : *front_motor_speed = *front_motor_speed - balance_reduction;
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

  if(*rear_motor_speed < 0)
    *rear_motor_speed = 0;

  if(*front_motor_speed < 0)
    *front_motor_speed = 0;
}

void
setMotorSpeed (int   motor_pin,  
               int   motor_speed)
{ 
  const int minimum_motor_output = 31;
  
  (motor_speed > minimum_motor_output) ? analogWrite (motor_pin, motor_speed) : analogWrite (motor_pin, 0);
}
