void 
pidControl (pidData*      data, 
            int           input,
            unsigned long now)
{
  double error = input;

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
             double val)
{
  double return_val = val;

  if((int) val>= max_val) {
    return_val = max_val;
  }else if ((int) val <= min_val) {
    return_val = min_val;
  }

  return return_val;
}

int
calcAngle(int curr, 
          int target)
{ 
  int phi = abs(target - curr) % 360;
  int diff = (phi > 180) ? 360 - phi : phi;

  return diff;
}

int
calcDirection(int curr,
              int target)
{
  int diff_right, diff_left;
  int return_val;

  if (curr > target) {
    diff_right = (curr % 360) + target;
    diff_left = curr - target;
  }else{
    diff_right = target - curr;
    diff_left = curr + (target % 360);
  }

  return_val = (diff_right < diff_left) ? 1 : -1; 

  return return_val;
}

int 
findTurnSide (int curr,
              int target)
{
  int diff = target - curr;

  if(diff < 0) {
    diff += 360;
  }
  if(diff > 180) {
    // Left
    return -1;
  }
  else {
    // Right
    return 1;
  }
}

void
setSteering(int  ctrl_signal,
            int  dir)
{
  int pwm_signal;
  const int rodder_offset = 0;
  const int rodder_max = 135;
  const int rodder_min = 45;
  
  pwm_signal = (dir > 0) ? (90 - ctrl_signal - rodder_offset) : (90 + ctrl_signal - rodder_offset);
  pwm_signal = checkMaxMin(rodder_max, rodder_min, pwm_signal);

  steeringServo.write(pwm_signal);
}


void
startMotor (int sig)
{
  if (sig == 1) {
    digitalWrite(RELAY_PIN, HIGH);
  } else {
    digitalWrite(RELAY_PIN, LOW); 
  }
}
