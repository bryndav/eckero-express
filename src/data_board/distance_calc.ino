float
calcVelocity (float prev_velocity,
              float acceleration)
{
  float return_val;
  float sample_period = 0.0020;

  return_val = prev_velocity + (acceleration * sample_period);

  return return_val;
}

float
calcDistance (float prev_distance,
              float current_speed,
              float acceleration)
{
  float return_val;
  float sample_period = 0.0020;

  return_val = prev_distance + (current_speed * sample_period) + 0,5 * (acceleration * (sample_period * sample_period));

  return return_val;
}
