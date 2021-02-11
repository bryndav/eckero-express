void
calcVelocity (float* prev_velocity,
              float acceleration)
{
  const float sample_period = 0.0100;

  *prev_velocity = *prev_velocity + (acceleration * sample_period);
}

float
calcDistance (float* prev_distance,
              float current_speed,
              float acceleration)
{
  const float sample_period = 0.0100;

  *prev_distance = *prev_distance + (current_speed * sample_period) + 0,5 * (acceleration * (sample_period * sample_period));
}
