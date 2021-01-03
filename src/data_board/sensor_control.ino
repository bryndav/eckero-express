void 
updateSensors ()
{
  gyroSensor.updateEuler ();
  gyroSensor.updateCalibStatus ();
  depthSensor.read ();
}

void
readSensors (int*     pitch,
             int*     pressure, 
             int*     depth, 
             float*   heading,
             float*   temperature,
             float*   acceleration)
{
  float temp_depth = 0.0;
  
  *pitch = (int) gyroSensor.readEulerPitch ();
  *heading = gyroSensor.readEulerHeading ();
  *acceleration = gyroSensor.readAccelerometer(X_AXIS);

  *pressure = (int) depthSensor.pressure ();
  *temperature = depthSensor.temperature ();

  // Converts depth into mm stored as a int
  temp_depth = depthSensor.depth ();
  temp_depth = temp_depth * 1000;
  *depth = (int) temp_depth;
}

int
correctDepth(int     depth,
             int     depth_offset)
{
  int return_val;
  
  return_val = depth - depth_offset;
  return_val = (return_val > 0) ? return_val : 0;

  return return_val;
}
