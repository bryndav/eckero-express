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
  *pitch = (int) gyroSensor.readEulerRoll ();
  *heading = gyroSensor.readEulerHeading ();
  *acceleration = gyroSensor.readAccelerometer(X_AXIS);

  *pressure = (int) depthSensor.pressure ();
  *temperature = depthSensor.temperature ();
  *depth = (int) depthSensor.depth ();
}

int
correctDepth(int     depth,
             int     depth_offset)
{
  int return_val;
  
  return_val = (depth * 1000) - depth_offset;
  return_val = (return_val > 0) ? return_val : 0;

  return return_val;
}
