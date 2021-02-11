void 
updateSensors ()
{
  gyroSensor.updateEuler ();
  gyroSensor.updateCalibStatus ();
  depthSensor.read ();
}

void
readSensors (Sensors*   sensor_values)
{
  float temp_depth = 0.0;
  
  sensor_values->pitch = (int) gyroSensor.readEulerPitch ();
  sensor_values->heading = gyroSensor.readEulerHeading ();
  sensor_values->acceleration = gyroSensor.readAccelerometer(X_AXIS);

  sensor_values->pressure = (int) depthSensor.pressure ();
  sensor_values->temperature = depthSensor.temperature ();

  // Converts depth into mm stored as a int
  temp_depth = depthSensor.depth ();
  temp_depth = temp_depth * 1000;
  sensor_values->depth = (int) temp_depth;
}

int
correctDepth(int*     depth,
             int      depth_offset)
{ 
  int temp_int;
  
  temp_int = depth - depth_offset;
  *depth = (temp_int > 0) ? temp_int : 0;
}
