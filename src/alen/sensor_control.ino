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
  sensor_values->pitch = (int) gyroSensor.readEulerPitch ();
  sensor_values->heading = gyroSensor.readEulerHeading ();
  sensor_values->acceleration = gyroSensor.readAccelerometer(X_AXIS);

  sensor_values->pressure = (int) depthSensor.pressure ();
  sensor_values->temperature = depthSensor.temperature ();

  // Converts depth into mm stored as a int
  sensor_values->depth = correctDepth(&sensor_values->depth, depthSensor.depth ());
}

int
correctDepth(int*     depth,
             float    new_depth)
{ 
  const int depth_offset = 540;
  float temp_depth;
  int temp_int;

  temp_depth = new_depth * 1000;
  temp_int = (((int) temp_depth) - depth_offset);
  *depth = (temp_int > 0) ? temp_int : 0;
}
