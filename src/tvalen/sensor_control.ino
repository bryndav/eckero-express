void
readHeading() {
  const double imu_offset = 0.0;
  
  sensors_event_t event, magnetometerData; 
  bno.getEvent(&event);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getCalibration(&sys, &gyro, &accel, &mag);

  imu_heading = event.orientation.x - imu_offset;
  imu_heading = (imu_heading > 0) ? imu_heading : 360.0 - imu_heading;
  inclanation = event.orientation.y;
}
