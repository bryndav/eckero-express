void
readHeading() {
  /* Get a new sensor event */ 
  sensors_event_t event; 
  bno.getEvent(&event);
   
  imu_heading = event.orientation.x;
  inclanation = event.orientation.y;

  bno.getCalibration(&sys, &gyro, &accel, &mag);
}
