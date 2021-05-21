void
readIMUData() {
  sensors_event_t event, magnetometerData;
  bno.getEvent(&event);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  
  imu_heading = event.orientation.x;
  imu_pitch = event.orientation.y;
  imu_roll = event.orientation.z;
}
