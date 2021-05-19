void
readIMUData() {
  sensors_event_t event, magnetometerData;
  bno.getEvent(&event);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);

  Serial.print("Mag x: ");
  Serial.println(magnetometerData.magnetic.x);
  Serial.print("Mag y: ");
  Serial.println(magnetometerData.magnetic.x);
  Serial.print("Mag z: ");
  Serial.println(magnetometerData.magnetic.x);
  imu_heading = event.orientation.x;
  imu_pitch = event.orientation.y;
  imu_roll = event.orientation.z;
}
