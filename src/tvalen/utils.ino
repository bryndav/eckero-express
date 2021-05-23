void readSerial() {
  while(Serial.available()) {
    char val[12] = "";
    Serial.readBytesUntil(0x0a, val, 12);
    heading = atoi(val);
  }
}

void
debugPrint()
{
  Serial.print("State: ");
  switch(STATE) {
    case WAIT_FOR_GPS:
      Serial.print("WAIT_FOR_GPS");
      break;
    case PLAN_COURSE:
      Serial.print("PLAN_COURSE");
      break;
    case NORMAL_OPERATIONS:
      Serial.print("NORMAL_OPERATIONS");
      break;
    case TARGET_REACHED:
      Serial.print("TARGET_REACHED");
      break;
  }

  Serial.print("\t\tPosition nr: ");
  Serial.println(pos_index + 1);
  Serial.print("Longitude: ");
  Serial.print(curr_pos.longitude, 6);
  Serial.print("\t\tLatitude: ");
  Serial.print(curr_pos.latitude, 6);
  Serial.print("\tIMU Heading: ");
  Serial.print(imu_heading);  
  Serial.print("\tGPS heading: ");
  Serial.println(heading);

  Serial.print("Servo signal: ");
  Serial.print(pid_steering.control_signal);
  Serial.print("\t\tDirection: ");
  if(steering > 0) {
    Serial.print("Right");
  }else {
    Serial.print("Left");
  }
  Serial.print("\t Satellites: ");
  Serial.println(gps_data.numSats);
  Serial.print("Target longitude: ");
  Serial.print(destination.longitude, 6);
  Serial.print("\tTarget latitude: ");
  Serial.println(destination.latitude, 6);
  Serial.print("Distance to target: ");
  Serial.print(distance_to_target);
  Serial.print("\tBearing to target: ");
  Serial.print(pid_steering.setpoint);
  Serial.print("\tDegree diff: ");
  Serial.println(degree_diff);
  displayCalStatus();
  Serial.println();
  Serial.println();
}


void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}
