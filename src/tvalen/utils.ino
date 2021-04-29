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

  Serial.print("\t\t Position nr: ");
  Serial.println(pos_index + 1);
  Serial.print("Longitude: ");
  Serial.print(curr_pos.longitude, 6);
  Serial.print("\t\tLatitude: ");
  Serial.print(curr_pos.latitude, 6);
  Serial.print("\tHeading: ");
  Serial.println(imu_heading);  

  Serial.print("Servo signal: ");
  Serial.print(pid_steering.control_signal);
  Serial.print("\t\tDirection: ");
  if(steering > 0) {
    Serial.println("Right");
  }else {
    Serial.println("Left");
  }

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
  
  Serial.println();
  Serial.println();
}
