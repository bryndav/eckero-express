#include <stdio.h>
/*
 * Communication header:
 * 
 * Message enum,
 * State
 * Longitude,
 * Latitude,
 * Heading,
 * Servo signal,
 * Direction,
 * Distance to target,
 * Heading to target,
 * Target longitude,
 * Target latitude *eol*
 */


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
      Serial.println("WAIT_FOR_GPS");
      break;
    case PLAN_COURSE:
      Serial.println("PLAN_COURSE");
      break;
    case NORMAL_OPERATIONS:
      Serial.println("NORMAL_OPERATIONS");
      break;
  }
  
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
  Serial.print("Sys:");
  Serial.print(sys, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.println(mag, DEC);
  Serial.println();
  Serial.println();
}
