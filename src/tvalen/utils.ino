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
    case TARGET_REACHED:
      Serial.println("TARGET_REACHED");
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
  Serial.print(destination->longitude, 6);
  Serial.print("\tTarget latitude: ");
  Serial.println(destination->latitude, 6);
  Serial.print("Distance to target: ");
  Serial.print(distance_to_target);
  Serial.print("\tBearing to target: ");
  Serial.print(pid_steering.setpoint);
  Serial.print("\tDegree diff: ");
  Serial.println(degree_diff);
  
  Serial.println();
  Serial.println();
}

void radioCom()
{
  //State
  char state[64];
  sprintf(state, "ST,%d\n", STATE);
  Serial.print(state);
  Serial1.print(state);
  
  delay(60);
  
  //Longitude
  char longi[64];
  sprintf(longi, "LT,%f\n", curr_pos.longitude);
  Serial.print(longi);
  Serial1.print(longi);
  
  delay(60);
  
  //Latitude
  char lat[64];
  sprintf(lat, "LA,%f\n", curr_pos.latitude);
  Serial.print(lat);
  Serial1.print(lat);
  
  delay(60);
  
  //Heading
  char head[64];
  sprintf(head, "HE,%f\n", imu_heading);
  Serial.print(head);
  Serial1.print(head);
  
  delay(60);
  
  //Servo
  char serv[64];
  sprintf(serv, "SE,%d\n", pid_steering.control_signal);
  Serial.print(serv);
  Serial1.print(serv);
  
  delay(60);
  
  //Distance to target
  char distance_to[64];
  sprintf(distance_to, "DT,%f\n", distance_to_target);
  Serial.print(distance_to);
  Serial1.print(distance_to);
  
  delay(60);
}

bool pollRadioRec() {
  return Serial1.available();
}

byte recieveInstruction() {
  byte instruction = Serial1.read();

  return instruction;
}

void actOnInstruction(byte instruction) {
  char resp[64];
  char msg[64];
  int servo_val;
  
  sprintf(resp, "Recieved instruction %c\n", instruction);
  Serial.print(resp);
  Serial1.print(resp);
  delay(60);
  
  switch(instruction) {
    case 71:
      digitalWrite(RELAY_PIN, HIGH);
      Serial.print("Motor ON\n");
      Serial1.print("Motor ON\n");
    break;
    
    case 83:
      digitalWrite(RELAY_PIN, LOW);
      Serial.print("Motor OFF\n");
      Serial1.print("Motor OFF\n");
    break;
    
    case 76:
      servo_val = steeringServo.read();
      servo_val = servo_val + 2;
      servo_val = (servo_val > (90 + 22)) ? (90 + 22) : servo_val;
      steeringServo.write(servo_val);

      sprintf(msg, "Left 2 deg, new val %d\n", servo_val);
      Serial.print(msg);
      Serial1.print(msg);
    break;

    case 82:
      servo_val = steeringServo.read();
      servo_val = servo_val - 2;
      servo_val = (servo_val > (90 - 22)) ? (90 - 22) : servo_val;
      steeringServo.write(servo_val);

      sprintf(msg, "Right 2 deg, new val %d\n", servo_val);
      Serial.print(msg);
      Serial1.print(msg);
    break;
    
    case 67:
      steeringServo.write(90);

      sprintf(msg, "Centered steering servo, new value %d\n", 90);
      Serial.print(msg);
      Serial1.print(msg);
    break;
  }
  
  delay(60);
}
