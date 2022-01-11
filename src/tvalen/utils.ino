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

void radioCom()
{  
  //Longitude
  char longi[64];
  sprintf(longi, "LT,%f\n", curr_pos.longitude);
  //Serial.print(longi);
  Serial1.print(longi);
  
  delay(60);
  
  //Latitude
  char lat[64];
  sprintf(lat, "LA,%f\n", curr_pos.latitude);
  //Serial.print(lat);
  Serial1.print(lat);
  
  delay(60);
  
  //Heading
  char head[64];
  sprintf(head, "HE,%f\n", imu_heading);
  //Serial.print(head);
  Serial1.print(head);
  
  delay(60);
  
  if (STATE != RADIO_CTRL) {
    //State
    char state[64];
    sprintf(state, "ST,%d\n", STATE);
    //Serial.print(state);
    Serial1.print(state);
    
    delay(60);  
    
    //Servo
    char serv[64];
    sprintf(serv, "SE,%d\n", pid_steering.control_signal);
    //Serial.print(serv);
    Serial1.print(serv);
    
    delay(60);
    
    //Distance to target
    char distance_to[64];
    sprintf(distance_to, "DT,%f\n", distance_to_target);
    //Serial.print(distance_to);
    Serial1.print(distance_to);
    
    delay(60);
  
    //Syst
    char syst[64];
    sprintf(syst, "SY,%d\n", sys);
    //Serial.print(syst);
    Serial1.print(syst);
    
    delay(60);
  
    //Gyro
    char gyr[64];
    sprintf(gyr, "GY,%d\n", gyro);
    //Serial.print(gyr);
    Serial1.print(gyr);
    
    delay(60);
  
    //Mag
    char magn[64];
    sprintf(magn, "MA,%d\n", mag);
    //Serial.print(magn);
    Serial1.print(magn);
    
    delay(60);
  
    //Acc
    char acc[64];
    sprintf(acc, "AC,%d\n", accel);
    //Serial.print(acc);
    Serial1.print(acc);
    
    delay(60);
  }
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
  int servo_val, servo_output;

  sprintf(resp, "OK, Recieved instruction %c\n", instruction);
  Serial.print(resp);
  Serial1.print(resp);
  delay(60);
  
  switch(instruction) {
    case 71:
      digitalWrite(RELAY_PIN, HIGH);
      Serial.print("OK, Motor ON\n");
      Serial1.print("OK, Motor ON\n");
    break;
    
    case 83:
      digitalWrite(RELAY_PIN, LOW);
      Serial.print("OK, Motor OFF\n");
      Serial1.print("OK, Motor OFF\n");
    break;
    
    case 76:
      servo_val = steeringServo.read() + 2 + 10;
      servo_output = (servo_val > (90 + 35)) ? (90 + 35) : servo_val;
      steeringServo.write(servo_output);

      sprintf(msg, "OK, Left 10 deg, new val %d\n", servo_output);
      Serial.print(msg);
      Serial1.print(msg);
    break;

    case 82:
      servo_val = steeringServo.read() + 2 - 10;
      servo_output = (servo_val < (90 - 35)) ? (90 - 35) : servo_val;
      steeringServo.write(servo_output);

      sprintf(msg, "OK, Right 10 deg, new val %d\n", servo_output);
      Serial.print(msg);
      Serial1.print(msg);
    break;
    
    case 67:
      setSteering(0, 1);

      sprintf(msg, "OK, Centered steering servo, new value %d\n", 90);
      Serial.print(msg);
      Serial1.print(msg);
    break;

    case 65:
      radio_state_switch = true;
      sprintf(msg, "OK, Entering automatic state\n");
      Serial.print(msg);
      Serial1.print(msg);
    break;
  }
  
  delay(60);
}
