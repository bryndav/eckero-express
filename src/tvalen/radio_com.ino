
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

  sprintf(resp, "OK, Received instruction %c\n", instruction);
  Serial1.print(resp);
  
  switch(instruction) {
    case 71:
      digitalWrite(RELAY_PIN, HIGH);
      Serial1.print("OK, Motor ON\n");
    break;
    
    case 83:
      digitalWrite(RELAY_PIN, LOW);
      Serial1.print("OK, Motor OFF\n");
    break;
    
    case 76:
      servo_val = steeringServo.read() + 2 + 10;
      servo_output = (servo_val > (90 + 35)) ? (90 + 35) : servo_val;
      steeringServo.write(servo_output);

      sprintf(msg, "OK, Left 10 deg, new val %d\n", servo_output);
      Serial1.print(msg);
    break;

    case 82:
      servo_val = steeringServo.read() + 2 - 10;
      servo_output = (servo_val < (90 - 35)) ? (90 - 35) : servo_val;
      steeringServo.write(servo_output);

      sprintf(msg, "OK, Right 10 deg, new val %d\n", servo_output);
      Serial1.print(msg);
    break;
    
    case 67:
      setSteering(0, 1);

      sprintf(msg, "OK, Centered steering servo, new value %d\n", 90);
      Serial1.print(msg);
    break;

    case 65:
      radio_state_switch = true;
      sprintf(msg, "OK, Entering automatic state\n");
      Serial1.print(msg);
    break;
  }
}


void sendSystemVars()
{
  char msg[64];

  switch(radio_com_index){
    
  case LONGITUDE_PRINT:
    sprintf(msg, "LT,%f\n", curr_pos.longitude);
    Serial1.print(msg);
  
    radio_com_index = LATITUDE_PRINT;
  break;
  
  case LATITUDE_PRINT:
    sprintf(msg, "LA,%f\n", curr_pos.latitude);
    Serial1.print(msg);

    radio_com_index = HEADING_PRINT;
  break;
  
  case HEADING_PRINT:
    sprintf(msg, "HE,%f\n", imu_heading);
    Serial1.print(msg);

    radio_com_index = STATE_PRINT;
  break;
  
  case STATE_PRINT:
    sprintf(msg, "ST,%d\n", STATE);
    Serial1.print(msg);

    radio_com_index = SERVO_PRINT;
  break;
  
  case SERVO_PRINT:
    sprintf(msg, "SE,%d\n", pid_steering.control_signal);
    Serial1.print(msg);

    radio_com_index = DISTANCE_PRINT;
  break;
  
  case DISTANCE_PRINT:
    sprintf(msg, "DT,%f\n", distance_to_target);
    Serial1.print(msg);

    radio_com_index = IMUMAG_PRINT;
  break;
  
  case IMUMAG_PRINT:
    sprintf(msg, "MA,%d\n", mag);
    Serial1.print(msg);
    
    radio_com_index = IMUSYS_PRINT;
  break;
  
  case IMUSYS_PRINT:
    sprintf(msg, "SY,%d\n", sys);
    Serial1.print(msg);

    radio_com_index = IMUGYR_PRINT;
  break;
  
  case IMUGYR_PRINT:
    sprintf(msg, "GY,%d\n", gyro);
    Serial1.print(msg);
    
    radio_com_index = IMUACC_PRINT;
  break;
  
  case IMUACC_PRINT:
    sprintf(msg, "AC,%d\n", accel);
    Serial1.print(msg);

    radio_com_index = LONGITUDE_PRINT;
  break;
  }
}
