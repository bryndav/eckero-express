void 
calcWantedDepth (int* set_depth, 
                 int  servo_input)
{
  if (servo_input < 1000)
    servo_input = 1000;

  if (servo_input > 2000)
    servo_input = 2000;

  *set_depth = map (servo_input, 1000, 2000, 0, 20);  
}

int
mmToDm (int mm_input)
{
  float f_converter;
  int return_val;

  f_converter = mm_input / 100; 
  f_converter = round(f_converter);
  
  return_val = (int) f_converter;

  return return_val;
}

void readSerial() {

  while(Serial.available()) {
    byte var_type = Serial.read();
    int new_value = 0;
    char value[12] = "";

    Serial.readBytesUntil(0x0a, value, 12);
    new_value = atoi(value);

    if (new_value > 255) {
      new_value = 255;
    }

    if (new_value < 0) {
      new_value = 0;
    }

    Serial.println();
      
    switch(var_type)
    {
      case BACK_MOTOR: //B
        rear_motor_speed = new_value;

        Serial.print("Setting rear motor speed to: ");
        Serial.println(new_value);
        
        break;
      case FRONT_MOTOR: //F
        front_motor_speed = new_value;

        Serial.print("Setting front motor speed to: ");
        Serial.println(new_value);
      
        break;
      case LEFT_MOTOR: //L
        left_motor_speed = new_value;

        Serial.print("Setting left motor speed to: ");
        Serial.println(new_value);
      
        break;
      case RIGHT_MOTOR: //R
        right_motor_speed = new_value;

        Serial.print("Setting right motor speed to: ");
        Serial.println(new_value);

        break;
      case BOTH_FORWARD: //W
        right_motor_speed = new_value;
        left_motor_speed = new_value;

        Serial.print("Setting right and left motor speed to: ");
        Serial.println(new_value);
        break;
      case DEPTH: //D
        pid_dive.setpoint = new_value;

        Serial.print("Setting set depth to: ");
        Serial.println(new_value);
        break;
      default:
        while(Serial.available()){
          Serial.print("Unknown identifier...");
          Serial.println(var_type);
          Serial.println("Flushing serial bus");
          Serial.print(Serial.read());
      }
    }
    Serial.println();
  }
}
