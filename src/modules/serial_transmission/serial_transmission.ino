// Operations
int right_motor_speed = 10;
int left_motor_speed = 10;
int rear_motor_speed = 10;
int front_motor_speed = 10;

typedef enum {
  SET = 83,
  GET = 71,
  BACK_MOTOR = 66,
  FRONT_MOTOR = 70,
  LEFT_MOTOR = 76,
  RIGHT_MOTOR = 82
} trans_def;

void
setup ()
{
 Serial.begin(9600); 
}

void
loop ()
{
  updateSerial();
  delay(5);
}

void updateSerial() {

  while(Serial.available()) {
    byte trans_type = Serial.read();
    byte var_type = Serial.read();
    int new_value;
    char value[12];

    if (trans_type == SET){ //S
      Serial.readBytesUntil(0x0a, value, 12);
      new_value = atoi(value);

      if (new_value > 255) {
        new_value = 255;
      }

      if (new_value < 0) {
        new_value = 0;
      }
      
      switch(var_type)
      {
        case BACK_MOTOR: //B
          rear_motor_speed = new_value;

          Serial.print("Setting rear motor speed to: ");
          Serial.println(rear_motor_speed);
          
          break;
        case FRONT_MOTOR: //F
          front_motor_speed = new_value;

          Serial.print("Setting front motor speed to: ");
          Serial.println(front_motor_speed);
        
          break;
        case LEFT_MOTOR: //L
          left_motor_speed = new_value;

          Serial.print("Setting left motor speed to: ");
          Serial.println(left_motor_speed);
        
          break;
        case RIGHT_MOTOR: //R
          right_motor_speed = new_value;

          Serial.print("Setting right motor speed to: ");
          Serial.println(right_motor_speed);
          
          break;
      }
    }else if (trans_type == GET) { // G
      switch(var_type)
      {
        case BACK_MOTOR: // B
          Serial.print("Rear motor value: ");
          Serial.println(rear_motor_speed);

          break;
        case FRONT_MOTOR: // F
          Serial.print("Front motor value: ");
          Serial.println(front_motor_speed);
        
          break;
        case LEFT_MOTOR: // L
          Serial.print("Left motor value: ");
          Serial.println(left_motor_speed);
        
          break;
        case RIGHT_MOTOR: // R
          Serial.print("Right motor value: ");
          Serial.println(right_motor_speed);
          
          break;
      }
    }else{
      while(Serial.available()){
        Serial.print("Unknown identifier...");
        Serial.println(var_type);
        Serial.println("Flushing serial bus");
        Serial.print(Serial.read());
      }
    }
  }
}
