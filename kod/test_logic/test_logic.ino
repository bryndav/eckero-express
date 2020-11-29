#define right_motor_pin 11
#define left_motor_pin 10
#define front_motor_pin 9
#define rear_motor_pin 6

int left_motor_speed;
int right_motor_speed;
int front_motor_speed;
int rear_motor_speed;

void
setup () 
{
  Serial.begin (115200);

  pinMode (right_motor_pin, OUTPUT);
  pinMode (left_motor_pin, OUTPUT);

  pinMode (front_motor_pin, OUTPUT);
  pinMode (rear_motor_pin, OUTPUT);

  left_motor_speed = 0;
  right_motor_speed = 0;
    
  rear_motor_speed = 220;
  front_motor_speed = 220;

  analogWrite (left_motor_pin, left_motor_speed);
  analogWrite (right_motor_pin, right_motor_speed);
  analogWrite (rear_motor_pin, rear_motor_speed);
  analogWrite (front_motor_pin, front_motor_speed);
}


void 
loop() 
{

}
