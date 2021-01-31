void 
calcWantedDepth (float*   set_depth, 
                 int      servo_input)
{
  if (servo_input < 1000)
    servo_input = 1000;

  if (servo_input > 2000)
    servo_input = 2000;

  *set_depth = map (servo_input, 2000, 1000, 0, 10);  
}
