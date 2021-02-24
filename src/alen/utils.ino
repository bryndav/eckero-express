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
