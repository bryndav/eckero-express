#include "LogicBoardDef.h"

int
readInt ()
{
  int output;
  byte high_byte, low_byte;

  high_byte = Wire.read ();
  low_byte = Wire.read ();

  output = low_byte | (high_byte << 8);

  if (output >  90)
    output = output - 256;

  return output;
}


float
readFloat ()
{
  float output;
  byte f_byte_array[4];
  
  for (int i = 0; i < 4; i++) {
    f_byte_array[i] = Wire.read ();
  }

  output = *(float *) &f_byte_array;

  return output;
}

void 
receiveEvent (int length)
{
  float f_float_converter = 0.0;
  int i_float_converter = 0;
  byte trans_type;

  trans_type = Wire.read ();

  switch (trans_type) 
  {
    case PITCH_TRANS:
      angle = readInt ();

      break;
    
    case DEPTH_TRANS:
      i_float_converter = readInt ();
      f_float_converter = i_float_converter / 100; 
      f_float_converter = round(f_float_converter);

      depth = (int) f_float_converter;

      break;

    case HEADING_TRANS:
      heading = readFloat ();

      break;

    default:
      byte val;
    
      Serial.print ("Wrong value recieved: ");
      Serial.print (trans_type);

      while (Wire.available() > 0) {
        val = Wire.read ();
        Serial.println (val);
      }
  }
}
