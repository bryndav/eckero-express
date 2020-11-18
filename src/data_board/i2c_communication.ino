#include <Wire.h>

const byte slave_address = 15;

int 
sendInt (byte   identifier,
         int*   value)
{
  int returnVal;

  Wire.beginTransmission (slave_address);
  Wire.write (identifier);
  Wire.write (highByte (*value));
  Wire.write (lowByte (*value));    
  returnVal = Wire.endTransmission ();

  return returnVal;
}

int
sendFloat(byte    identifier,
          float*  value)
{
  byte f_byteArray[4];
  float temp_Depth;
  int return_val;
  
  float2Bytes (*value, &f_byteArray[0]);
  Wire.beginTransmission (slave_address);
  Wire.write (identifier);
  
  for (int i = 0; i < sizeof (f_byteArray); i++) {
    Wire.write (f_byteArray[i]);
  }
  return_val = Wire.endTransmission ();

  return return_val;
}

void 
float2Bytes (float  val, 
             byte*  bytes_array) 
{
  union {
    float float_variable;
    byte temp_array[4];
  } u;
    
  u.float_variable = val;
  memcpy (bytes_array, u.temp_array, 4);
}
