/*** Needed libraries ***/
#include "NineAxesMotion.h"
#include <Wire.h>
#include "MS5837.h"

/*** Object initialization ***/
NineAxesMotion gyroSensor;
MS5837 depthSensor;

/*** Global variables ***/
unsigned long now = 0;
unsigned long lastSampel = 0;
unsigned long lastTransmission = 0;
const int sampelPeriod = 20;
const int transmissionPeriod = 100;

int pitch = 0;
float heading = 0.0;
float depth = 0.0;

int returnCode;
byte f_byteArray[4];

void setup(){
    Serial.begin(115200);
    I2C.begin();
    Wire.begin();

    gyroSensor.initSensor();
    gyroSensor.setOperationMode(OPERATION_MODE_NDOF);
    gyroSensor.setUpdateMode(MANUAL);

    depthSensor.init();
    depthSensor.setModel(MS5837::MS5837_30BA);
    depthSensor.setFluidDensity(997);          // kg/m^3 (freshwater, 1029 for seawater)
    delay(5000);
    Serial.println("Sensors initialized");
}

void loop(){

    now = millis();

    if((now - lastSampel) >= sampelPeriod){
        lastSampel = now;
        updateSensors();
        readSensors(&pitch, &depth, &heading);
    }

    if(now - lastTransmission > transmissionPeriod){
      returnCode = sendValues();
    }
    
}

void updateSensors(){
    gyroSensor.updateEuler();
    gyroSensor.updateCalibStatus();
    depthSensor.read();
}

void readSensors(int* pitch, float* depth, float* heading){

    if(*pitch != (int) gyroSensor.readEulerRoll()){
        *pitch = (int) gyroSensor.readEulerRoll();

    }

    if(*depth != depthSensor.depth()){
      *depth = depthSensor.depth();
    }

    if(*heading != gyroSensor.readEulerHeading()){
      *heading = gyroSensor.readEulerHeading();
    }

}

int sendValues(){
    int returnVal;
    float temp_Depth;
  
    // Send pitch value
    Wire.beginTransmission(15);
    Wire.write("P");
    Wire.write(highByte (pitch));
    Wire.write(lowByte (pitch));    
    returnVal = Wire.endTransmission();

    if(returnVal){
      returnVal += 1;
    }

    // Send depth value
    float2Bytes(depth, &f_byteArray[0]);
    Wire.beginTransmission(15);
    Wire.write("D");
    for(int i = 0; i < sizeof(f_byteArray); i++){
      Wire.write(f_byteArray[i]);
    }
    returnVal = Wire.endTransmission();

    // Send heading value
    float2Bytes(heading, &f_byteArray[0]);
    Wire.beginTransmission(15);
    Wire.write("H");
    for(int i = 0; i < sizeof(f_byteArray); i++){
      Wire.write(f_byteArray[i]);
    }
    returnVal = Wire.endTransmission();


    return returnVal;
}

void float2Bytes(float val, byte* bytes_array){
  // Create union of shared memory space
  union {
    float float_variable;
    byte temp_array[4];
  } u;
  
  // Overite bytes of union with float variable
  u.float_variable = val;
  // Assign bytes to input array
  memcpy(bytes_array, u.temp_array, 4);
}
