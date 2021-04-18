#include <Arduino_LSM9DS1.h>

float x, y, z;
unsigned long last_sampel;
unsigned long last_debug_print;

void setup () {
  Serial.begin(115200);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while(1);
  }

}


void loop () {
  const int debug_rate = 1000;
  const int sampel_period = 20;
  unsigned long now = millis ();

  if ((now - last_sampel) >= sampel_period) {  
    if(IMU.gyroscopeAvailable()){
      IMU.readGyroscope(x, y, z);
    }
  }

  if (now - last_debug_print > debug_rate) {
    debugPrint();
    
    last_debug_print = now;
  } 
}

void debugPrint(){
  Serial.print("Gyroscope sample rate: ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.print("\t\t");
  Serial.print("Gyro X: ");
  Serial.print(x);
  Serial.print("\t\t");
  Serial.print("Gyro Y: ");
  Serial.print(y);
  Serial.print("\t\t");
  Serial.print("Gyro Z: ");
  Serial.println(z);
}
