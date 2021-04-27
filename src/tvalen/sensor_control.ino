void
readHeading() {
  doNMeasurements (50, magX, magY, magZ);        //Average measurements to reduce noise
  
  imu_heading = atan2(magY,magX)*180/PI +180;
  inclanation = atan(-magZ/sqrt(magX*magX +magY*magY)) *180/PI;    // by definition a positive inclination means the Z component is negative
}

void doNMeasurements(unsigned int N, float& averX, float& averY, float& averZ) 
{    
  float x, y, z;
  
  averX=0; averY =0; averZ =0;
  
  for (int i = 1; i <= N; i++) { 
    while (!IMU.magnetAvailable());
    
    IMU.readMagnet(x, y, z);
    averX += x; averY += y;  averZ += z;
  } 
  
  averX /= N;    averY /= N;  averZ /= N;
}
