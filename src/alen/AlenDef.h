#ifndef _ALENDEF_H_
#define _ALENDEF_H_

typedef struct Sensors {
  int pitch;
  int pressure;
  int depth;
  float heading;
  float temperature;
  float acceleration;
}Sensors;

typedef struct Pos {
  char UTCtime[10];
  float latitude;
  char latitudeDir;
  float longitude;
  char longitudeDir;
  int quality;
  int numSats;
  float altitude;
  float bearing;
  bool valid;
}Position;

typedef struct pid {
  int setpoint;
  double total_error;
  double last_error;
  double Kp; //proportional gain
  double Ki; //integral gain
  double Kd; //derivative gain
  
  int T; //sample time in milliseconds (ms)
  unsigned long last_time;

  double control_signal;
  int max_control;
  int min_control;
}pidData;

#endif
