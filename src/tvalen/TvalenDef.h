#ifndef _ALENDEF_H_
#define _ALENDEF_H_

struct GPSData {
  char UTCtime[10];
  double latitude;
  char latitudeDir;
  double longitude;
  char longitudeDir;
  int quality;
  int numSats;
  float altitude;
  float bearing;
  bool valid;
};

typedef struct pidData {
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
};

typedef struct pos {
  double latitude;
  double longitude;
  struct pos* next;
};

typedef enum {
  WAIT_FOR_GPS = 10,
  MAG_OPERATIONS = 30,
  
  PLAN_COURSE = 20,
  NORMAL_OPERATIONS = 40,
  TARGET_REACHED = 50,
};

#endif
