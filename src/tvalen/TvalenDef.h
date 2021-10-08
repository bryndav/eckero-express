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
};

typedef enum {
  WAIT_FOR_GPS = 10,
  PLAN_COURSE = 20,
  NORMAL_OPERATIONS = 40,
  RADIO_CTRL = 60,
  LONGITUDE_PRINT = 1,
  LATITUDE_PRINT = 2,
  HEADING_PRINT = 3,
  STATE_PRINT = 4,
  SERVO_PRINT = 5,
  DISTANCE_PRINT = 6,
  IMUMAG_PRINT = 7,
  IMUSYS_PRINT = 8,
  IMUGYR_PRINT = 9,
  IMUACC_PRINT = 10
};

#endif
