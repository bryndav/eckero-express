#ifndef _LOGICBOARDDEF_H_
#define _LOGICBOARDDEF_H_

typedef enum {
  PITCH_TRANS = 10,
  PITCH_REQ = 11,
  HEADING_TRANS = 12,
  HEADING_REQ = 13,
  DEPTH_TRANS = 20,
  DEPTH_REQ = 21
} trans_def;

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
