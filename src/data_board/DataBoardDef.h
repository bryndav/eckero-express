#ifndef _DATABOARDDEF_H_
#define _DATABOARDDEF_H_

typedef struct Pos {
  char utc_time[10];
  float latitude;
  char latitude_dir;
  float longitude;
  char longitude_dir;
  int quality;
  int num_sats;
  float altitude;
  float bearing;
  bool valid;
}Position;

typedef enum {
  PITCH_TRANS = 10,
  PITCH_REQ = 11,
  HEADING_TRANS = 12,
  HEADING_REQ = 13,
  DEPTH_TRANS = 20,
  DEPTH_REQ = 21,
  GPS_REQ = 30,
  LONG_TRANS = 31,
  LAT_TRANS = 32,
  GPS_UNAVAILABLE = 33
} trans_def;

#endif
