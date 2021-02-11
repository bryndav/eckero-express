#ifndef _DATABOARDDEF_H_
#define _DATABOARDDEF_H_

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

typedef enum {
  PITCH_TRANS = 10,
  PITCH_REQ = 11,
  HEADING_TRANS = 12,
  HEADING_REQ = 13,
  DEPTH_TRANS = 20,
  DEPTH_REQ = 21
} trans_def;

#endif
