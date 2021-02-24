#include "AlenDef.h"

Position 
getPos ()
{
  char NMEA[88];
  char GPRMC[88];
  Position tmpPos = {" ", 0.0, 'F', 0.0, 'F', 0, 0, 0.0, 0.0, false};
  int index;
  char work[12];
  char *comp;

  // Grab GPGGA message containing most relavant position data
  if (gps_serial.find("$GPGGA,")){
    index = gps_serial.readBytesUntil(0x0D, NMEA, 88);
    
    // Uncomment to see raw serial string
    //Serial.println(NMEA);
  }

  // Grab GPRMC message containing the bearing in true north format
  if (gps_serial.find("$GPRMC,")){
    gps_serial.readBytesUntil(0x0D, GPRMC, 88);

    // Uncomment to see raw serial string
    //Serial.println(GPRMC);
  }

  if (index < 60) {
    return tmpPos;
  }

  comp = strtok(NMEA, ",");

  // Store UTC time in struct
  strcpy(tmpPos.UTCtime, comp);

  // Store latitude
  comp = strtok(NULL, ","); 
  strcpy(work, comp); 
  tmpPos.latitude = degreeConversion(work);

  // Store N or S in direction
  comp = strtok(NULL, ","); 
  tmpPos.latitudeDir = *comp; 

  // Store longitude
  comp = strtok(NULL, ","); 
  strcpy(work, comp); 
  tmpPos.longitude = degreeConversion(work);  

  // Store E or W in direction
  comp = strtok(NULL, ","); 
  tmpPos.longitudeDir = *comp;

  // Store message quality
  comp = strtok(NULL, ",");
  tmpPos.quality = atoi(comp);

  // Store number of satellites
  comp = strtok(NULL, ",");
  tmpPos.numSats= atoi(comp);

  // Skip to mean sea level
  strtok(NULL, ",");
  strtok(NULL, ",");
  strtok(NULL, ",");

  // Store mean sea level
  comp = strtok(NULL, ",");
  tmpPos.altitude = (float) atof(comp);

  // Store bearing
  comp = strtok(GPRMC, ",");

  // Skip information until bearing
  for (int i = 0; i < 7; i++){
    comp = strtok(NULL, ",");
  }

  return tmpPos;  
}

bool
validateQuality (int quality,
                 int num_satelites)
{
  return (quality >= 1 && num_satelites >= 3);  
}

float 
degreeConversion (char* nmea)
{
  float conversion = 0.0;
  float deg = 0.0;
  float decimals = 0.0;
  int p = 1;
  int i = 0;

  // Skip starting 0 in longitude
  if (strlen(nmea) > 10)
    i++;

  // Conversion from hours, mins format into degrees
  for (i ; i < strlen (nmea); i++){
    if (nmea[i] == '.')
      continue;

    conversion = (float)(nmea[i] - '0');

    if (p >= 0){
      deg += (conversion * pow(10, p));
    }else{
      decimals += (conversion * pow(10, p + 2));
    }

    p--;
  }
 
  decimals = decimals / 60.0;
  deg += decimals;

  return deg;
}

bool
newCoordinate (float current_coordinate,
               float old_coordinate)
{
  float diff;
  float coordinate_offset = 0.000005;
  bool new_coordinate = false;

  diff = old_coordinate - current_coordinate;

  if (diff < 0.0){
    diff = diff * -1.0;
  }

  if (diff > coordinate_offset){
    new_coordinate = true;
  }

  return new_coordinate;
}
