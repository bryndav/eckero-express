#include "DataBoardDef.h"

Position 
getPos ()
{
  char nmea[88];
  char gprmc[88];
  Position tmp_pos = {" ", 0.0, 'F', 0.0, 'F', 0, 0, 0.0, 0.0, false};
  int index;
  char work[12];
  char *comp;

  // Grab GPGGA message containing most relavant position data
  if (gpsSerial.find("$GPGGA,")){
    index = gpsSerial.readBytesUntil(0x0D, nmea, 88);
    
    // Uncomment to see raw serial string
    //Serial.println(nmea);
  }

  // Grab gprmc message containing the bearing in true north format
  if (gpsSerial.find("$gprmc,")){
    gpsSerial.readBytesUntil(0x0D, gprmc, 88);

    // Uncomment to see raw serial string
    //Serial.println(gprmc);
  }

  if (index < 60) {
    return tmp_pos;
  }

  comp = strtok(nmea, ",");

  // Store UTC time in struct
  strcpy(tmp_pos.utc_time, comp);

  // Store latitude
  comp = strtok(NULL, ","); 
  strcpy(work, comp); 
  tmp_pos.latitude = degreeConversion(work);

  // Store N or S in direction
  comp = strtok(NULL, ","); 
  tmp_pos.latitude_dir = *comp; 

  // Store longitude
  comp = strtok(NULL, ","); 
  strcpy(work, comp); 
  tmp_pos.longitude = degreeConversion(work);  

  // Store E or W in direction
  comp = strtok(NULL, ","); 
  tmp_pos.longitude_dir = *comp;

  // Store message quality
  comp = strtok(NULL, ",");
  tmp_pos.quality = atoi(comp);

  // Store number of satellites
  comp = strtok(NULL, ",");
  tmp_pos.num_sats= atoi(comp);

  // Skip to mean sea level
  strtok(NULL, ",");
  strtok(NULL, ",");
  strtok(NULL, ",");

  // Store mean sea level
  comp = strtok(NULL, ",");
  tmp_pos.altitude = (float) atof(comp);

  // Store bearing
  comp = strtok(gprmc, ",");

  // Skip information until bearing
  for (int i = 0; i < 7; i++){
    comp = strtok(NULL, ",");
  }

  // Check position quality
  tmp_pos.valid = validateQuality(tmp_pos.quality, tmp_pos.num_sats);

  return tmp_pos;
}

bool
validateQuality (int quality,
                 int num_sats)
{
  return (quality >= 1 && num_sats >= 5);  
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
newCoordinates (float longitude, 
                float latitude,
                float old_long,
                float old_lat)
{
  bool new_value = false;
  float diff = 0.0;
  float coordinate_offset = 0.000005;

  diff = old_long - longitude;

  if (diff < 0.0){
    diff = diff * -1.0;
  }

  if (diff > coordinate_offset){
    new_value = true;
  }

  diff = old_lat - latitude;

  if (diff < 0.0){
    diff = diff * -1.0;
  }

  if (diff > coordinate_offset){
    new_value = true;
  }

  return new_value;
}
