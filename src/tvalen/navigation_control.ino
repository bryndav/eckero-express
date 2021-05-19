struct GPSData 
getPos()
{
  char NMEA[88];
  char GPRMC[88];
  struct GPSData tmpPos = {" ", 0.0, 'F', 0.0, 'F', 0, 0, 0.0, 0.0, false};
  int index = 0;
  char work[12];
  char *comp;

  // Grab GPGGA message containing most relavant position data
  if (gpsSerial.find("$GPGGA,")){
    index = gpsSerial.readBytesUntil(0x0D, NMEA, 88);
    
    // Uncomment to see raw serial string
    //Serial.println(NMEA);
  }

  // Grab GPRMC message containing the bearing in true north format
  if (gpsSerial.find("$GPRMC,")){
    gpsSerial.readBytesUntil(0x0D, GPRMC, 88);

    // Uncomment to see raw serial string
    //Serial.println(GPRMC);
  }

  if (index > 60 ){
    comp = strtok(NMEA, ",");

    // Store UTC time in struct
    strcpy(tmpPos.UTCtime, comp);

    // Move on to latitude
    comp = strtok(NULL, ","); 
    strcpy(work, comp); 
    tmpPos.latitude = degreeConversion(work);

    // Store N or S in direction
    comp = strtok(NULL, ","); 
    tmpPos.latitudeDir = *comp; 

    // Move on to longitude
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
  }

  if (index > 60 ){
    comp = strtok(GPRMC, ",");

    //Skip information until bearing
    for(int i = 0; i < 7; i++){
      comp = strtok(NULL, ",");
    }

      (atof(comp) > 0.0 && atof(comp) < 360.0) ? tmpPos.bearing = atof(comp): tmpPos.bearing = gps_data.bearing;
  } 

  // Check if position should be considered valid
  if(tmpPos.quality >= 1 && tmpPos.numSats >= 4 && newCoordinates(tmpPos.longitude, tmpPos.latitude)){
    tmpPos.valid = true;
  }else{
    tmpPos.valid = false;
  }

  return tmpPos;  
}


double 
degreeConversion(char* nmea)
{
  double conversion = 0.0;
  double deg = 0.0;
  double decimals = 0.0;
  int p = 1;
  int i = 0;
  
  // Skip starting 0 in longitude
  if (strlen(nmea) > 10){
    i++;
  }
  
  // Conversion from hours, mins format into degrees
  for(i ;i < strlen(nmea); i++){
  
    if(nmea[i] == '.'){
      continue;
  }
  
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
newCoordinates(double longitude, 
               double latitude)
{
    bool newValue = false;
    double diff = 0.0;
    float coordinateOffset = 0.000005;

    diff = gps_data.longitude - longitude;

    if (diff < 0.0){
      diff = diff * -1.0;
    }

    if (diff > coordinateOffset){
      newValue = true;
    }

    diff = gps_data.latitude - latitude;

    if (diff < 0.0){
      diff = diff * -1.0;
    }
  
    if (diff > coordinateOffset){
      newValue = true;
    }

    return newValue;
}


double
calcDistance(pos curr,
             pos destination)
{
  // Accepts signed decimal degrees without compass direction
  // Negative values indicates west/south (e.g 40.7486, -73.9864)  
  double lat_curr_rad, lat_dest_rad;
  double delta_lat_rad, delta_lon_rad;
  double a, c, distance;
  
  lat_curr_rad = curr.latitude * (pi/180.0);
  lat_dest_rad = destination.latitude * (pi/180.0);
  delta_lat_rad = (curr.latitude - destination.latitude) * (pi/180.0);
  delta_lon_rad = (curr.longitude - destination.longitude) * (pi/180.0);
  
  a = sin(delta_lat_rad/2.0) * sin(delta_lat_rad/2.0) +
      cos(lat_curr_rad) * cos(lat_dest_rad) *
      sin(delta_lon_rad/2.0) * sin(delta_lon_rad/2.0);
  
  c = 2 * atan2(sqrt(a), sqrt(1-a));
  
  distance = earth_radius * c;
  
  return distance;
}


double
calcBearing(pos curr,
            pos destination)
{
    // Accepts signed decimal degrees without compass direction
    // Negative values indicates west/south (e.g 40.7486, -73.9864)
    double lat_curr_rad, lat_dest_rad;
    double delta_lon_rad;
    double y, x, bearing;

    lat_curr_rad = curr.latitude * (pi/180.0);
    lat_dest_rad = destination.latitude * (pi/180.0);
    delta_lon_rad = (curr.longitude - destination.longitude) * (pi/180.0);

    y = sin(delta_lon_rad) * cos(lat_dest_rad);
    x = cos(lat_curr_rad) * sin(lat_dest_rad) -
        sin(lat_curr_rad) * cos(lat_dest_rad) * cos(delta_lon_rad);

    bearing = atan2(y, x);
    bearing = bearing * (180.0/pi);
    bearing = (((int)bearing + 360) % 360);

    if (bearing < 0){
        bearing = bearing * -1;
    }else{
        bearing = 360 - bearing;
    }

    return bearing;
}

void
updatePosition (GPSData gps_pos)
{
  if (gps_pos.valid){
    curr_pos.latitude = gps_pos.latitude;
    curr_pos.longitude = gps_pos.longitude;

    heading = gps_pos.bearing;
  }
}

float
readMagnometerDir(int num_readings)
{
  float mx, my, mz, mag_readings;
  int reading;
  
  for(int i = 0; i < num_readings;){
    if (IMU.magneticFieldAvailable()) {
        IMU.readMagneticField(mx, my, mz);
        mag_readings = (atan2(my, mx) * (180/M_PI));
        
        i++;
      }
  }

  mag_readings = mag_readings / num_readings;

  return mag_readings;
}

void 
printPosition(){

    Serial.print("Time: \t\t\t");
    Serial.println(gps_data.UTCtime);
    Serial.print("Latitude: \t\t");
    Serial.println(gps_data.latitude, 6);
    Serial.print("Latitude direction: \t");
    Serial.println(gps_data.latitudeDir);
    Serial.print("Longitude: \t\t");
    Serial.println(gps_data.longitude, 6);
    Serial.print("Longitude direction: \t");
    Serial.println(gps_data.longitudeDir);
    Serial.print("Bearing: \t\t");
    Serial.println(gps_data.bearing);
    Serial.print("Message quality: \t");
    Serial.println(gps_data.quality);
    Serial.print("Number of satellites: \t");
    Serial.println(gps_data.numSats);
    Serial.print("Altitude: \t\t"); 
    Serial.println(gps_data.altitude, 6);
    Serial.print("Google map URL: \thttp://www.google.com/maps/place/");
    Serial.print(gps_data.latitude, 6);
    Serial.print(",");
    Serial.print(gps_data.longitude, 6);

    Serial.println();
    Serial.println();
}
