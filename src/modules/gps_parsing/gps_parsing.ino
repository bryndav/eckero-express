#include <SoftwareSerial.h>

SoftwareSerial gps_serial(3, 4); // Rx, Tx

struct Position {
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
};

float degreeConversion(char* nmea);
struct Position getPos();
bool newCoordinates(float longitude, float latitude);
void printPosition();

struct Position currPos = {" ", 0.0, 'F', 0.0, 'F', 0, 0, 0.0, 0.0, false};

void setup() {
  	gps_serial.begin(9600);
	Serial.begin(9600);
}

void loop() {
	struct Position tempPos;

	tempPos = getPos();

  	if (tempPos.valid){
    	currPos = tempPos;
    	printPosition();
  	}

}

struct Position getPos(){
	char NMEA[88];
	char GPRMC[88];
	struct Position tmpPos = {" ", 0.0, 'F', 0.0, 'F', 0, 0, 0.0, 0.0, false};
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

    	(atof(comp) > 0.0 && atof(comp) < 360.0) ? tmpPos.bearing = atof(comp): tmpPos.bearing = currPos.bearing;
	}	

	// Check if position should be considered valid
	if(tmpPos.quality >= 1 && tmpPos.numSats >= 3 && newCoordinates(tmpPos.longitude, tmpPos.latitude)){
		tmpPos.valid = true;
	}else{
		tmpPos.valid = false;
	}

	return tmpPos;	
}

float degreeConversion(char* nmea){
  	float conversion = 0.0;
	float deg = 0.0;
	float decimals = 0.0;
	int p = 1;
  	int i = 0;

  	// Skip starting 0 in longitude
  	if (strlen(nmea) > 10){
  		i++;
  	}

	// Conversion from hours, mins format into degrees
	for(i; i < strlen(nmea); i++){

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

bool newCoordinates(float longitude, float latitude){
  	bool newValue = false;
  	float diff = 0.0;
  	float coordinateOffset = 0.000005;

  	diff = currPos.longitude - longitude;

  	if (diff < 0.0){
    	diff = diff * -1.0;
  	}

  	if (diff > coordinateOffset){
    	newValue = true;
  	}

  	diff = currPos.latitude - latitude;

  	if (diff < 0.0){
    	diff = diff * -1.0;
  	}
  
  	if (diff > coordinateOffset){
    	newValue = true;
  	}

  	return newValue;
}

void printPosition(){

  	Serial.print("Time: \t\t\t");
  	Serial.println(currPos.UTCtime);
  	Serial.print("Latitude: \t\t");
  	Serial.println(currPos.latitude, 6);
  	Serial.print("Latitude direction: \t");
  	Serial.println(currPos.latitudeDir);
  	Serial.print("Longitude: \t\t");
  	Serial.println(currPos.longitude, 6);
  	Serial.print("Longitude direction: \t");
  	Serial.println(currPos.longitudeDir);
  	Serial.print("Bearing: \t\t");
  	Serial.println(currPos.bearing);
  	Serial.print("Message quality: \t");
  	Serial.println(currPos.quality);
  	Serial.print("Number of satellites: \t");
  	Serial.println(currPos.numSats);
  	Serial.print("Altitude: \t\t"); 
  	Serial.println(currPos.altitude, 6);
  	Serial.print("Google map URL: \thttp://www.google.com/maps/place/");
  	Serial.print(currPos.latitude, 6);
  	Serial.print(",");
  	Serial.print(currPos.longitude, 6);

  	Serial.println();
  	Serial.println();
}
