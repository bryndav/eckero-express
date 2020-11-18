#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

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
	_Bool valid;
};

float degreeConversion(char* nmea);
struct Position getPos(int tries);

int main() {
	struct Position currPos;

	currPos = getPos(30);

	if (currPos.valid){
		printf("Time: \t\t\t%s\n", currPos.UTCtime);
		printf("Latitude: \t\t%f\n", currPos.latitude);
		printf("Latitude direction: \t%c\n", currPos.latitudeDir);
		printf("Longitude: \t\t%f\n", currPos.longitude);
		printf("Longitude direction: \t%c\n", currPos.longitudeDir);
		printf("Bearing: \t\t%f\n", currPos.bearing);
		printf("Message quality: \t%d\n", currPos.quality);
		printf("Number of satellites: \t%d\n", currPos.numSats);
		printf("Altitude: \t\t%f\n", currPos.altitude);	
	}else {
		printf("Invalid position......\n");
	}
}

struct Position getPos(int tries){
	char GPRMC[88] = "220516,A,5133.82,N,00042.24,W,173.8,231.8,130694,004.2,W*70\r\n";
	char NMEA[88] = "092750.000,5321.6802,N,00630.3372,W,2,8,1.03,61.7,M,55.2,M,,*76\r\n";
	struct Position currPos = {" ", 0.0, 'F', 0.0, 'F', 0, 0, 0.0, 0.0, false};
	char work[12];
	char *comp;

	if (strlen(NMEA) < 70 && tries > 0){
		getPos(tries -1);
	}

	if (tries > 0){
			
		comp = strtok(NMEA, ",");

		// Store UTC time in struct
		strcpy(currPos.UTCtime, comp);

		// Move on to latitude
		comp = strtok(NULL, ",");	
		strcpy(work, comp);	
		currPos.latitude = degreeConversion(work);

		// Store N or S in direction
		comp = strtok(NULL, ",");	
		currPos.latitudeDir = *comp;	

		// Move on to longitude
		comp = strtok(NULL, ",");	
		strcpy(work, comp);	
		currPos.longitude = degreeConversion(work);	

		// Store E or W in direction
		comp = strtok(NULL, ",");	
		currPos.longitudeDir = *comp;

		// Store message quality
		comp = strtok(NULL, ",");
		currPos.quality = atoi(comp);

		// Store number of satellites
		comp = strtok(NULL, ",");
		currPos.numSats= atoi(comp);

		// Skip to mean sea level
		strtok(NULL, ",");
		strtok(NULL, ",");
		strtok(NULL, ",");
		
		// Store mean sea level
		comp = strtok(NULL, ",");
		currPos.altitude = (float) atof(comp);		
	}

	if(tries > 0){
		comp = strtok(GPRMC, ",");

		//Skip information until bearing
		for(int i = 0; i < 7; i++){
			comp = strtok(NULL, ",");
		}

		currPos.bearing = atof(comp);
	}

	// Check if position should be considered valid
	if(currPos.quality < 2 || currPos.numSats < 6){
		if(tries > 0){
			getPos(tries -1);	
		}
	}else{
		currPos.valid = true;
	}

	return currPos;	
}

float degreeConversion(char* nmea){
	float degrees = 0.0;
	float decimals = 0.0;
	int p = 1;

	// Conversion from some format into degrees
	for(int i = 0; i < strlen(nmea); i++){

		if(i == 4){
			continue;
		}

		if (p >= 0){
			degrees += (float)(nmea[i] - '0') * pow(10, p);
		}
		else{
			decimals += (float)(nmea[i] - '0') * pow(10, p + 2);
		}

		p--;
	}

	decimals = decimals / 60.0;
	degrees += decimals;

	return degrees;
}
