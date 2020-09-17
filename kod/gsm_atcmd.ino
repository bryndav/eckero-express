#include <SoftwareSerial.h>

void updateSerial();
void sendSMS();
char* readSMS();
int parseSMS(char* content);

SoftwareSerial mySerial(3, 2);

void setup() {
	Serial.begin(9600);

	mySerial.begin(9600);

	Serial.println("Initializing...");
	delay(1000);

	mySerial.println("AT"); // Handshake, returns OK if successfull
	updateSerial();
	mySerial.println("ATI"); // Reads GSM information
	updateSerial();
	mySerial.println("AT+CBC"); // Reads battery status
	updateSerial();
	mySerial.println("AT+CSQ"); // Signal quality test, range 0-31;
	updateSerial();
	mySerial.println("AT+CCID"); // Read SIM information, confirms SIM inserted
	updateSerial();
	mySerial.println("AT+CREG?"); // Checks network registration
	updateSerial();
	mySerial.println("AT+COPS?"); // Asks for network operator
	updateSerial();
	mySerial.println("AT+CMGF=1"); // Configure TEXT mode
	updateSerial();
	mySerial.println("AT+CNMI=1,2,0,0,0"); // Configure how incoming messages should be handled
	updateSerial();
	sendSMS();
	updateSerial();
}

void loop(){
	updateSerial();
}

void sendSMS(){
	mySerial.println("AT+CMGS=\"+46733188823"); // Set phone number
	updateSerial();
	mySerial.println("Eckero express com test..."); // Text message to be sent
	updateSerial();
	mySerial.write(26); // Message delimiter?
}

char* readSMS(){
	char message[128];
	char content[24];
	char * comp = NULL;
	int i = 0;

	// Reads in the GSM buffer into message variable
	while(mySerial.available() > 0){
		message[i] = Serial.read();
		i++;
	}

	// Check to see that buffer content is a text msg
	comp = strstr(message, "+CMT");
	
	if (!comp){
		comp = strtok(message, "\r\n");
		comp = strtok(NULL, "\r\n");
		strcpy(content, comp);
	}

	Serial.print(content);

	return content;
}

int parseSMS(char* content){
	char firstChar = content[0];
	char stateLengthChar[1];
	int stateLengthInt;
	int returnVal;

	// Wrong format of text message
	if (strlen(content) > 4){
		returnVal = -1;
	}

	// Parse the length of the state
	for(int i = 2 int p = 0; i < strlen(content); i++){
		stateLengthChar[p] = content[i];
		p++;
	}
	
	stateLengthInt = atoi(stateLengthChar);

	Serial.print("State: ");
	Serial.println(firstChar);
	Serial.print("Lenght: ");
	Serial.println(stateLengthInt);

	// Parse staterepresentation
	switch(firstChar){
		case 'f':
			returnVal = 1;
			break;
		case 'v':
			returnVal = 2;
			break;
		case 'h':
			returnVal = 3;
			break;
		case 's':
			returnVal = 0;
			break;
		default:
			returnVal = -1;
	}

	return returnVal;
}

void updateSerial() {
	delay(500);
	while(Serial.available()) {
		mySerial.write(Serial.read()); // Console input to GSM
	}
	while(Serial.available()){
		Serial.write(mySerial.read()); // Forwards GSM output to console
	}
}
