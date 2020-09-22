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

void updateSerial() {
	delay(500);
	while(Serial.available()) {
		mySerial.write(Serial.read()); // Console input to GSM
	}
	while(Serial.available()){
		Serial.write(mySerial.read()); // Forwards GSM output to console
	}
}
