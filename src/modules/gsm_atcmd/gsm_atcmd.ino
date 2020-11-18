#include <SoftwareSerial.h>

void updateSerial();
void sendSMS();
char* readSMS();
int parseSMS(char* content);

SoftwareSerial mySerial(10, 11);

void setup() {
	Serial.begin(9600);

	mySerial.begin(9600);

	Serial.println("Initializing...");
	delay(1000);

  Serial.println("Sending AT");
	mySerial.println("AT"); // Handshake, returns OK if successfull
	updateSerial();
 Serial.println("Sending ATI");
	mySerial.println("ATI"); // Reads GSM information
	updateSerial();
 Serial.println("Sending AT+CBC");
	mySerial.println("AT+CBC"); // Reads battery status
	updateSerial();
 Serial.println("Sending AT+CSQ");
	mySerial.println("AT+CSQ"); // Signal quality test, range 0-31;
	updateSerial();
 Serial.println("Sending AT+CCID");
	mySerial.println("AT+CCID"); // Read SIM information, confirms SIM inserted
	updateSerial();
 Serial.println("Sending AT+CREG");
	mySerial.println("AT+CREG?"); // Checks network registration
	updateSerial();
 Serial.println("Sending AT+COPS?");
	mySerial.println("AT+COPS?"); // Asks for network operator
	updateSerial();
 Serial.println("Sending AT+CMGF=1");
	mySerial.println("AT+CMGF=1"); // Configure TEXT mode
	updateSerial();
 Serial.println("Sending AT+CNMI=1,2,0,0,0");
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
