#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#define OFF 0
#define ON 1

enum state_def {
	state_1 = 1,
	state_2 = 2,
	state_3 = 3,
	state_4 = 4,	
};

struct State {
	int state_nr;
	int leftPumpVal;
	int rightPumpVal;
	int state_start;
	int state_end;
};

struct State state1 = { state_1,  OFF, OFF, 0, 0 };
struct State state2 = { state_2,  ON, ON, 0, 0 };
struct State state3 = { state_3,  ON, OFF, 0, 0 };
struct State state4 = {state_4,  OFF, ON, 0, 0 };
struct State *currentState = &state1;

_Bool readSMS(char* message);
struct State parseSMS(char* content);

int main() {	
	char message[24];
	_Bool newMessage;
	struct State newState;

	newMessage = readSMS(message);

	if (newMessage){
		newState = parseSMS(message);
		//currentState = &newState;
	}

	if (currentState->state_nr != newState.state_nr){
		printf("Change state from: %d to %d\n", currentState->state_nr, newState.state_nr);
		printf("State length set to: %d\n", newState.state_end);
	}

	return(0);
}

_Bool readSMS(char* message){
	char sampleMessage[256] = "+CMT: \"+46733188823\", \"\", \"20/09/19,11:50:30+22\", \r\nv 50";
	char *comp;
	_Bool newMessage = false;

	comp = strstr(sampleMessage, "+CMT");

	if(comp) {
		comp = strtok(sampleMessage, "\r\n");
		comp = strtok(NULL, "\r\n");
		strcpy(message, comp);
		newMessage = true;
	}

	return newMessage;
}

struct State parseSMS(char* content){
	char firstChar;
	char stateLengthChar[2];
	int i = 2; 
	int p = 0;
	int stateLengthInt;
	struct State returnVal;

	firstChar = content[0];

	// Check length of text message, if to long sets state 1
	if (strlen(content) > 4){
		printf("Invalid format, string to long...Setting state to 4\n");
		firstChar = 's';		
	}

	// Parse length of state
	for(; i < strlen(content); i++){
		stateLengthChar[p] = content[i];
		p++;
	}

	stateLengthInt = atoi(stateLengthChar) * 1000;

	// From the inital letter select state
	switch(firstChar){
		case 'f':
			returnVal = state1;
			break;
		case 'v':
			returnVal = state2;
			break;
		case 'h':
			returnVal = state3;
			break;
		case 's':
			returnVal = state4;
			break;
		default:
			returnVal = state4;
	}

	returnVal.state_end = stateLengthInt;
	
	return returnVal;
}