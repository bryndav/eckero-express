/* Enkel tidsstyrning av två utgångar */

#define rightPump 2 
#define leftPump 4

#define state_1 5000
#define state_2 10000
#define state_3 13000
#define state_4 16000

bool checkState(int state);
void changeState(int state);

unsigned long prevMillis = 0;
int currState = 0; 

uint8_t leftPumpVal = 0;
uint8_t rightPumpVal = 0;

void setup() {
	pinMode(rightPump, OUTPUT);
	pinMode(leftPump, OUTPUT);

	// Make sure outputs are low at start
	digitalWrite(rightPump, LOW);
	digitalWrite(leftPump, LOW);
}

void loop() {
	bool stateChange = false;
	unsigned long currMillis = 0;
	unsigned long diff = 0;

	currMillis = millis();
	diff = currMillis - prevMillis;

	if (diff <= state_1){
		currState = state_1;
	}else if (diff > state_1 && diff <= state_2){
		currState = state_2;
	}else if (diff > state_2 && diff <= state_3){
		currState = state_3;
	}else if (diff > state_3 && diff <= state_4){
		currState = state_4;
	}else{
		prevMillis = currMillis;
	}

	stateChange = checkState(currState);

	if (stateChange){
		changeState(currState);
	}
}

bool checkState(int state){
	bool returnVal = false;

	if (state == state_1){
		returnVal = (leftPumpVal == 0 && rightPumpVal == 0) ? false : true;
	}else if (state == state_2){
		returnVal = (leftPumpVal == 1 && rightPumpVal == 1) ? false : true;
	}else if (state == state_3){
		returnVal = (leftPumpVal == 1 && rightPumpVal == 0) ? false : true;
	}else if (state == state_4){
		returnVal = (leftPumpVal == 0 && rightPumpVal == 1) ? false : true;
	}else{
		returnVal = false;
	}

	return returnVal;
}

void changeState(int state){
	if (state == state_1){
		leftPumpVal = 0;
		rightPumpVal = 0;

		digitalWrite(leftPump, leftPumpVal);
		digitalWrite(rightPump, rightPumpVal);
	}else if (state == state_2){
		leftPumpVal = 1;
		rightPumpVal = 1;

		digitalWrite(leftPump, leftPumpVal);
		digitalWrite(rightPump, rightPumpVal);
	}else if (state == state_3){
		leftPumpVal = 1;
		rightPumpVal = 0;

		digitalWrite(leftPump, leftPumpVal);
		digitalWrite(rightPump, rightPumpVal);
	}else if (state == state_4){
		leftPumpVal = 0;
		rightPumpVal = 1;

		digitalWrite(leftPump, leftPumpVal);
		digitalWrite(rightPump, rightPumpVal);
	}else{
		leftPumpVal = 0;
		rightPumpVal = 0;

		digitalWrite(leftPump, leftPumpVal);
		digitalWrite(rightPump, rightPumpVal);
	}

}