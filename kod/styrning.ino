/*****************************************************************************
*																			 *
*	Simple program to blink two leds in different patterns. The program is	 *
*	made a bit more complex by implementing a state machine to handle The 	 *	
*	flashing of leds. 														 *
*																			 *
*****************************************************************************/

#define rightPump 2 
#define leftPump 4
#define ON 1
#define OFF 0

enum state_def {
	state_1 = 5000,
	state_2 = 10000,
	state_3 = 13000,
	state_4 = 16000,	
};

struct State {
	int state_nr;
	uint8_t leftPumpVal;
	uint8_t rightPumpVal;
	int state_start;
	int state_end;
	struct State* next;
};

struct State state1 = { state_1,  OFF, OFF, 0, state_1, NULL };
struct State state2 = { state_2,  ON, ON, state_1, state_2, NULL };
struct State state3 = { state_3,  ON, OFF, state_2, state_3, NULL };
struct State state4 = {state_4,  OFF, ON, state_3, state_4, NULL};
struct State* currentState = NULL;

// Counter and compare values
const uint16_t t1_load = 0;
const uint16_t t1_comp = 20000; // Gives 10 ms interrupts 

unsigned long currMillis = 0;

bool checkState();
void changeState();

void setup() {
	// Linking the state machine
	state1.next = &state2;
	state2.next = &state3;
	state3.next = &state4;
	state4.next = &state1;
	currentState = &state1;

	// Reset Timer1 control register A
	TCCR1A = 0;

	// Set prescaler to 256
	TCCR1B &= ~(1 << CS12);
	TCCR1B |= (1 << CS11);
	TCCR1B &= ~(1 << CS10);

	// Reset Timer1 and set compare value
	TCNT1 = t1_load;
	OCR1A = t1_comp;

	// Enable Timer1 compare interrupt
	TIMSK1 = (1 << OCIE1A);

	// Enable global interrupts
	sei();

	pinMode(rightPump, OUTPUT);
	pinMode(leftPump, OUTPUT);

	// Make sure outputs are low at start
	digitalWrite(rightPump, LOW);
	digitalWrite(leftPump, LOW);
}

void loop() {
	bool stateChange = false;

	stateChange = checkState();

	if(stateChange){
		changeState();
	}
}


/*****************************************************************************
*																			 *
*	Checks if the time passed since boot has surpassed the max state time.	 *
*	Returns a boolean with true if max state time has passed i.e we need	 *
*	to change the state. Else it returns false.								 *
*																			 *
*****************************************************************************/

bool checkState() {
	static unsigned long prevMillis = 0;
	unsigned long diff = 0;
	bool changeState = false;

	// If there is a overflow reset prevMillis aswell
	if(prevMillis > currMillis){
		prevMillis = 0;
	}

	diff = currMillis - prevMillis;

	if ((int) diff > currentState->state_end){
		changeState = true;

		if(currentState->state_nr == state_4){
			prevMillis = currMillis;
		}
	}

	return changeState;
}


/*****************************************************************************
*																			 *
*	Function that handles the actual change of state. Current state pointer	 *
*	moves and new values are written to the leds. 							 *
*																			 *
*****************************************************************************/

void changeState() {

	currentState = currentState->next;

	digitalWrite(leftPump, currentState->leftPumpVal);
	digitalWrite(rightPump, currentState->rightPumpVal);
}

/*****************************************************************************
*																			 *
*	Interrupt service rutine, increases the timer value with 10 each tick.	 *
*	The timer still gives milliseconds since start but with lower sampling.  *
*																			 *
*****************************************************************************/

ISR(TIMER1_COMPA_vect){
	TCNT1 = t1_load;
	currMillis = currMillis + 10;
}