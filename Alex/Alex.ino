#include "buffer.h"
#include <serialize.h>
#include <stdarg.h>
#include "packet.h"
#include "constants.h"
#include <math.h>


typedef enum
{
	STOP=0,
	FORWARD=1,
	BACKWARD=2,
	LEFT=3,
	RIGHT=4
} TDirection;
volatile TDirection dir = STOP;


/*
 * Alex's configuration constants
 */

// Number of ticks per revolution from the 
// wheel encoder.

#define LEFT_COUNTS_PER_REV 178
#define RIGHT_COUNTS_PER_REV 180

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled 
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC (6.4*3.14159)
#define ALEX_LENGTH 6
#define ALEX_BREADTH 7

//compute these in the setup function
float alexDiagonal = 0;
float alexCirc = 0;

// Motor control pins. You need to adjust these till+
// Alex moves in the correct direction
#define LF                  5   // Left forward pin
#define LR                  6   // Left reverse pin
#define RF                  10  // Right forward pin
#define RR                  11  // Right reverse pin
#define LEFT_FORWARD_REGISTER OCR0B
#define LEFT_REVERSE_REGISTER OCR0A
#define RIGHT_FORWARD_REGISTER OCR1B
#define RIGHT_REVERSE_REGISTER OCR2A
//color sensor setup definitions
//port D
#define COLOR_S0 0b00010000 //pin4
#define COLOR_S1 0b10000000 //pin 7
//port B
#define COLOR_S2 0b00000001 //pin 8
#define COLOR_S3 0b00010000 //pin 12
#define COLOR_OUTPUT 0b00100000 //pin 13

//Colour Frequency output`
int frequency = 0;


/*
 *    Alex's State Variables
 */

// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long leftForwardTicks; 
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks; 
volatile unsigned long rightReverseTicks;

//counter for wheel turns

volatile unsigned long leftForwardTicksTurns; 
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns; 
volatile unsigned long rightReverseTicksTurns;

// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

//keep track of dist covered
unsigned long deltaDist;
unsigned long newDist;
unsigned long leftDeltaTicks;
unsigned long rightDeltaTicks;
unsigned long leftTargetTicks;
unsigned long rightTargetTicks;

//serialisation stuff
#define BUFFER_LEN 256
//buffers for uart
volatile TBuffer _recvBuffer;
volatile TBuffer _xmitBuffer;

/*
 * 
 * Alex Communication Routines.
 * 
 */

TResult readPacket(TPacket *packet)
{
	// Reads in data from the serial port and
	// deserializes it.Returns deserialized
	// data in "packet".

	char buffer[PACKET_SIZE];
	int len;

	len = readSerial(buffer);

	if(len == 0)
		return PACKET_INCOMPLETE;
	else
		return deserialize(buffer, len, packet);

}

void sendStatus()
{
	// Implement code to send back a packet containing key
	// information like leftTicks, rightTicks, leftRevs, rightRevs
	// forwardDist and reverseDist
	// Use the params array to store this information, and set the
	// packetType and command files accordingly, then use sendResponse
	// to send out the packet. See sendMessage on how to use sendResponse.
	//
	TPacket statusPacket;
	statusPacket.packetType = PACKET_TYPE_RESPONSE;
	statusPacket.command = RESP_STATUS;
	statusPacket.params[0] = leftForwardTicks;
	statusPacket.params[1] = rightForwardTicks;
	statusPacket.params[2] = leftReverseTicks;
	statusPacket.params[3] = rightReverseTicks;
	statusPacket.params[4] = leftForwardTicksTurns;
	statusPacket.params[5] = rightForwardTicksTurns;
	statusPacket.params[6] = leftReverseTicksTurns;
	statusPacket.params[7] = rightReverseTicksTurns;
	statusPacket.params[8] = forwardDist;
	statusPacket.params[9] = reverseDist;
	sendResponse(&statusPacket);

}

void sendMessage(const char *message)
{
	// Sends text messages back to the Pi. Useful
	// for debugging.

	TPacket messagePacket;
	messagePacket.packetType=PACKET_TYPE_MESSAGE;
	strncpy(messagePacket.data, message, MAX_STR_LEN);
	sendResponse(&messagePacket);
}

void dbprintf(char *format, ...) {
	va_list args;
	char buffer[128];
	va_start(args, format);
	vsprintf(buffer, format, args);
	sendMessage(buffer);
}

void sendBadPacket()
{
	// Tell the Pi that it sent us a packet with a bad
	// magic number.

	TPacket badPacket;
	badPacket.packetType = PACKET_TYPE_ERROR;
	badPacket.command = RESP_BAD_PACKET;
	sendResponse(&badPacket);

}

void sendBadChecksum()
{
	// Tell the Pi that it sent us a packet with a bad
	// checksum.

	TPacket badChecksum;
	badChecksum.packetType = PACKET_TYPE_ERROR;
	badChecksum.command = RESP_BAD_CHECKSUM;
	sendResponse(&badChecksum);  
}

void sendBadCommand()
{
	// Tell the Pi that we don't understand its
	// command sent to us.

	TPacket badCommand;
	badCommand.packetType=PACKET_TYPE_ERROR;
	badCommand.command=RESP_BAD_COMMAND;
	sendResponse(&badCommand);

}

void sendBadResponse()
{
	TPacket badResponse;
	badResponse.packetType = PACKET_TYPE_ERROR;
	badResponse.command = RESP_BAD_RESPONSE;
	sendResponse(&badResponse);
}

void sendOK()
{
	TPacket okPacket;
	okPacket.packetType = PACKET_TYPE_RESPONSE;
	okPacket.command = RESP_OK;
	sendResponse(&okPacket);  
}

void sendResponse(TPacket *packet)
{
	// Takes a packet, serializes it then sends it out
	// over the serial port.
	char buffer[PACKET_SIZE];
	int len;

	len = serialize(buffer, packet, sizeof(TPacket));
	writeSerial(buffer, len);
}


/*
 * Setup and start codes for external interrupts and 
 * pullup resistors.
 * 
 */
// Enable pull up resistors on pins 2 and 3
void enablePullups()
{
	// Use bare-metal to enable the pull-up resistors on pins
	// 2 and 3. These are pins PD2 and PD3 respectively.
	// We set bits 2 and 3 in DDRD to 0 to make them inputs. 
	DDRD &= ~(0b00001100);
	PORTD |= 0b00001100;

}

// Functions to be called by INT0 and INT1 ISRs.
void leftISR()
{
	switch(dir)
	{
		case(FORWARD): 
			leftForwardTicks++;
			//Serial.print("LEFT Forward: ");
			//Serial.println(leftForwardTicks);
			//forwardDist = (float)(WHEEL_CIRC / LEFT_COUNTS_PER_REV) * (leftForwardTicks);
		case(BACKWARD):
			leftReverseTicks++;
			//Serial.print("LEFT Reverse: ");
			//Serial.println(leftReverseTicks);
			//reverseDist = (float)(WHEEL_CIRC / LEFT_COUNTS_PER_REV) * (leftReverseTicks);
		case(LEFT):
			leftReverseTicksTurns++;
		case(RIGHT):
			leftForwardTicksTurns++;
	}

	if(dir == FORWARD)
	{
		forwardDist = (unsigned long) ((float) leftForwardTicks / LEFT_COUNTS_PER_REV * WHEEL_CIRC);
	}
	if(dir == BACKWARD)
	{
		reverseDist = (unsigned long) ((float) leftReverseTicks / LEFT_COUNTS_PER_REV * WHEEL_CIRC);
	}
}

void rightISR()
{
	switch(dir)
	{
		case(FORWARD):
			rightForwardTicks++;
			//Serial.print("RIGHT Forward: ");
			//Serial.println(rightForwardTicks);
			//forwardDist = (float)(WHEEL_CIRC / RIGHT_COUNTS_PER_REV) * (rightForwardTicks);
		case(BACKWARD):
			rightReverseTicks++;
			//Serial.print("RIGHT Reverse: ");
			//Serial.println(rightReverseTicks);
			//forwardDist = (float)(WHEEL_CIRC / RIGHT_COUNTS_PER_REV) * (rightReverseTicks);
		case(LEFT):
			rightForwardTicksTurns++;
		case(RIGHT):
			rightReverseTicksTurns++;          
	}
}

// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered. Use bare-metal.
void setupEINT()
{
	// Use bare-metal to configure pins 2 and 3 to be
	// falling edge triggered. Remember to enable
	// the INT0 and INT1 interrupts.

	EICRA = 0b00001010;

	DDRD &= 0b11110011;

	EIMSK = 0b00000011;

}

// Implement the external interrupt ISRs below.
// INT0 ISR should call leftISR while INT1 ISR
// should call rightISR.


ISR(INT0_vect) 
{
	leftISR();
}

ISR(INT1_vect) 
{
	rightISR();
}
// Implement INT0 and INT1 ISRs above.

/*
 * Setup and start codes for serial communications
 * 
 */
// Set up the serial connection. For now we are using 
// Arduino Wiring, you will replace this later
// with bare-metal code.

ISR(USART_RX_vect)
{
	unsigned char data = UDR0;
	//note that it will fail silently if duffer is full
	writeBuffer(&_recvBuffer, data);
}

ISR(USART_UDRE_vect)
{
	unsigned char data;
	TBufferResult result;
  	result = readBuffer(&_xmitBuffer, &data);
	if(result == BUFFER_OK)
	{
		UDR0 = data;
	}else if(result == BUFFER_EMPTY)
	{
		//disable udre interrupt after done transmitting buffer
		UCSR0B &= 0b11011111;
	}
}

void setupSerial()
{
	// To replace later with bare-metal.
	//Serial.begin(9600);
	
	//set baud rate to 9600
  initBuffer(&_recvBuffer, BUFFER_LEN);
  initBuffer(&_xmitBuffer, BUFFER_LEN);
	UBRR0L = 103;
	UBRR0H = 0;

	//asynchronous uart, no parity, 1 stop bit, 8 bit data size per packet
	UCSR0C = 0b00000110;

	//0 the ucsr0a register first
	UCSR0A = 0;

}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial()
{
	// Empty for now. To be replaced with bare-metal code
	// later on.
	//enable interrupts for uarts and enabling transmitters and receivers
	UCSR0B = 0b10111000;

}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid. 
// This will be replaced later with bare-metal code.

int readSerial(char *buffer)
{

	int count=0;
	
	/*TBufferResult result;
	do
	{
		result = readBuffer(&_recvBuffer, (unsigned char *)&buffer[count]);
		if(result == BUFFER_OK)
		{
			count++;
		}
	}while(result == BUFFER_OK);*/
	//while(Serial.available())
	//	buffer[count++] = Serial.read();
	TBufferResult result = BUFFER_OK;
	for(count = 0; dataAvailable(&_recvBuffer) && result == BUFFER_OK; count +=1)
	{
		result = readBuffer(&_recvBuffer, (unsigned char*)&buffer[count]);
	}

	return count;

}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const char *buffer, int len)
{
	//Serial.write(buffer, len);
	/*TBufferResult result = BUFFER_OK;
	for (int pos = 0; pos < len && result == BUFFER_OK; pos++)
	{
		result = writeBuffer(&_xmitBuffer, buffer[pos]);
	}
	//load first bit of data to send out
	UDR0 = buffer[0];*/
	TBufferResult result = BUFFER_OK;
	for (int pos = 1; pos < len && result == BUFFER_OK; pos++)
	{
		result = writeBuffer(&_xmitBuffer, buffer[pos]);
	}
	UDR0 = buffer[0];
	//enable udre interrupt
	UCSR0B |= 0b00100000;
}

/*
 * Alex's motor drivers.
 * 
 */

// Set up Alex's motors. Right now this is empty, but
// later you will replace it with code to set up the PWMs
// to drive the motors.
void setupMotors()
{
	/* Our motor set up is:  
	 *    A1IN - Pin 5, PD5, OC0B
	 *    A2IN - Pin 6, PD6, OC0A
	 *    B1IN - Pin 10, PB2, OC1B
	 *    B2In - pIN 11, PB3, OC2A
	 */

	
	//setup pwm for timers 0 and 2 first since they are the same
	//set prescaler at 256
	TCCR0A = 0b00000001;
	TCCR0B = 0b00000100;
	TCNT0 = 0;
	TCCR2A = 0b00000001;
	TCCR2B = 0b00000100;
	TCNT2 = 0;

	//setup pwm for timer 1
	//mode 3 phase correct 8 bits
	TCCR1A =  0b00000001;
	TCCR1B = 0b00000100;
	TCNT1 = 0;
	 

}

// Start the PWM for Alex's motors.
// We will implement this later. For now it is
// blank.
void startMotors()
{
 DDRD |= 0b01100000;
 DDRB |= 0b00001100;
 TCCR0A |= 0b10100000;
 TCCR1A |= 0b00100000;
 TCCR2A |= 0b10000000;
 LEFT_FORWARD_REGISTER = 0;
 RIGHT_FORWARD_REGISTER = 0;
 LEFT_REVERSE_REGISTER = 0;
 RIGHT_REVERSE_REGISTER = 0;
}

// Convert percentages to PWM values
int pwmVal(float speed)
{
	if(speed < 0.0)
		speed = 0;

	if(speed > 100.0)
		speed = 100.0;

	return (int) ((speed / 100.0) * 255.0);
}

// Move Alex forward "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// move forward at half speed.
// Specifying a distance of 0 means Alex will
// continue moving forward indefinitely.
void forward(float dist, float speed)
{
	dir = FORWARD;
	int val = pwmVal(speed);
  //baremetal wpm setting
  	LEFT_FORWARD_REGISTER = val;
	RIGHT_FORWARD_REGISTER = val;
	if (dist > 0)
	{
		deltaDist = dist;
	} else
	{
		deltaDist = 9999999;
	}
	newDist = forwardDist + deltaDist;

	// For now we will ignore dist and move
	// forward indefinitely. We will fix this
	// in Week 9.

	// LF = Left forward pin, LR = Left reverse pin
	// RF = Right forward pin, RR = Right reverse pin
	// This will be replaced later with bare-metal code.
  // turn on bare metal pwm
    //TCCR0A |= 0b00100000;
    //TCCR2A |= 0b00000000;
    //TCCR1A |= 0b00100000;
   
	//analogWrite(LF, val);
	//analogWrite(RF, val);
	//analogWrite(LR,0);
	//analogWrite(RR, 0);
}

// Reverse Alex "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// reverse at half speed.
// Specifying a distance of 0 means Alex will
// continue reversing indefinitely.
void reverse(float dist, float speed)
{
  
	dir = BACKWARD;
	int val = pwmVal(speed);
  //baremetal wpm setting
	LEFT_REVERSE_REGISTER = val;
	RIGHT_REVERSE_REGISTER = val;
	if (dist > 0)
	{
		deltaDist = dist;
	} else
	{
		deltaDist = 9999999;
	}
	newDist = reverseDist + deltaDist;

	// For now we will ignore dist and 
	// reverse indefinitely. We will fix this
	// in Week 9.
// turn on bare metal pwm
    //TCCR0A |= 0b10000000;
    //TCCR2A |= 0b10000000;
    //TCCR1A |= 0b00000000;
   
	// LF = Left forward pin, LR = Left reverse pin
	// RF = Right forward pin, RR = Right reverse pin
	// This will be replaced later with bare-metal code.
	//analogWrite(LR, val);
	//analogWrite(RR, val);
	//analogWrite(LF, 0);
	//analogWrite(RF, 0);
}

//function to estimate number of wheel ticks needed to turn an angle for left wheel
unsigned long computeLeftDeltaTicks(float ang)
{
	//When Alex is moving in a straight line, he will move WHEEL_CIRC cm forward (or backward) in one
	//wheel revolution. We assume that when Alex is moving in circles “on a dime”, the wheels make
	//the same WHEEL_CIRC cm angular distance in one revolution.
	//The total number of wheel turns required to turn 360 degrees is therefore AlexCirc/WHEEL_CIRC,
	//where AlexCirc is the circumference of the circle made by Alex tunring on a dime. (Once again, to
	//“turn on a dime” means that Alex rotates about its center axis, without moving forward or
	//backward).
	//To turn ang degrees, the number of wheel turns is ang/360.0 * AlexCIRC/WHEEL_CIRC.
	//The number of ticks is ang/360.0 * AlexCIRC/WHEEL_CIRC * COUNTS_PER_REV
	unsigned long leftTicks = (unsigned long) ((ang * alexCirc * LEFT_COUNTS_PER_REV) / (360.0 * WHEEL_CIRC));
	return leftTicks;
}

//function to estimate number of wheel ticks needed to turn an angle for right wheel
unsigned long computeRightDeltaTicks(float ang)
{
	//When Alex is moving in a straight line, he will move WHEEL_CIRC cm forward (or backward) in one
	//wheel revolution. We assume that when Alex is moving in circles “on a dime”, the wheels make
	//the same WHEEL_CIRC cm angular distance in one revolution.
	//The total number of wheel turns required to turn 360 degrees is therefore AlexCirc/WHEEL_CIRC,
	//where AlexCirc is the circumference of the circle made by Alex tunring on a dime. (Once again, to
	//“turn on a dime” means that Alex rotates about its center axis, without moving forward or
	//backward).
	//To turn ang degrees, the number of wheel turns is ang/360.0 * AlexCIRC/WHEEL_CIRC.
	//The number of ticks is ang/360.0 * AlexCIRC/WHEEL_CIRC * COUNTS_PER_REV
	unsigned long rightTicks = (unsigned long) ((ang * alexCirc * RIGHT_COUNTS_PER_REV) / (360.0 * WHEEL_CIRC));
	return rightTicks;
}

// Turn Alex left "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn left indefinitely.
void left(float ang, float speed)
{
	if(ang == 0)
	{
		leftDeltaTicks = 99999999;
		rightDeltaTicks = 99999999;
	}
	else
	{
		leftDeltaTicks = computeLeftDeltaTicks(ang);
		rightDeltaTicks = computeRightDeltaTicks(ang);
	}
	leftTargetTicks = leftReverseTicksTurns + leftDeltaTicks;
	rightTargetTicks = rightForwardTicksTurns + rightDeltaTicks;
	dir = LEFT;
	int val = pwmVal(speed);
    //baremetal wpm setting
	RIGHT_FORWARD_REGISTER = val;
	LEFT_REVERSE_REGISTER = val;

	// For now we will ignore ang. We will fix this in Week 9.
	// We will also replace this code with bare-metal later.
	// To turn left we reverse the left wheel and move
	// the right wheel forward.
 // turn on bare metal pwm
    //TCCR0A |= 0b10000000;
    //TCCR2A |= 0b00000000;
    //TCCR1A |= 0b00100000;
   

	//analogWrite(LR, val);
	//analogWrite(RF, val);
	//analogWrite(LF, 0);
	//analogWrite(RR, 0);
}

// Turn Alex right "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn right indefinitely.
void right(float ang, float speed)
{
	dir = RIGHT;
	int val = pwmVal(speed);
      //baremetal wpm setting
 	LEFT_FORWARD_REGISTER = val;
	RIGHT_REVERSE_REGISTER = val;
	if(ang == 0)
	{
		leftDeltaTicks = 99999999;
		rightDeltaTicks = 99999999;
	}
	else
	{
		leftDeltaTicks = computeLeftDeltaTicks(ang);
		rightDeltaTicks = computeRightDeltaTicks(ang);
	}
	leftTargetTicks = leftForwardTicksTurns + leftDeltaTicks;
	rightTargetTicks = rightReverseTicksTurns + rightDeltaTicks;

	// For now we will ignore ang. We will fix this in Week 9.
	// We will also replace this code with bare-metal later.
	// To turn right we reverse the right wheel and move
	// the left wheel forward.
   // turn on bare metal pwm
    //TCCR0A |= 0b00100000;
    //TCCR2A |= 0b10000000;
    //TCCR1A |= 0b00000000;
   
	//analogWrite(RR, val);
	//analogWrite(LF, val);
	//analogWrite(LR, 0);
	//analogWrite(RF, 0);
}

// Stop Alex. To replace with bare-metal code later.
void stop()
{
	dir = STOP;
	LEFT_FORWARD_REGISTER = 0;
	RIGHT_FORWARD_REGISTER = 0;
	LEFT_REVERSE_REGISTER = 0;
	RIGHT_REVERSE_REGISTER = 0;
	//analogWrite(LF, 0);
	//analogWrite(LR, 0);
	//analogWrite(RF, 0);
	//analogWrite(RR, 0);
 // turn on bare metal pwm
    //TCCR0A &= ~(0b11110000);
    //TCCR2A &= ~(0b11110000);
    //TCCR1A &= ~(0b11110000);
   
}

void setupColorSensor() 
{
  //Set S0, S1, S2 AND S3 to be OUTPUT while output frequency is set to INPUT
  DDRD |= ((COLOR_S0) | (COLOR_S1));
  DDRB |= ((COLOR_S2) | (COLOR_S3));
  DDRB &= ~(COLOR_OUTPUT);

  //Setting frequency scaling to 20% by setting S0 to HIGH and S1 to LOW
  PORTD |= COLOR_S0;
  PORTD &= ~(COLOR_S1);
}

void readColorValues()
{
  //create packet to send
  TPacket colorPacket;
  colorPacket.packetType = PACKET_TYPE_RESPONSE;
  colorPacket.command = RESP_COLOR;

  //Read red color values first

  //Setting red filtered photodiodes to be read
  PORTB &= ~((COLOR_S2) | (COLOR_S3));

  //Reading output frequency
  frequency = pulseIn(13, LOW);
  colorPacket.params[0] = frequency;

  //Printing value on serial monitor
  //Serial.print("R= ");
  //Serial.print(frequency);
  //Serial.print(" ");
  delay(100);

  //Setting Green filtered photodiodes to be read
  PORTB |= ((COLOR_S2) | (COLOR_S3));

  //Reading output frequency
  frequency = pulseIn(13, LOW);
  colorPacket.params[1] = frequency;

  //Printing value on serial monitor
  //Serial.print("G= ");
  //Serial.print(frequency);
  //Serial.print(" ");
  delay(100);
  
  //Setting BLUE filtered photodiodes to be read
  PORTB &= ~(COLOR_S2);
  PORTB |= (COLOR_S3);

  //Reading output frequency
  frequency = pulseIn(13, LOW);
  colorPacket.params[2] = frequency;

  //Printing value on serial monitor
  //Serial.print("B= ");
  //Serial.print(frequency);
  //Serial.print(" ");
  delay(100);
  
  //Serial.println("");
  //determine if color detected is red or green: 1 for red, 0 for green
  if (colorPacket.params[0] < colorPacket.params[1])
  {
  	colorPacket.params[3] = 1;
  }else
  {
  	colorPacket.params[3] = 0;
  }

  sendResponse(&colorPacket);
  
}

/*
 * Alex's setup and run codes
 * 
 */

// Clears all our counters
void clearCounters()
{
	leftForwardTicks=0;
	rightForwardTicks=0;
	leftReverseTicks=0;
	rightReverseTicks=0;
	leftForwardTicksTurns = 0; 
	rightForwardTicksTurns = 0;
	leftReverseTicksTurns = 0; 
	rightReverseTicksTurns = 0;
	leftRevs=0;
	rightRevs=0;
	forwardDist=0;
	reverseDist=0; 
}

// Clears one particular counter
void clearOneCounter(int which)
{
	clearCounters();
	/*switch(which)
	  {
	  case 0:
	  clearCounters();
	  break;

	  case 1:
	  leftTicks=0;
	  break;

	  case 2:
	  rightTicks=0;
	  break;

	  case 3:
	  leftRevs=0;
	  break;

	  case 4:
	  rightRevs=0;
	  break;

	  case 5:
	  forwardDist=0;
	  break;

	  case 6:
	  reverseDist=0;
	  break;
	  } */
}
// Intialize Vincet's internal states

void initializeState()
{
	clearCounters();
}

void handleCommand(TPacket *command)
{
	switch(command->command)
	{
		// For movement commands, param[0] = distance, param[1] = speed.
		case COMMAND_FORWARD:
			sendOK();
			forward((float) command->params[0], (float) command->params[1]);
			break;
		case COMMAND_REVERSE:
			sendOK();
			reverse((float) command->params[0], (float) command->params[1]);
			break;
		case COMMAND_TURN_LEFT:
			sendOK();
			left((float) command->params[0], (float) command->params[1]);
			break;
		case COMMAND_TURN_RIGHT:
			sendOK();
			right((float) command->params[0], (float) command->params[1]);
			break;
		case COMMAND_STOP:
			sendOK();
			stop();
			break;
		case COMMAND_GET_STATS:
			sendStatus();
			break;
		case COMMAND_CLEAR_STATS:
			clearOneCounter(command->params[0]);
			sendOK();
			break;
		case COMMAND_GET_COLOR:
			readColorValues();
			break;

			/*
			 * Implement code for other commands here.
			 * 
			 */

		default:
			sendBadCommand();
	}
}

void waitForHello()
{
	int exit=0;

	while(!exit)
	{
		TPacket hello;
		TResult result;

		do
		{
			result = readPacket(&hello);
		} while (result == PACKET_INCOMPLETE);

		if(result == PACKET_OK)
		{
			if(hello.packetType == PACKET_TYPE_HELLO)
			{


				sendOK();
				exit=1;
			}
			else
				sendBadResponse();
		}
		else
			if(result == PACKET_BAD)
			{
				sendBadPacket();
			}
			else
				if(result == PACKET_CHECKSUM_BAD)
					sendBadChecksum();
	} // !exit
}


void setup() {
	// put your setup code here, to run once:
	alexDiagonal = sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH * ALEX_BREADTH));
	alexCirc = PI * alexDiagonal;
	cli();
	setupEINT();
	setupSerial();
	startSerial();
	setupMotors();
	startMotors();
  setupColorSensor();
	enablePullups();
	initializeState();
	sei();
}

void handlePacket(TPacket *packet)
{
	switch(packet->packetType)
	{
		case PACKET_TYPE_COMMAND:
			handleCommand(packet);
			break;

		case PACKET_TYPE_RESPONSE:
			break;

		case PACKET_TYPE_ERROR:
			break;

		case PACKET_TYPE_MESSAGE:
			break;

		case PACKET_TYPE_HELLO:
			break;
	}
}

void loop() {

	// Uncomment the code below for Step 2 of Activity 3 in Week 8 Studio 2

	//forward(0, 100);

	// Uncomment the code below for Week 9 Studio 2


	// put your main code here, to run repeatedly:

  	//readColorValues();
  
	TPacket recvPacket; // This holds commands from the Pi

	TResult result = readPacket(&recvPacket);

	if(result == PACKET_OK)
		handlePacket(&recvPacket);
	else
		if(result == PACKET_BAD)
		{
			sendBadPacket();
		}
		else
			if(result == PACKET_CHECKSUM_BAD)
			{
				sendBadChecksum();
			} 

	if(deltaDist > 0)
	{
		if(dir==FORWARD)
		{
			if(forwardDist > newDist)
			{
				deltaDist=0;
				newDist=0;
				stop();
			}
		}
		else
			if(dir == BACKWARD)
			{
				if(reverseDist > newDist)
				{
					deltaDist=0;
					newDist=0;
					stop();
				}
			}
			else
				if(dir == STOP)
				{
					deltaDist=0;
					newDist=0;
					stop();
				}
	}
	if(leftDeltaTicks > 0 || rightDeltaTicks > 0)
	{
		if(dir == LEFT)
		{
			if(leftReverseTicksTurns >= leftTargetTicks || rightForwardTicksTurns >= rightTargetTicks)
			{
				leftDeltaTicks = 0;
				rightDeltaTicks = 0;
				leftTargetTicks = 0;
				rightTargetTicks = 0;
				stop();
			}
		}
		else if(dir == RIGHT)
		{
			if(rightReverseTicksTurns >= rightTargetTicks || leftForwardTicksTurns >= leftTargetTicks)
			{
				leftDeltaTicks = 0;
				rightDeltaTicks = 0;
				leftTargetTicks = 0;
				rightTargetTicks = 0;
				stop();
			}
		}
		else if(dir == STOP)
		{			
				leftDeltaTicks = 0;
				rightDeltaTicks = 0;
				leftTargetTicks = 0;
				rightTargetTicks = 0;
				stop();
		}
	}
}
