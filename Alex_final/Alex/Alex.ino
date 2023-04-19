#include "buffer.h"
#include <serialize.h>
#include <stdarg.h>
#include "packet.h"
#include "constants.h"
#include <math.h>

typedef enum
{
  STOP = 0,
  FORWARD = 1,
  BACKWARD = 2,
  LEFT = 3,
  RIGHT = 4
} TDirection;
volatile TDirection dir = STOP;

/*
   Alex's configuration constants
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
#define COLOR_S0 0b00000100 //Analog pin A2
#define COLOR_S1 0b00001000 //Analog pin A3
//port B
#define COLOR_S2 0b00000001 //pin 8
#define COLOR_S3 0b00010000 //pin 12
#define COLOR_OUTPUT 0b00100000 //pin 13

//Ultrasonic sensor pins
#define TRIG_PIN_FRONT          0b00000001 //Analog pin A0
#define TRIG_PIN_LEFT           0b00000010 //Analog pin A1
#define TRIG_PIN_RIGHT          0b00010000 //Analog pin A4

#define ECHO_PIN_FRONT                0b00000010 //Digital pin 9
#define ECHO_ORIG_FRONT               9


#define ECHO_PIN_LEFT                0b00010000 //Digital pin 4
#define ECHO_ORIG_LEFT               4

#define ECHO_PIN_RIGHT               0b10000000 //Digital pin 7
#define ECHO_ORIG_RIGHT              7

//Ultrasonic sensor variables
long duration_front;
int distance_front;
long duration_left;
int distance_left;
long duration_right;
int distance_right;

//Colour Frequency output`
int frequency = 0;

//Buzzer pin and musical note frequency definitions
void rickroll();
#define  NOTE_A3FLAT    208                    // 208 Hz
#define  NOTE_B3FLAT    233                   // 233 Hz
#define  NOTE_B3     247                     // 247 Hz
#define  NOTE_C4     261                    // 261 Hz MIDDLE C
#define  NOTE_C4SHARP    277               // 277 Hz
#define  NOTE_E4FLAT    311               // 311 Hz    
#define  NOTE_F4     349                 // 349 Hz 
#define  NOTE_A4FLAT    415             // 415 Hz  
#define  NOTE_B4FLAT    466            // 466 Hz 
#define  NOTE_B4     493              // 493 Hz 
#define  NOTE_C5     523             // 523 Hz 
#define  NOTE_C5SHARP    554        // 554 Hz
#define  NOTE_E5FLAT    622        // 622 Hz  
#define  NOTE_F5     698          // 698 Hz 
#define  NOTE_F5SHARP    740     // 740 Hz
#define  NOTE_A5FLAT    831     // 831 Hz 

#define GAP    -1
#define BUZZER_PIN  A5 // Connect your BUZZER_PIN buzzer to this pin or change it to match your circuit!
#define BUZZER_PIN_BARE 0b00100000

volatile int beatlength = 100; // determines tempo
float beatseparationconstant = 0.3;
int threshold;
int note_index; // note index
int song1_chorus_melody[] =
{ NOTE_B4FLAT, NOTE_B4FLAT, NOTE_A4FLAT, NOTE_A4FLAT,
  NOTE_F5, NOTE_F5, NOTE_E5FLAT, NOTE_B4FLAT, NOTE_B4FLAT, NOTE_A4FLAT, NOTE_A4FLAT, NOTE_E5FLAT, NOTE_E5FLAT, NOTE_C5SHARP, NOTE_C5, NOTE_B4FLAT,
  NOTE_C5SHARP, NOTE_C5SHARP, NOTE_C5SHARP, NOTE_C5SHARP,
  NOTE_C5SHARP, NOTE_E5FLAT, NOTE_C5, NOTE_B4FLAT, NOTE_A4FLAT, NOTE_A4FLAT, NOTE_A4FLAT, NOTE_E5FLAT, NOTE_C5SHARP,
  NOTE_B4FLAT, NOTE_B4FLAT, NOTE_A4FLAT, NOTE_A4FLAT,
  NOTE_F5, NOTE_F5, NOTE_E5FLAT, NOTE_B4FLAT, NOTE_B4FLAT, NOTE_A4FLAT, NOTE_A4FLAT, NOTE_A5FLAT, NOTE_C5, NOTE_C5SHARP, NOTE_C5, NOTE_B4FLAT,
  NOTE_C5SHARP, NOTE_C5SHARP, NOTE_C5SHARP, NOTE_C5SHARP,
  NOTE_C5SHARP, NOTE_E5FLAT, NOTE_C5, NOTE_B4FLAT, NOTE_A4FLAT, GAP, NOTE_A4FLAT, NOTE_E5FLAT, NOTE_C5SHARP, GAP
};
int song1_chorus_rhythmn[] =
{ 1, 1, 1, 1,
  3, 3, 6, 1, 1, 1, 1, 3, 3, 3, 1, 2,
  1, 1, 1, 1,
  3, 3, 3, 1, 2, 2, 2, 4, 8,
  1, 1, 1, 1,
  3, 3, 6, 1, 1, 1, 1, 3, 3, 3, 1, 2,
  1, 1, 1, 1,
  3, 3, 3, 1, 2, 2, 2, 4, 8, 4
};

/*
      Alex's State Variables
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
TBuffer _recvBuffer;
TBuffer _xmitBuffer;

/*
   Alex Communication Routines.
*/
TResult readPacket(TPacket *packet)
{
  // Reads in data from the serial port and
  // deserializes it.Returns deserialized
  // data in "packet".
  char buffer[PACKET_SIZE];
  int len;
  len = readSerial(buffer);
  if (len == 0)
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
  messagePacket.packetType = PACKET_TYPE_MESSAGE;
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
  badCommand.packetType = PACKET_TYPE_ERROR;
  badCommand.command = RESP_BAD_COMMAND;
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
   Setup and start codes for external interrupts and
   pullup resistors.
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
  switch (dir)
  {
    case (FORWARD):
      leftForwardTicks++;
      break;
    case (BACKWARD):
      leftReverseTicks++;
      break;
    case (LEFT):
      leftReverseTicksTurns++;
      break;
    case (RIGHT):
      leftForwardTicksTurns++;
      break;
  }
  if (dir == FORWARD)
  {
    forwardDist = (unsigned long) ((float) leftForwardTicks / LEFT_COUNTS_PER_REV * WHEEL_CIRC);
  }
  if (dir == BACKWARD)
  {
    reverseDist = (unsigned long) ((float) leftReverseTicks / LEFT_COUNTS_PER_REV * WHEEL_CIRC);
  }
}

void rightISR()
{
  switch (dir)
  {
    case (FORWARD):
      rightForwardTicks++;
      break;
    case (BACKWARD):
      rightReverseTicks++;
      break;
    case (LEFT):
      rightForwardTicksTurns++;
      break;
    case (RIGHT):
      rightReverseTicksTurns++;
      break;
  }
}

// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered. Use bare-metal.
void setupEINT()
{
  // Use bare-metal to configure pins 2 and 3 to be
  // falling edge triggered. Enabled
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

/*
   Setup and start codes for serial communications

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
  //if there is data in buffer, transfer to UDR0
  //else disable udre interrupt after done transmitting buffer
  if (result == BUFFER_OK)
  {
    UDR0 = data;
  } else if (result == BUFFER_EMPTY)
  {
    UCSR0B &= 0b11011111;
  }
}

void setupSerial()
{
  //set baud rate to 9600 and initialize our transmit and receive buffers
  initBuffer(&_recvBuffer, BUFFER_LEN);
  initBuffer(&_xmitBuffer, BUFFER_LEN);
  UBRR0L = 103;
  UBRR0H = 0;
  //asynchronous uart, no parity, 1 stop bit, 8 bit data size per packet
  UCSR0C = 0b00000110;
  //0 the ucsr0a register first
  UCSR0A = 0;
}

// Start the serial connection.
void startSerial()
{
  //enable interrupts for uarts and enabling transmitters and receivers
  UCSR0B = 0b10111000;
}

// Read the serial port. 
int readSerial(char *buffer)
{
  //check if there is data in the receive buffer, continue to read from it until empty
  int count = 0;
  TBufferResult result = BUFFER_OK;
  for (count = 0; dataAvailable(&_recvBuffer) && result == BUFFER_OK; count += 1)
  {
    result = readBuffer(&_recvBuffer, (unsigned char*)&buffer[count]);
  }
  return count;
}

// Write to the serial port.
void writeSerial(const char *buffer, int len)
{
  //write to the transmit buffer which will copy the array contents one by 1 to UDR0 to be pushed out in the interrupt function
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
   Alex's motor drivers.

*/
// Set up Alex's motors. Right now this is empty, but
// later you will replace it with code to set up the PWMs
// to drive the motors.
void setupMotors()
{
  /* Our motor set up is:
        A1IN - Pin 5, PD5, OC0B
        A2IN - Pin 6, PD6, OC0A
        B1IN - Pin 10, PB2, OC1B
        B2In - pIN 11, PB3, OC2A
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
  if (speed < 0.0)
    speed = 0;

  if (speed > 100.0)
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
  if (ang == 0)
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
  if (ang == 0)
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
}

// Stop Alex. To replace with bare-metal code later.
void stop()
{
  dir = STOP;
  LEFT_FORWARD_REGISTER = 0;
  RIGHT_FORWARD_REGISTER = 0;
  LEFT_REVERSE_REGISTER = 0;
  RIGHT_REVERSE_REGISTER = 0;
}

void setupColorSensor()
{
  //Set S0, S1, S2 AND S3 to be OUTPUT while output frequency is set to INPUT
  DDRC |= ((COLOR_S0) | (COLOR_S1));
  DDRB |= ((COLOR_S2) | (COLOR_S3));
  DDRB &= ~(COLOR_OUTPUT);
  //Setting frequency scaling to 20% by setting S0 to HIGH and S1 to LOW
  PORTC |= COLOR_S0;
  PORTC &= ~(COLOR_S1);
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
  delay(100);
  //Setting Green filtered photodiodes to be read
  PORTB |= ((COLOR_S2) | (COLOR_S3));
  //Reading output frequency
  frequency = pulseIn(13, LOW);
  colorPacket.params[1] = frequency;
  delay(100);
  //Setting BLUE filtered photodiodes to be read
  PORTB &= ~(COLOR_S2);
  PORTB |= (COLOR_S3);
  //Reading output frequency
  frequency = pulseIn(13, LOW);
  colorPacket.params[2] = frequency;
  delay(100);
  //determine if color detected is red or green: 1 for red, 0 for green
  // if red, play the alert tune(rickroll)
  if (colorPacket.params[0] < colorPacket.params[1] && colorPacket.params[2] > 1000)
  {
    colorPacket.params[3] = 1;
    rickroll();
  } else if(colorPacket.params[0] > colorPacket.params[1] && colorPacket.params[2] > 1000)
  {
    colorPacket.params[3] = 0;
  } else
  {
    colorPacket.params[3] = 2;
  }
  
  //send response packet containing readings to raspberry pi
  sendResponse(&colorPacket);
}

void setupUltrasonic()
{
  DDRC |= (TRIG_PIN_FRONT | TRIG_PIN_LEFT | TRIG_PIN_RIGHT);
  DDRB &= ~(ECHO_PIN_FRONT);
  DDRD &= ~(ECHO_PIN_LEFT | ECHO_PIN_RIGHT);
}

void getUltrasonic()
{
  TPacket ultrasonicPacket;
  ultrasonicPacket.packetType = PACKET_TYPE_RESPONSE;
  ultrasonicPacket.command = RESP_ULTRASONIC;
  
  //SETTING UP FRONT ULTRASONIC SENSOR
  PORTC &= ~(TRIG_PIN_FRONT);
  delayMicroseconds(2);
  //Sets the TRIG_PIN_FRONT on HIGH state for 10 micro seconds
  PORTC |= TRIG_PIN_FRONT;
  delayMicroseconds(10);
  PORTC &= ~(TRIG_PIN_FRONT);
  //Reads the ECHO_PIN_FRONT, returns the sound wave travel time in microseconds
  duration_front = pulseIn(ECHO_ORIG_FRONT, HIGH);
  //Calculating the distance using forumla duration*speed(in cm)/2 since distance is 2 way
  distance_front = duration_front * 0.034 / 2;
  ultrasonicPacket.params[0] = distance_front;
  
  //SETTING UP LEFT ULTRASONIC SENSOR
  PORTC &= ~(TRIG_PIN_LEFT);
  delayMicroseconds(2);
  //Sets the TRIG_PIN_FRONT on HIGH state for 10 micro seconds
  PORTC |= TRIG_PIN_LEFT;
  delayMicroseconds(10);
  PORTC &= ~(TRIG_PIN_LEFT);
  //Reads the ECHO_PIN_FRONT, returns the sound wave travel time in microseconds
  duration_left = pulseIn(ECHO_ORIG_LEFT, HIGH);
  //Calculating the distance
  distance_left = duration_left * 0.034 / 2;
  ultrasonicPacket.params[1] = distance_left;
  
  //SETTING UP RIGHT ULTRASONIC SENSOR
  PORTC &= ~(TRIG_PIN_RIGHT);
  //digitalWrite(TRIG_PIN_RIGHT, LOW);
  delayMicroseconds(2);
  //Sets the TRIG_PIN_FRONT on HIGH state for 10 micro seconds
  PORTC |= TRIG_PIN_RIGHT;
  delayMicroseconds(10);
  PORTC &= ~(TRIG_PIN_RIGHT);
  //Reads the ECHO_PIN_FRONT, returns the sound wave travel time in microseconds
  duration_right = pulseIn(ECHO_ORIG_RIGHT, HIGH);
  //Calculating the distance
  distance_right = duration_right * 0.034 / 2;
  ultrasonicPacket.params[2] = distance_right;
 
  //send a response packet containing readings to raspberry pi
  sendResponse(&ultrasonicPacket);
}

//Buzzer setup and run codes

void setupBuzzer()
{
  DDRC |= BUZZER_PIN_BARE;

  note_index = 0;
}

//Function to replace tone() on the Arduino library to prevent PWM pins from being affected
//simulate a 50%pwm cycle with different frequencies
void sound(byte pin, uint16_t frequency, uint16_t duration)
{
  unsigned long startTime = millis();
  unsigned long halfPeriod = 1000000L / frequency / 2;
  DDRC |= BUZZER_PIN_BARE;
  while (millis() - startTime < duration)
  {
    PORTC |= BUZZER_PIN_BARE;
    delayMicroseconds(halfPeriod);
    PORTC &= ~(BUZZER_PIN_BARE);
    delayMicroseconds(halfPeriod);
  }
  DDRC &= ~(BUZZER_PIN_BARE);
}

void rickroll()
{
  for (int i = 0; i < 59; i++)
  {
    int notelength;
    // chorus
    notelength = beatlength * song1_chorus_rhythmn[note_index];
    if (song1_chorus_melody[note_index] > 0) 
    {
      sound(BUZZER_PIN, song1_chorus_melody[note_index], notelength/6);
    }
    note_index++;
    if (note_index >= sizeof(song1_chorus_melody) / sizeof(int)) 
    {
      note_index = 0;
    }
    delay(notelength/6);
  }
}

/*
   Alex's setup and run codes

*/
// Clears all our counters
void clearCounters()
{
  leftForwardTicks = 0;
  rightForwardTicks = 0;
  leftReverseTicks = 0;
  rightReverseTicks = 0;
  leftForwardTicksTurns = 0;
  rightForwardTicksTurns = 0;
  leftReverseTicksTurns = 0;
  rightReverseTicksTurns = 0;
  leftRevs = 0;
  rightRevs = 0;
  forwardDist = 0;
  reverseDist = 0;
}

// Clears all counters
void clearOneCounter(int which)
{
  clearCounters();
}

// Intialize Vincet's internal states
void initializeState()
{
  clearCounters();
}

void handleCommand(TPacket *command)
{
  switch (command->command)
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
    case COMMAND_GET_ULTRASONIC:
      getUltrasonic();
      break;
    case COMMAND_RICK_ROLL:
      sendOK();
      rickroll();
      break;
    default:
      sendBadCommand();
  }
}

void waitForHello()
{
  int exit = 0;
  while (!exit)
  {
    TPacket hello;
    TResult result;
    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);
    if (result == PACKET_OK)
    {
      if (hello.packetType == PACKET_TYPE_HELLO)
      {
        sendOK();
        exit = 1;
      }
      else
        sendBadResponse();
    }
    else if (result == PACKET_BAD)
    {
      sendBadPacket();
    }
    else if (result == PACKET_CHECKSUM_BAD)
      sendBadChecksum();
  } // !exit
}


void setup() 
{
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
  setupUltrasonic();
  setupBuzzer();
  enablePullups();
  initializeState();
  sei();
}

void handlePacket(TPacket *packet)
{
  switch (packet->packetType)
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
  TPacket recvPacket; // This holds commands from the Pi
  TResult result = readPacket(&recvPacket);
  if (result == PACKET_OK)
    handlePacket(&recvPacket);
  else if (result == PACKET_BAD)
  {
    sendBadPacket();
  }
  else if (result == PACKET_CHECKSUM_BAD)
  {
    sendBadChecksum();
  }
  
  //moving forward/backward conditions for arduino. it will continue moving in specified direction until hall sensors ticks reaches a certain amount
  if (deltaDist > 0)
  {
    if (dir == FORWARD)
    {
      if (forwardDist > newDist)
      {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    }
    else if (dir == BACKWARD)
    {
      if (reverseDist > newDist)
      {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    }
    else if (dir == STOP)
    {
      deltaDist = 0;
      newDist = 0;
      stop();
    }
  }
  
  //turning conditions for the arduino. it will continue to turn left or right until the hall sensor ticks reaches a certain amount
  if (leftDeltaTicks > 0 || rightDeltaTicks > 0)
  {
    if (dir == LEFT)
    {
      if (leftReverseTicksTurns >= leftTargetTicks || rightForwardTicksTurns >= rightTargetTicks)
      {
        leftDeltaTicks = 0;
        rightDeltaTicks = 0;
        leftTargetTicks = 0;
        rightTargetTicks = 0;
        stop();
      }
    }
    else if (dir == RIGHT)
    {
      if (rightReverseTicksTurns >= rightTargetTicks || leftForwardTicksTurns >= leftTargetTicks)
      {
        leftDeltaTicks = 0;
        rightDeltaTicks = 0;
        leftTargetTicks = 0;
        rightTargetTicks = 0;
        stop();
      }
    }
    else if (dir == STOP)
    {
      leftDeltaTicks = 0;
      rightDeltaTicks = 0;
      leftTargetTicks = 0;
      rightTargetTicks = 0;
      stop();
    }
  }
}
