// library for keyboard contol
#include <ncurses.h>
// Routines to create a TLS client
#include "make_tls_client.h"
// Network packet types
#include "netconstants.h"
// Packet types, error codes, etc.
#include "constants.h"
#include <string.h>

// Tells us that the network is running.
static volatile int networkActive=0;

void handleError(const char *buffer)
{
	switch(buffer[1])
	{
		case RESP_OK:
			printw("Command / Status OK\n");
			break;

		case RESP_BAD_PACKET:
			printw("BAD MAGIC NUMBER FROM ARDUINO\n");
			break;

		case RESP_BAD_CHECKSUM:
			printw("BAD CHECKSUM FROM ARDUINO\n");
			break;

		case RESP_BAD_COMMAND:
			printw("PI SENT BAD COMMAND TO ARDUINO\n");
			break;

		case RESP_BAD_RESPONSE:
			printw("PI GOT BAD RESPONSE FROM ARDUINO\n");
			break;

		default:
			printw("PI IS CONFUSED!\n");
	}
}

void handleStatus(const char *buffer)
{
	int32_t data[16];
	memcpy(data, &buffer[1], sizeof(data));
	printw("\n ------- ALEX STATUS REPORT ------- \n\n");
	printw("Left Forward Ticks:\t\t%d\n", data[0]);
	printw("Right Forward Ticks:\t\t%d\n", data[1]);
	printw("Left Reverse Ticks:\t\t%d\n", data[2]);
	printw("Right Reverse Ticks:\t\t%d\n", data[3]);
	printw("Left Forward Ticks Turns:\t%d\n", data[4]);
	printw("Right Forward Ticks Turns:\t%d\n", data[5]);
	printw("Left Reverse Ticks Turns:\t%d\n", data[6]);
	printw("Right Reverse Ticks Turns:\t%d\n", data[7]);
	printw("Forward Distance:\t\t%d\n", data[8]);
	printw("Reverse Distance:\t\t%d\n", data[9]);
	printw("\n---------------------------------------\n\n");
}

//print out color value readings from Alex
void handleColor(const char *buffer)
{
	int32_t data[16];
	memcpy(data, &buffer[1], sizeof(data));
	printw("\n ---------------- ALEX COLOR REPORT ----------------------- \n\n");
	printw("R:\t\t%d\n",data[0]);
	printw("G:\t\t%d\n",data[1]);
	printw("B:\t\t%d\n",data[2]);
	if(data[3] == 1)
	{
		printw("DETECTED COLOR IS: RED\n");
	}else if(data[3] == 0)
	{
		printw("DETECTED COLOR IS: GREEN\n");
	}
	printw("\n---------------------------------------\n\n");
}

//print out ultrasonic readings from Alex
void handleUltrasonic (const char *buffer) 
{
	int32_t data[16];
	memcpy(data, &buffer[1], sizeof(data));
	printw("\n ---------------- ALEX ULTRASONIC DISTANCE  REPORT ----------------------- \n\n");
	printw("FRONT DISTANCE:\t\t%d\n",data[0]);
	printw("LEFT DISTANCE:\t\t%d\n",data[1]);
	printw("RIGHT DISTANCE:\t\t%d\n",data[2]);
	printw("\n---------------------------------------\n\n");
}

void handleMessage(const char *buffer)
{
	printw("MESSAGE FROM ALEX: %s\n", &buffer[1]);
}

void handleCommand(const char *buffer)
{
	// We don't do anything because we issue commands
	// but we don't get them. Put this here
	// for future expansion
}

void handleNetwork(const char *buffer, int len)
{
	// The first byte is the packet type
	int type = buffer[0];
	switch(type)
	{
		case NET_ERROR_PACKET:
			handleError(buffer);
			break;

		case NET_STATUS_PACKET:
			handleStatus(buffer);
			break;

		case NET_MESSAGE_PACKET:
			handleMessage(buffer);
			break;

		case NET_COMMAND_PACKET:
			handleCommand(buffer);
			break;

		case NET_COLOR_PACKET:
			handleColor(buffer);
			break;

		case NET_ULTRASONIC_PACKET:
			handleUltrasonic(buffer);
			break;
	}
}

void sendData(void *conn, const char *buffer, int len)
{
	int c;
	printw("\nSENDING %d BYTES DATA\n\n", len);
	if(networkActive)
	{
		//SSL write here to write buffer to network
		sslWrite(conn, buffer, len);
		networkActive = (c > 0);
	}
}

void *readerThread(void *conn)
{
	char buffer[128];
	int len;
	while(networkActive)
	{
		// SSL read here into buffer
		len = sslRead(conn, buffer, sizeof(buffer));
		printw("read %d bytes from server.\n", len);
		networkActive = (len > 0);
		if(networkActive)
			handleNetwork(buffer, len);
	}

	printw("Exiting network listener thread\n");
	//Stop the client loop and call EXIT_THREAD
	stopClient();
	EXIT_THREAD(conn);
}

//flush input buffer to prevent reading garbage
void flushInput()
{
	char c;
	while((c = getchar()) != '\n' && c != EOF);
}

void getParams(int32_t *params)
{
	printw("\nEnter distance/angle in cm/degrees (e.g. 50) and power in %% (e.g. 75) separated by space.\n");
	printw("E.g. 50 75 means go at 50 cm at 75%% power for forward/backward, or 50 degrees left or right turn at 75%%  power\n");
	//flush input before scanning for input
	flushinp();
	scanw("%d %d", &params[0], &params[1]);
}

void *writerThread(void *conn)
{
	int quit=0;
	//initialise ncurses
	initscr();
	cbreak();
	//enable arrowkeypad input
	keypad(stdscr, TRUE);

	while(!quit)
	{
		//clear screen if screen is out of space
		if(getcury(stdscr) >= LINES -1)
		{
			clear();
			move(0,0);
		}
		int ch;
		printw("Command (f=forward, b=reverse, l=turn left, r=turn right, s=stop, c=clear stats, g=get stats, x=get color, q=exit,o=clear screen, up/back arrow key to make robot move 5cm at 70%% power,left/right arrow keys to make robot move 10 degrees at 65%% power, p=rick roll)\n");
		ch = getch();
		char buffer[10];
		int32_t params[2];
		buffer[0] = NET_COMMAND_PACKET;
		switch(ch)
		{
			case 'f':
			case 'F':
			case 'b':
			case 'B':
			case 'l':
			case 'L':
			case 'r':
			case 'R':
				getParams(params);
				buffer[1] = (char)ch;
				memcpy(&buffer[2], params, sizeof(params));
				sendData(conn, buffer, sizeof(buffer));
				break;
			case 's':
			case 'S':
			case 'c':
			case 'C':
			case 'g':
			case 'G':
				params[0]=0;
				params[1]=0;
				memcpy(&buffer[2], params, sizeof(params));
				buffer[1] = (char)ch;
				sendData(conn, buffer, sizeof(buffer));
				break;
			case 'x':
			case 'X':
				params[0] = 0;
				params[1] = 0;
				memcpy(&buffer[2],params,sizeof(params));
				buffer[1] = (char)ch;
				sendData(conn, buffer, sizeof(buffer));
				break;
			case KEY_UP:
				params[0] = 2;
				params[1] = 100;
				memcpy(&buffer[2], params, sizeof(params));
				buffer[1] = 'f';
				sendData(conn, buffer, sizeof(buffer));
				printw("up arrow key has been pressed\n");
				break;
			case KEY_DOWN:
				params[0] = 2;
				params[1] = 100;
				memcpy(&buffer[2], params, sizeof(params));
				buffer[1] = 'b';
				sendData(conn, buffer, sizeof(buffer));
				printw("down arrow key has been pressed\n");
				break;
			case KEY_LEFT:
				params[0] = 10;
				params[1] = 90;
				memcpy(&buffer[2], params, sizeof(params));
				buffer[1] = 'l';
				sendData(conn, buffer, sizeof(buffer));
				printw("left arrow key has been pressed\n");
				break;
			case KEY_RIGHT:
				params[0] = 10;
				params[1] = 90;
				memcpy(&buffer[2], params, sizeof(params));
				buffer[1] = 'r';
				sendData(conn, buffer, sizeof(buffer));
				printw("right arrow key has been pressed\n");
				break;
			case 'u':
			case 'U':
				params[0] = 0;
				params[1] = 0;
				memcpy(&buffer[2],params,sizeof(params));
				buffer[1] = ch;
				sendData(conn, buffer, sizeof(buffer));
				break;
			case 'p':
			case 'P':
				printw("Enter the following code to rick roll your enemies - 'rickroll'\n");
				char* input = new char[10];
				getnstr(input, 10);
				if(strcmp(input, "rickroll") == 0) 
				{
					params[0] = 0;
					params[1]= 0;
					memcpy(&buffer[2],params,sizeof(params));
					buffer[1] = (char)ch;
					sendData(conn, buffer, sizeof(buffer));
				}
				delete input;
				break;
			case 'o':
			case 'O':
				clear();
				break;
			case 'q':
			case 'Q':
				quit=1;
				break;
			default:
				printw("BAD COMMAND\n");
		}
	}

	printw("Exiting keyboard thread\n");
	endwin();
	//Stop the client loop and call EXIT_THREAD
	stopClient();
	EXIT_THREAD(conn);
}

//define filenames for the client private key, certificates,
//   CA filename, etc. that you need to create a client
#define CLIENT_PRIVATE_KEY "laptop.key"
#define CLIENT_CERTIFICATE "laptop.crt"
#define CA_CERTIFICATE "signing.pem"
#define SERVER_NAME_ON_CERT "alex.epp.com"

void connectToServer(const char *serverName, int portNum)
{
	// Create a new client
	createClient(serverName, portNum, 1, CA_CERTIFICATE, SERVER_NAME_ON_CERT, 1, CLIENT_CERTIFICATE, CLIENT_PRIVATE_KEY, readerThread, writerThread);
}

int main(int ac, char **av)
{
	if(ac != 3)
	{
		fprintf(stderr, "\n\n%s <IP address> <Port Number>\n\n", av[0]);
		exit(-1);
	}
	networkActive = 1;
	connectToServer(av[1], atoi(av[2]));
	// Add in while loop to prevent main from exiting while the
	//   client loop is running 
	while(client_is_running());
	printf("\nMAIN exiting\n\n");
}
