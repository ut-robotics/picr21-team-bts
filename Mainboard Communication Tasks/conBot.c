#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <fcntl.h> 
#include <termios.h>
#include <unistd.h>


#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <pthread.h>
#include <ncurses.h>

typedef struct __attribute__((__packed__)) Command { // (1)
  int16_t speed1;
  int16_t speed2;
  int16_t speed3;
  uint16_t throwerSpeed;
  char zeroChar;
  uint16_t delimiter; // (2)
} Command;

typedef struct Feedback { // (3)
  int16_t speed1;
  int16_t speed2;
  int16_t speed3;
  uint16_t delimiter;
} Feedback;


int set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        if (tcgetattr (fd, &tty) != 0)
        {
                printf ("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                printf ("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}

void set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf ("error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                printf ("error %d setting term attributes", errno);
}
Command command = {
	.speed1 = 0, 
	.speed2 = 0, .speed3 = 0, 
	.throwerSpeed = 0, 
	.zeroChar = 0,
	.delimiter = 0xAAAA
	};
Feedback feedback = {
      .speed1 = 0,
      .speed2 = 0,
      .speed3 = 0,
      .delimiter = 0xAAAA
    };

void* moveSend(void *arg)  //forever loop for geting pressed keys
{
	keypad(stdscr, TRUE);	
	noecho();
	int ch;
	while(1)
	{
		initscr();
		timeout(-1); //forever waiting for input
		ch = getch();
		endwin();
		move(4,0);
		clrtoeol();
		switch(ch){
			case KEY_UP:
			case 'w':
				mvprintw(4,0,"Forward");
				command.speed1 = 5;
				command.speed2 = -5;
				command.speed3 = 0;
				break;
			case KEY_RIGHT:
			case 'd':
				mvprintw(4,0,"Right");
				command.speed1 = -5;
				command.speed2 = 0;
				command.speed3 = 5;
				break;
			case KEY_LEFT:
			case 'a':
				mvprintw(4,0,"Left");
				command.speed1 = 0;
				command.speed2 = 5;
				command.speed3 = -5;
				break;
			case KEY_DOWN:
			case 's':
				mvprintw(4,0,"Down");
				command.speed1 = -5;
				command.speed2 = 5;
				command.speed3 = 0;
				break;
			case 'q':
				mvprintw(4,0,"Rotate left");
				command.speed1 = 3;
				command.speed2 = 3;
				command.speed3 = 3;
				break;
			case 'e':
				mvprintw(4,0,"Rotate right");
				command.speed1 = -3;
				command.speed2 = -3;
				command.speed3 = -3;
				break;
			default:
				mvprintw(4,0,"Tiemout no action");
				command.speed1 = 0;
				command.speed2 = 0;
				command.speed3 = 0;
		 }
		refresh();
	}
	keypad(stdscr, FALSE);
}


int main(int argc, char *argv[])
{
	pthread_t moveThread;
	char *portname = "/dev/ttyACM1";
	int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0)
	{
			printf ("error %d opening %s: %s", errno, portname, strerror (errno));
			return 0;
	}

	set_interface_attribs (fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
	set_blocking (fd, 0);                // set no blocking
	int rdlen, wlen;
	if(pthread_create(&moveThread, NULL, moveSend, NULL) != 0) { //Thread to input
		printf("Failed to create thread\n");
	}
	do {
		mvprintw(0,0,"Data to send s1 %d, s2 %d, s3 %d: \n", command.speed1,command.speed2, command.speed3);	
		wlen = write(fd, &command, sizeof(command)); //Sending signal
		usleep ((12 + 25) * 100); 
		if (wlen != sizeof(command)) {
			mvprintw(1,0,"Error from write: %d, %d\n", wlen, errno);
		}		
		rdlen = read(fd, &feedback, sizeof(feedback)); //Reading response
		if (rdlen > 0) {
            mvprintw(1,0,"Read %d:\n", rdlen);
            mvprintw(2,0,"Data s1 %d, s2 %d, s3 %d:\n", feedback.speed1,feedback.speed2, feedback.speed3);
			//TO RESET SPEED TO 0 after command
			command.speed1 = 0;
			command.speed2 = 0;
			command.speed3 = 0;
        } else if (rdlen < 0) {
             mvprintw(1,0,"Error from read: %d: %s\n", rdlen, strerror(errno));	
        } else {  // rdlen == 0 
             mvprintw(1,0,"Timeout from read\n");
        }  				
		usleep(100000);
    } while (1);
	pthread_cancel(moveThread);
	return 0;
}
