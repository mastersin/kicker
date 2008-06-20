       #include <sys/types.h>
       #include <sys/stat.h>
       #include <fcntl.h>
       #include <termios.h>
       #include <stdio.h>
       #include <strings.h>

       #define BAUDRATE B38400
       #define MODEMDEVICE "/dev/ttyUSB1"
//       #define MODEMDEVICE "/dev/ttyS0"
       #define _POSIX_SOURCE 1 /* POSIX compliant source */
       #define FALSE 0
       #define TRUE 1

       volatile int STOP=FALSE;
	
       unsigned char to_char(unsigned char byte) {
    	    byte &= 0xf;
	    printf("CS:%2.2x", byte);
    	    if (byte > 9)
		    return byte + 'A' - 0xA;
	    return byte + '0';
       }

       int main(int argc, char *argv[])
       {
         int fd,c, res,i,j,x = 0;
	 int state = 0;
         struct termios oldtio,newtio;
         unsigned char buf[255];
         unsigned char detect[255];

        fd = open(MODEMDEVICE, O_RDWR | O_NOCTTY );
        if (fd <0) {perror(MODEMDEVICE); return(-1); }

        tcgetattr(fd,&oldtio); /* save current port settings */

        bzero(&newtio, sizeof(newtio));
        //newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
        //newtio.c_iflag = IGNPAR;
        //newtio.c_oflag = 0;
        ///* set input mode (non-canonical, no echo,...) */
        //newtio.c_lflag = 0;

	//newtio = oldtio;
        //newtio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP
        //                   | INLCR | IGNCR | ICRNL | IXON);
        //newtio.c_oflag &= ~OPOST;
        //newtio.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
        //newtio.c_cflag &= ~(CSIZE | PARENB);
        //newtio.c_cflag |= CS8;

        //newtio.c_cflag = BAUDRATE | CS8 | CREAD | CSTOPB | CLOCAL;
        newtio.c_cflag = BAUDRATE | CS8 | CREAD | CLOCAL;

        newtio.c_cc[VTIME]    = 1;   /* inter-character timer unused */
        newtio.c_cc[VMIN]     = 1;   /* blocking read until 5 chars received */

        tcflush(fd, TCIFLUSH);
        tcsetattr(fd,TCSANOW,&newtio);

	if (argc > 1) {
		int i;
		char *cmd = argv[1];
		unsigned char cs = 0;
		unsigned char buff[12];
		buff[0] = ':';
		buff[1] = '0';
		buff[2] = '5';
		buff[3] = ',';
		if (argc < 2)
			buff[4] = 'B';
		else
			buff[4] = cmd[0];
		buff[5] = '0';
		buff[6] = '0';
		buff[7] = '0';
		buff[8] = '0';
		for (i = 1; i < 9; i++)
			cs += buff[i];
		buff[9] = ',';
		buff[10] = to_char(cs>>4);
		buff[11] = to_char(cs);
		res = write(fd,buff,12);
		res = write(2,buff,12);
		return res;
	}

        while (STOP==FALSE) {       /* loop for input */
          res = read(fd,buf,255);   /* returns after 5 chars have been input */
	  if (res > 0) {
	        //printf(":%d", res);
		for (i=0; i<res; i++) {
			//if (i == 0)
			//	printf(":");
			//printf("%2.2x", (unsigned char)buf[i]);
			printf("%c", buf[i]);
		}
		printf("\n");
		write(2,buf,res);
	  }
/*	  for (i=0; i<res; i++) {
		switch (state) {
		case 1:
			if ((buf[i] != 0x2) && (x < 255)) {
				detect[x++] = buf[i];
				break;				
			}
			state = 0;
		case 0:
			if (x != 0) {
				int sum = 0, bsum = 0;
				for (j=0; j<x; j++) {
					if (j == 0)
					    printf("\nWord detected: ");
//					if ((detect[j] & 0x7f) == 0)
//						continue;
					if (detect[j] > 0x0) {
						printf("1");
						fprintf(stderr,"1");
						sum += 2;
						bsum++;
					} else {
						printf("0");
						fprintf(stderr,"0");
						sum += 1;
						bsum++;
					}
				}				
				printf(" (%d, %d)\n\n", sum, bsum);
				fprintf(stderr," (%d, %d)\n", sum, bsum);
			}
			if (buf[i] == 0x2)
				state = 1;
			x = 0;
			break;
		default:
			state = 0;
			x = 0;
		}
	  }*/
//	  if (res > 0)
//		fwrite(buf,1,res,stderr);
          if (res == 0) STOP=TRUE;
        }
        tcsetattr(fd,TCSANOW,&oldtio);
       }

