/*Non-Canonical Input Processing*/

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>

#define BAUDRATE B38400
#define MODEMDEVICE "/dev/ttyS0"
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define FALSE 0
#define TRUE 1

volatile int STOP=FALSE;
int timeout_seconds = 5;
int timeout_microseconds = 1000000;

int main(int argc, char** argv)
{
    int fd,c, res;
    struct termios oldtio,newtio;
    unsigned char buf[255];
    int i, sum = 0, speed = 0;

    if ( (argc < 2) ||
         ((strcmp("/dev/ttyS0", argv[1])!=0) &&
          (strcmp("/dev/ttyS1", argv[1])!=0) &&
          (strcmp("/dev/ttyS10", argv[1])!=0) &&
          (strcmp("/dev/ttyS11", argv[1])!=0) )) {
        printf("Usage:\tnserial SerialPort\n\tex: nserial /dev/ttyS0\n");
        exit(1);
    }


    /*
    Open serial port device for reading and writing and not as controlling tty
    because we don't want to get killed if linenoise sends CTRL-C.
    */


    fd = open(argv[1], O_RDWR | O_NOCTTY );
    if (fd < 0) { perror(argv[1]); exit(-1); }

    if ( tcgetattr(fd,&oldtio) == -1) { /* save current port settings */
        perror("tcgetattr");
        exit(-1);
    }

    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    /* set input mode (non-canonical, no echo,...) */
    newtio.c_lflag = 0;

    newtio.c_cc[VTIME]    = 1;   /* inter-character timer unused */
    newtio.c_cc[VMIN]     = 0;   /* blocking read until 5 chars received */



    /*
    VTIME e VMIN devem ser alterados de forma a proteger com um temporizador a
    leitura do(s) próximo(s) caracter(es)
    */


    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd,TCSANOW,&newtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }

    printf("New termios structure set\n");

    buf[0] = 0x5c;
    buf[1] = 0x01;
    buf[2] = 0x03;
    buf[3] = 0x02;
    buf[4] = 0x5c;
    res = 5;
 
    res = write(fd,buf,5);
    //printf("%s\n",buf);
    printf("%d bytes written\n", res);
    printf("----------------Trama SET enviada----------------\n");
            printf("Waiting for data...\n");

    // Wait for data or timeout
    while (STOP == FALSE && timeout_seconds > 0)
    {
        res = read(fd, buf + i, 1);
    
        if (res > 0)
        {
            // Data received
            // Process the received data as needed
            STOP=TRUE;
        }
        else if (res == 0)
        {
            // No data received, wait for a specified duration
            usleep(timeout_microseconds);
            timeout_seconds--;
        }
        else
        {
            // Error occurred while reading data
            perror("read");
            break;
        }
    }
    STOP = FALSE;
    i=6;
    while(STOP==FALSE){    
    res = read(fd,buf+i,1);   
    //buf[res]=0;               
        //printf(":%c:%d\n", buf[i], res);
        //printf(":%s\n", buf);
       // printf("%d\n",i);

    if (res>0){ 
       if(buf[i] == 0x5c)
        i++;
       if(buf[i] == 0x01)
        i++;
       if(buf[i] == 0x06)
        i++;
       if(buf[i] == 0x06)
        i++;
       if(buf[i] == 0x5c)
        i++;
    }
    
    else if(i < 10 && res<1)
    STOP=TRUE;  

    if(i==10)
    {
        printf("----------------Trama UA recebida----------------\n");
        STOP = TRUE;
    }
    }





    /*
    O ciclo FOR e as instruções seguintes devem ser alterados de modo a respeitar
    o indicado no guião
    */


    if ( tcsetattr(fd,TCSANOW,&oldtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }


    close(fd);
    return 0;
}