/*Non-Canonical Input Processing*/

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>

#define BAUDRATE B9600
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define FALSE 0
#define TRUE 1

volatile int STOP=FALSE;

int main(int argc, char** argv)
{
    int fd,c, res;
    struct termios oldtio,newtio;
    char buf[255];

    if ( (argc < 2) ||
         ((strcmp("/dev/ttyS0", argv[1])!=0) &&
          (strcmp("/dev/ttyS1", argv[1])!=0) )) {
        printf("Usage:\tnserial SerialPort\n\tex: nserial /dev/ttyS1\n");
        exit(1);
    }


    /*
    Open serial port device for reading and writing and not as controlling tty
    because we don't want to get killed if linenoise sends CTRL-C.
    */


    fd = open(argv[1], O_RDWR | O_NOCTTY );
    if (fd < 0) { perror(argv[1]); exit(-1); }

    if (tcgetattr(fd,&oldtio) == -1) { /* save current port settings */
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
    int i = 0;
    char helper;
    printf("Press enter when sent\n");
    while(helper !='\n')
    {    
    scanf("%c",&helper);
    }

    printf("New termios structure set\n");

    while(STOP==FALSE){    
    res = read(fd,buf+i,1);   
    //buf[res]=0;               
        printf(":%s:%d\n", buf, res);
        printf("%d\n",i);

    if (res>0){ 
       if(buf[i] == (char*)0x5c)
        i++;
       if(buf[i] == (char*)0x01)
        i++;
       if(buf[i] == (char*)0x03)
        i++;
       if(buf[i] == (char*)0x02)
        i++;
       if(buf[i] == (char*)0x5c)
        i++;
    }
    else if(i < 5 && res<1)
    STOP=TRUE;  

    if(i==5)
    printf("trama set recebida\n");

    if(i>=5&& i <=10){
        if(i==5){    
        buf[i] = (char*)0x5c;
        i++;
        }
        if(i==6){    
        buf[i] = (char*)0x01;
        i++;
        }
        if(i==7){    
        buf[i] = (char*)0x07;
        i++;
        }
        if(i==8){    
        buf[i] = (char*)0x06;
        i++;
        }
        if(i==9){    
        buf[i] = (char*)0x5c;
        i++;
        }
    }
    if(i == 10){
    res = write(fd,buf+i,5);
    printf("%s\n",buf);
    printf("trama UA enviada\n");
    STOP = TRUE;
 }
}
    /*for(int i= 5; i<255; i++){
    rs = read(fd,buf,1);
    buf[res]=0;
    printf(":%s:%d\n", buf, res);    
    if(buf[0]=='z') break;
    }*/

    /*
    O ciclo WHILE deve ser alterado de modo a respeitar o indicado no guião
    */
    sleep(1);
    tcsetattr(fd,TCSANOW,&oldtio);
    close(fd);
    return 0;
}

