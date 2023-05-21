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

#define F        0x7E
#define ESC      0x7D
#define A        0x03
#define A_DISC   0x01
#define C_SET    0x03
#define C_UA     0x07
#define C_RR0    0x05
#define C_RR1    0xB5
#define C_REJ0   0x01
#define C_REJ1   0x81
#define C_I0     0x00
#define C_I1     0xB0
#define C_DISC   0x0B
int typeI=1;
int RR;
volatile int STOP=FALSE;
volatile int TIMERVAR=FALSE;
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
    i=5;
    while (TIMERVAR == FALSE && timeout_seconds > 0)
    {
        res = read(fd, buf + i, 1);
    
        if (res > 0)
        {
            // Data received
            // Process the received data as needed
            TIMERVAR=TRUE;
        }
        else if (res == 0)
        {
            // No data received, wait for a specified duration
            {
            buf[0] = 0x5c;
            buf[1] = 0x01;
            buf[2] = 0x04;
            buf[3] = 0x02;
            buf[4] = 0x5c;
            res = 5;
        
            res = write(fd,buf,5);
            printf("----------------Trama SET reenviada----------------\n");
            usleep(timeout_microseconds);
            //timeout_seconds--;
            }
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
    TIMERVAR = FALSE;


while(STOP==FALSE){    
    if(i>4 && i<11){
        if(i>5) 
        res = read(fd,buf+i,1);
    if (res>0){
       if(buf[i] == 0x5C)
        i++;
 
       else if(buf[i] == 0x01)
        i++;

       else if(buf[i] == 0x07)
        i++;

       else if(buf[i] == 0x06)
        i++;
       else if (buf[i]==0x5c)
        i++;
    }
    else if(res<1){
    //STOP=TRUE;  
    printf("----------------ERROR----------------\n");
    usleep(timeout_microseconds);
    i=5;
     res = read(fd, buf + i, 1);
    }
    }
   if(i==10)
    {
        printf("----------------Trama UA recebida----------------\n");
        //printf(":%s\n", buf);
        buf[10] = F;
        buf[11] = A;
        if(typeI==0)
        buf[12] = C_I0;
        if(typeI==1)
        buf[12] = C_I1;
        //buf[12] = C_I1;
        buf[13] = C_SET^A;
        //NOW ITS DATA//
        res = write(fd,buf+i,4);//Usar variavel global que incrementa com os dados enviados para dizer o nº de carateres que irao ser escritos
        //printf(":%s\n", buf);
        if(typeI==0)
        printf("----------------Trama I0 enviada-----------------\n");
        if(typeI==1)
        printf("----------------Trama I1 enviada-----------------\n");
        i = 14;
    }
    if(i==14)
    printf("Waiting for data...\n");
    while (TIMERVAR== FALSE && timeout_seconds > 0 && i==14)
    {
        res = read(fd, buf + i, 1);

        if (res > 0)
        {
            // Data received
            // Process the received data as needed
            TIMERVAR=TRUE;
        }
        else if (res == 0)
        {
            // No data received, wait for a specified duration
            usleep(2*timeout_microseconds);
            timeout_seconds--;
            /*    buf[10] = F;
                buf[11] = A;
                buf[12] = C_I0;
                //buf[12] = C_I1;
                buf[13] = C_SET^A;
                //NOW ITS DATA//
                //res = write(fd,buf+i,4);//Usar variavel global que incrementa com os dados enviados para dizer o nº de carateres que irao ser escritos
                //printf(":%s\n", buf);
                printf("----------------Trama I enviada-----------------\n");*/
        }
        else
        {
            // Error occurred while reading data
            perror("read");
            break;
        }
    }
    TIMERVAR=FALSE;
    if(i>13 && i<19){
        if(i>14) 
        res = read(fd,buf+i,1);
    if (res>0){ 
        if(buf[i] == F)
        i++;
 
        else if(buf[i] == A)
        i++;

        else if(buf[i] == C_RR0){
        i++;
        RR = 0;
        }

        else if(buf[i] == C_RR1){
        i++;
        RR = 1;
        }

        else if(buf[i] == C_SET^A)
        i++;

        else  if(buf[i] == F)
        i++;

        else
        STOP=TRUE;  

    }
    else
    STOP=TRUE;  
    }
    if(i==19){
        if(RR==0)
        printf("----------------Trama RR0 detetada------------------\n"); 
        if(RR==1)
        printf("----------------Trama RR1 detetada------------------\n"); 
        STOP=TRUE;
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