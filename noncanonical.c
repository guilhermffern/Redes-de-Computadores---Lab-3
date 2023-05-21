/*Non-Canonical Input Processing*/

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>

#define BAUDRATE B38400
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
volatile int STOP=FALSE;
volatile int TIMERVAR = FALSE;

int main(int argc, char** argv)
{
    int fd,c, res;
    struct termios oldtio,newtio;
    unsigned char buf[255];

    if ( (argc < 2) ||
         ((strcmp("/dev/ttyS0", argv[1])!=0) &&
          (strcmp("/dev/ttyS1", argv[1])!=0) &&
          (strcmp("/dev/ttyS10", argv[1])!=0) &&
          (strcmp("/dev/ttyS11", argv[1])!=0) )) {
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

    int timeout_seconds = 5;  // Adjust the timeout duration as needed
    int timeout_microseconds = 1000000;

    printf("Waiting for data...\n");
    
    // Wait for data or timeout
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
    TIMERVAR=FALSE;
    printf("New termios structure set\n");

    while(STOP==FALSE){ 
    //buf[res]=0;               
       // printf(":%s:%d\n", buf, res);
        //printf("%d\n",i);
    if(i<5){
        if(i>0) 
        res = read(fd,buf+i,1);   
    if (res>0){ 
       if(buf[i] == 0x5c)
        i++;
       else if(buf[i] == 0x01)
        i++;
       else if(buf[i] == 0x03)
        i++;
       else if(buf[i] == 0x02)
        i++;
       else if(buf[i] == 0x5c)
        i++;
    }
    else if(res<1)
    STOP=TRUE;  
    }

    if(i==5){
    printf("----------------Trama SET recebida----------------\n");

        buf[5] = 0x5c;

        buf[6] = 0x01;
 
        buf[7] = 0x07;
   
        buf[8] = 0x06;
   
        buf[9] = 0x5c;
    res = write(fd,buf+i,5);
   // printf("%s\n",buf);
    printf("----------------Trama UA enviada------------------\n");
    i=10;
    //STOP = TRUE;
    }
    // printf(":%s\n", buf); 
    if(i==10) 
    printf("Waiting for data...\n");
    while (TIMERVAR== FALSE && timeout_seconds > 0 && i==10)
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
    TIMERVAR=FALSE;

    if(i>9 && i<14){
        if(i>10) 
        res = read(fd,buf+i,1);
    if (res>0){ 
       // printf("%d\n",res);
       if(buf[i] == F)
        i++;
 
       else if(buf[i] == A)
        i++;

       else if(buf[i] == C_I0)
        i++;

       else if(buf[i] == C_SET^A)
        i++;

    }
    else
    STOP=TRUE;  
    }
    if(i>=14){
        printf("----------------Trama I detetada------------------\n");
        buf[14] = F;
        buf[15] = A;
        buf[16] = C_RR0;
        //buf[16] = C_RR1;
        buf[17] = C_SET^A;
        buf[18] = F;
        res = write(fd,buf+i,5);//Usar variavel global que incrementa com os dados enviados para dizer o nº de carateres que irao ser escritos
        printf("----------------Trama RR enviada----------------\n");
        STOP=TRUE;
    }

}

    /*
    O ciclo WHILE deve ser alterado de modo a respeitar o indicado no guião
    */
    sleep(1);
    tcsetattr(fd,TCSANOW,&oldtio);
    close(fd);
    return 0;
}

