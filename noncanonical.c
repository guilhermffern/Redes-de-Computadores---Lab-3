/*Non-Canonical Input Processing*/

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>

#define BAUDRATE B38400
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define FALSE 0
#define TRUE 1

#define F 0x5c
#define A 0x03
#define ALT_A 0x01
#define C_SET 0x03
#define C_UA 0x07
#define C_NS_0 0x00
#define C_NS_1 0x02
#define C_RR_0 0x01
#define C_RR_1 0x21
#define C_REJ_0 0x05
#define C_REJ_1 0x25
#define C_DISC 0x0B
volatile int STOP=FALSE;
volatile int TIMERVAR = FALSE;
int RRX = 0;
int RRX_Read = 0;
int resendSize, alarmeMax=5, alarmeTime=3, alarmeCounter = 0;

void alarme();
void readSET(int fd);
void writeUA(int fd);
void writeRR(int fd);
void readRR(int fd);
void writeDISC(int fd);
void readDISC(int fd);

typedef enum{
    Init,
    StateF,
    StateA,
    C,
    BCC,
    Other,
    Stop
} frameStates;

frameStates frameState = Init;

int SMFlag = 0;

char resend[255], buf[255], SET[5], UA[5],RR[5],DISC[5];

void alarme(){ /* picks up alarm */
    int res, i;
    printf("Retrying connection in %d seconds...\n", alarmeTime);
   
    res = write(fd,resend,resendSize);
    printf("%d bytes written\n", res);
    alarmeCounter++;
    if (alarmeCounter == alarmeMax){
        printf("## WARNING: Reached max (%d) retries. ## \nExiting...\n", alarmMax);
        exit(1);
    }
    
    alarm(alarmeTime);
}

void readSET(int fd){
    int res;
    while(STOP==FALSE){
    res = read(fd,buf,1);

    switch(frameState){
        case Init:
            if(buf[0]==F)
                frameState=StateF;
        
        case StateF:
            if(buf[0]==F)
                frameState=StateF;

            else if(buf[0]==A || buf[0]==ALT_A)
                frameState = StateA;
            
            else{
                frameState = Init;
                SMFlag = 0;
            }
            break;
        
        case StateA:
            if(buf[0]==F)
                frameState=StateF;

            else if(buf[0]==C_SET)
                frameState = C;
            
            else{   
                frameState = Init;
                SMFlag = 0;
            }
            break;            

        case C:
            if(buf[0]==F)
                frameState=StateF;

            else if(buf[0]==A^C_SET||buf[0]==ALT_A^C_SET)
                frameState = BCC;
            
            else{
                frameState = Init;
                SMFlag = 0;
            }
            break;
    }
        if(buf[0]==F && SMFlag == 1)
            STOP = TRUE;

        if(buf[0] == F)
            SMFlag = 1;

}
SMFlag = 0;
printf("trama SET recebida\n");
STOP = FALSE;
}

void writeUA(int fd){
    int res;
    UA[0] = F;   
    UA[1] = A;      
    UA[2] = C_UA;     
    UA[3] = C_UA^A;  
    UA[4] = F;    
    res = write(fd,UA,5);
    printf("trama UA enviada");
}

void writeRR(int fd){
    int i, res;
    printf("--- Sending RR ---\n");
    RR[0] = F;  
    RR[1] = A;   
    if (RRX == 0){   
        RR[2] = C_RR_1;       
        RRX = 1;
    }
    else{
        RR[2] = C_RR_0;
        RRX = 0;
    }
    RR[3] = RR[2]^A; 
    RR[4] = F;   
    res = write(fd,RR,5);
}

void readRR (int fd){
    char C_RR;
    int res;

    if(RRX_Read==0)
        C_RR = C_RR_1;
    else
        C_RR = C_RR_0;


    while (STOP == FALSE) {     
        res = read(fd,buf,1);       
        switch(frameState){

            case Init:
                if (buf[0] == F)
                    frameState = StateF;
                break;

            case StateF:
                if (buf[0] == F){
                    frameState = StateF;
                    SMFlag = 0;
                }
                else if (buf[0] == A || buf[0] == ALT_A)
                    frameState = StateA;
                
                else{
                    frameState = Init;          
                    SMFlag = 0;
                }
                break;

            case StateA:
                if (buf[0] == F)
                    frameState = StateF;
                
                else if (buf[0] == C_RR)
                    frameState = C;
                
                else{
                    frameState = Init; 
                    SMFlag = 0;
                }
                break;  

            case C:
                if (buf[0] == F){
                    frameState = StateF;
                    SMFlag = 0;
                }
                else if (buf[0] == A^C_RR || buf[0] == ALT_A^C_RR){
                    frameState = BCC;
                    STOP = TRUE;
                }
                else{
                    frameState = Init;
                    SMFlag = 0;
                }
                break;
            
            case BCC:
                frameState = Init;
                break;
        } 
        
        if (buf[0] == F && SMFlag == 1)
            STOP = TRUE;
        
        if(buf[0] == F)
            SMFlag = 1;
    } 
    STOP = FALSE;
    SMFlag = 0;
    if(RRX_Read == 0)
        RRX_Read = 1;
    else 
        RRX_Read = 0;
}

void readDISC (int fd){
    int res;
     while (STOP == FALSE) {    
        res = read(fd,buf,1);
        switch(frameState){

            case Init:
                if (buf[0] == F)
                    frameState = StateF;
                break;

            case StateF:
                if (buf[0] == F){
                    frameState = StateF;
                    SMFlag = 0;
                }
                else if (buf[0] == A || buf[0] == ALT_A)
                    frameState = StateA;
                
                else{
                    frameState = Init; 
                    SMFlag = 0;
                }
                break;

            case StateA:
                if (buf[0] == F)
                    frameState = StateF;
                
                else if (buf[0] == C_DISC)
                    frameState = C;
                
                else{
                    frameState = Init; 
                    SMFlag = 0;
                }
                break;  

            case C:
                if (buf[0] == F){
                    frameState = StateF;
                    SMFlag = 0;
                }
                else if (buf[0] == A^C_DISC || buf[0] == ALT_A^C_DISC){
                    frameState = BCC;
                    STOP = TRUE;
                }
                else{
                    frameState = Init;
                    SMFlag = 0;
                }
                break;
            
            case BCC:
                frameState = Init;                
                break;
        } 
        
        if (buf[0] == F && SMFlag == 1)
            STOP = TRUE;
        
        if(buf[0] == F)
            SMFlag = 1;
    } 
    STOP = FALSE;
    SMFlag = 0;
}

void writeDISC (int fd){
    int res;
    DISC[0] = F;
    DISC[1] = A;
    DISC[2] = C_DISC;
    DISC[3] = C_DISC^A;
    DISC[4] = F;
    res = write(fd,DISC,5);
}

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
   /* int i = 0;

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
    else if(res<1){
    //STOP=TRUE;  
    printf("----------------ERROR----------------\n");
                usleep(timeout_microseconds);

    i=0;
     res = read(fd, buf + i, 1);
    }
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

       else if(buf[12] == C_I0){
        i++;
        RR = 0;
        }

        else if(buf[i] == C_I1){
        i++;
        RR = 1;
        }
       else if(buf[i] == C_SET^A)
        i++;

    }
    else
    STOP=TRUE;  
    }
    if(i>=14){
        if(RR==0)
        printf("----------------Trama I0 detetada------------------\n");
        if(RR==1)
        printf("----------------Trama I1 detetada------------------\n");
        buf[14] = F;
        buf[15] = A;
        if(RR==0)
        buf[16] = C_RR0;
        if(RR==1)
        buf[16] = C_RR1;
        buf[17] = C_SET^A;
        buf[18] = F;
        res = write(fd,buf+i,5);//Usar variavel global que incrementa com os dados enviados para dizer o nº de carateres que irao ser escritos
        if(RR==0)
        printf("----------------Trama RR0 enviada----------------\n");
        if(RR==1)
        printf("----------------Trama RR1 enviada----------------\n");
        STOP=TRUE;
    }

}

    /*
    O ciclo WHILE deve ser alterado de modo a respeitar o indicado no guião
    */
    int timeout_seconds = 5;  // Adjust the timeout duration as needed
    int timeout_microseconds = 1000000;

    printf("Waiting for data...\n");
    
    // Wait for data or timeout
    while (TIMERVAR == FALSE && timeout_seconds > 0)
    {
        res = read(fd, buf, 1);

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
    while (TIMERVAR == FALSE && timeout_seconds > 0)
    {
        res = read(fd, buf, 1);

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
    readSET(fd);
    writeUA(fd);
    usleep(timeout_microseconds);
    writeRR(fd);
    while (TIMERVAR == FALSE && timeout_seconds > 0)
    {
        res = read(fd, buf, 1);

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
    readDISC(fd);
    writeDISC(fd);
    printf("Iam finished\n\n");
    alarm(0);
    alarmeCounter = 0;
    sleep(1);
    tcsetattr(fd,TCSANOW,&oldtio);
    close(fd);
    return 0;
}
