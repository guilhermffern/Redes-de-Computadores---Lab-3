#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#define BAUDRATE B38400
#define MODEMDEVICE "/dev/ttyS1"
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define FALSE 0
#define TRUE 1

#define F 0x5c
#define A 0x03
#define ALT_A 0x01
#define C_SET 0x03
#define C_UA 0x07
#define C_I_0 0x00
#define C_I_1 0x02
#define C_RR_0 0x01
#define C_RR_1 0x21
#define C_REJ_0 0x05
#define C_REJ_1 0x25
#define C_DISC 0x0B

void alarme();
void writeSET(int fd);
void readUA(int fd);
void writeUA(int fd);
void readUA(int fd);
void writeRR(int fd);
void readRR(int fd);
void writeDISC(int fd);
void readDISC(int fd);

volatile int STOP = FALSE;
volatile int switchwrite_C_RCV = 0;
volatile int switchread_C_RCV = 0;
volatile int switchRR = 0;
volatile int switchreadRR = 0;

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
struct termios oldtio, newtio;

int fd, resendSize, alarmeMax, alarmeTime, alarmeCounter = 0;
int SMFlag = 0;
char resend[255] = {0}, buf[255], SET[5], UA[5], DISC[5];

typedef struct linkLayer{
    char serialPort[50];
    int role;
    int baudRate;
    int numTries;
    int timeOut;
}linkLayer;

void alarme(){ // Gets activated after alarm(alarmetime) signals 
    int res, i;
    printf("\nTrying again\n");
   
    res = write(fd,resend,resendSize);
    printf("%d bytes written\n", res);
    alarmeCounter++;
    if (alarmeCounter == alarmeMax){
        printf("Closing connection...\n");
        exit(1);
    }
    
    alarm(alarmeTime);
}

void writeSET(int fd){
    int res;
    SET[0] = F;    
    SET[1] = A;       
    SET[2] = C_SET;       
    SET[3] = C_SET^A; 
    SET[4] = F;   
    res = write(fd,SET, 5);
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

void readUA (int fd){
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
                
                else if (buf[0] == C_UA)
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
                else if (buf[0] == A^C_UA || buf[0] == ALT_A^C_UA){
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

void writeRR(int fd){
    char RR[5];
    int resRR, i;
    RR[0] = F;       
    RR[1] = A;          
    if (switchRR == 0)      
        RR[2] = C_RR_1;       
    else
        RR[2] = C_RR_0;
    RR[3] = RR[2]^A; 
    RR[4] = F;       
    resRR = write(fd,RR,5);
    switchRR = !switchRR;
    printf("\n%d bytes written\n", resRR);
}

void readRR(int fd){
    int res;
    char C_RCV, buf[255];
    printf("--- Reading RR ---\n");
    
    if(switchreadRR == 0)
        C_RCV = C_RR_1;
    else
        C_RCV = C_RR_0;

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
                
                else if (buf[0] == C_RCV)
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
                else if (buf[0] == A^C_RCV || buf[0] == ALT_A^C_RCV){
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
    switchreadRR = !switchreadRR;
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

/* Opens a connection using the "port" parameters defined in struct linkLayer, returns "-1" on error and "1" on sucess */
int llopen(linkLayer connectionParameters){
    int i = 0;

    fd = open(connectionParameters.serialPort, O_RDWR | O_NOCTTY );     /* Open the serial port*/
    if (fd < 0) { perror(connectionParameters.serialPort); exit(-1); }
    if (tcgetattr(fd,&oldtio) == -1) { /* save current port settings */
        perror("tcgetattr");
        exit(-1);
    }

    /* Configure new port settings*/
    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = connectionParameters.baudRate | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    /* set input mode (non-canonical, no echo,...) */
    newtio.c_lflag = 0;

    newtio.c_cc[VTIME]    = 0;   /* inter-character timer unused */
    newtio.c_cc[VMIN]     = 1;   /* blocking read until 1 char received */


    /*
    VTIME e VMIN devem ser alterados de forma a proteger com um temporizador a
    leitura do(s) pro'ximo(s) caracter(es)
    */


    tcflush(fd, TCIOFLUSH);     /* Flushes data received but not read */

    sleep(1);
    if (tcsetattr(fd,TCSANOW,&newtio) == -1) {     /* Set the new port settings*/
        perror("tcsetattr");
        exit(-1);
    }

    printf("--- New termios structure set ---\n");

    if (connectionParameters.role == 0){
        (void) signal(SIGALRM, alarme); //After Alarmtime SIGALRM gets signaled and calls alarme function
        writeSET(fd);     // Send SET
        for(i = 0; i < 5; i++){
            resend[i] = SET[i];
        }
        resendSize = 5;

        alarmeMax = connectionParameters.numTries;
        alarmeTime = connectionParameters.timeOut;

        alarm(alarmeTime);     //Set alarm that calls function alarme after alarmeTime
        printf("--- UA State Machine has started ---\n"); 
        readUA(fd);
        alarm(0);     //Disables alarm
        printf("--- UA READ -----\n");
        alarmeCounter = 0;
    
        sleep(1);
        return 0;
    }

    else if(connectionParameters.role == 1){
        printf("--- SET State Machine has started ---\n"); 

        readSET(fd);
        printf("--- SET READ -----\n");
        writeUA(fd);

        sleep(1);

        return 0;
    }
    return -1;
}

/* Sends data in buf with bufSize*/
int llwrite(char* buf, int bufSize){
    int i, j, resData, auxSize = 0;
    char xor = buf[0], auxBuf[2000];
    for(i = 1; i < bufSize; i++)
        xor = xor^buf[i];

    /* stuffing */
    for(i = 0; i < bufSize; i++)
        auxBuf[i] = buf[i];
    
    auxSize = bufSize;

    for(i = 0; i < auxSize; i++){
        if(auxBuf[i] == 0x5d){ //if (0x5d) -> (0x5d/0x7d)
            for(j = auxSize+1; j > i+1; j--)
                auxBuf[j] = auxBuf[j-1];
            auxBuf[i+1] = 0x7d;
            auxSize++;
        }
    }

    for(i = 1; i < auxSize; i++){
        if(auxBuf[i] == 0x5c){ //if (0x5c) -> (0x5d/0x7c)
            auxBuf[i] = 0x5d;
            for(j = auxSize+1; j > i+1; j--)            
                auxBuf[j] = auxBuf[j-1];
            auxBuf[i+1] = 0x7c;
            auxSize++;
        }
    }

    char str[auxSize+6];

    str[0] = F;          
    str[1] = A;             
    if (switchwrite_C_RCV == 0) 
        str[2] = C_I_0;       
    else
        str[2] = C_I_1;
    switchwrite_C_RCV = !switchwrite_C_RCV;
    str[3] = str[2]^A;      

    for(j = 0; j < auxSize; j++)
        str[j+4] = auxBuf[j];
    
    if(xor == 0x5c){
        auxSize++;
        str[auxSize+4] = 0x5d;
        str[auxSize+5] = 0x7c;
        str[auxSize+6] = F;

    }
    else{
        str[auxSize+4] = xor;
        str[auxSize+5] = F;
    }

    for(j = 0; j < auxSize+6; j++)
        resend[j] = str[j];
    resendSize = auxSize+6;

    resData = write(fd, str, auxSize+6);
    printf("%d bytes written\n", resData);

    alarm(alarmeTime); 
    readRR(fd);
    printf("--- RR READ -----\n");
    alarm(0);
    alarmeCounter = 0;
    printf("--- RR Checked ---\n");
    return 1;
}

int llread(char* packet){
    int i = 0, j, res, xor, bytes_read, SMFlag = 0, destuffFlag = 0, skip = 0;
    char aux, aux2, C_RCV, buf[1], str[1050];
    frameState = Init;
    if(switchread_C_RCV == 0)
        C_RCV = C_I_0;
    else
        C_RCV = C_I_1;

    while (STOP == FALSE){       
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
                
                else if (buf[0] == C_RCV)
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
                else if (buf[0] == A^C_RCV || buf[0] == ALT_A^C_RCV)
                    frameState = BCC;
                                
                else{
                    frameState = Init;
                    SMFlag = 0;
                }
                break;
            
            case BCC:
                frameState = Init;
                break;
        } 

        // destuffing 
        if (buf[0] == 0x5d)
            destuffFlag = 1;

        if (destuffFlag && buf[0] == 0x7c){
            str[i-1] = 0x5c;
            skip = 1;
        } 
        else if (destuffFlag && buf[0] == 0x7d){
            skip = 1;
        } 
        // if an ESC byte is found change the first char
        if (!skip){
            str[i] = buf[0];
            if (i > 0)
                aux = str[i-1];
            i++;
        }
        else{
            skip = 0;
            destuffFlag = 0;
        }

        // Compares FLAG and checks BCC
        if (buf[0] == F && SMFlag && i > 0){
                xor = str[4];
                for(j = 5; j < i - 2; j++)
                    xor = xor^str[j];
                if(aux == xor){
                    STOP = TRUE;
                    printf("\n --- FRAME READ ---\n");
                }
                else{
                    printf("XOR VALUE IS: 0x%02x\nShould be: 0x%02x\n", (unsigned int)(xor & 0xff), (unsigned int)(aux & 0xff));
                    printf("\n --- BCC2 FAILED! ---\n");
                    return -1;
                }
            
        }

        if(buf[0] == F)
            SMFlag = 1;
    }
    STOP = FALSE;
    switchread_C_RCV = !switchread_C_RCV;

    for(j = 4; j < i-2; j++)
        packet[j-4] = str[j];
    bytes_read = i-6;

    writeRR(fd);

    return bytes_read;
}


int llclose(linkLayer connectionParameters, int showStatistics){ 
    int i;
    printf("\n\n --- PREPARING TO CLOSE ---\n\n");

    if(connectionParameters.role == 0){ //tx
        writeDISC(fd);
        for(i = 0; i < 5; i++)
            resend[i] = DISC[i];
        alarm(alarmeTime);
        readDISC(fd);
        printf("--- DISC READ -----\n");
        alarm(0);
        alarmeCounter = 0;
        printf("\n\n --- READY TO CLOSE ---\n\n");
        writeUA(fd);
        sleep(1);
        if (tcsetattr(fd,TCSANOW,&oldtio) == -1){
            perror("tcsetattr");
            exit(-1);
        }
        
        close(fd);
        return 0;
    }

    if(connectionParameters.role == 1){ //rx    
        sleep(1); 

        readDISC(fd);
        printf("--- DISC READ OK ! ---\n");

        writeDISC(fd);
        for(i = 0; i < 5; i++)
            resend[i] = DISC[i];
        alarm(alarmeTime);
        readUA(fd);
        alarm(0);
        alarmeCounter = 0;

        printf("--- UA READ OK ! ---\n");

        ("\n\n --- READY FOR CLOSURE ---\n\n");
        if (tcsetattr(fd,TCSANOW,&oldtio) == -1){
            perror("tcsetattr");
            exit(-1);
        }
        
        close(fd);
        return 0;
    }

    return -1;
}
