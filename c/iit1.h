//#define OUTPUT_VERBOSE

#ifndef IIT1_H_INCLUDED
#define IIT1_H_INCLUDED

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <windows.h>
#include <conio.h>
#include <stdbool.h>

HANDLE serPort;
HANDLE hTimer;

void oneStep(void);
float controller(bool btnState, float speed, float pedal);

HANDLE startTimer(DWORD period){

    HANDLE hTimer;
    BOOL success=CreateTimerQueueTimer(&hTimer,NULL,(WAITORTIMERCALLBACK)oneStep,(PVOID)NULL,period,period,0);
    if (success)
        return hTimer;
    else
        return INVALID_HANDLE_VALUE;
}

HANDLE serialOpen(const char* device, uint32_t baudRate){
    HANDLE port = CreateFileA(device,GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL,NULL);
    if (port==INVALID_HANDLE_VALUE){
        printf("Could not open %s\n",device);
        return INVALID_HANDLE_VALUE;
    }
    BOOL success = FlushFileBuffers(port);
    if (!success){
        printf("Could not flush serial port buffer\n");
        CloseHandle(port);
        return INVALID_HANDLE_VALUE;
    }
    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = 0;
    timeouts.ReadTotalTimeoutConstant = 5;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 5;
    timeouts.WriteTotalTimeoutMultiplier = 0;

    success = SetCommTimeouts(port,&timeouts);
    if (!success){
        printf("Could not set timeouts\n");
        CloseHandle(port);
        return INVALID_HANDLE_VALUE;
    }

    DCB state;
    state.DCBlength = sizeof(DCB);
    success = GetCommState(port,&state);
    if (!success){
        printf("Could not get serial port settings");
        CloseHandle(port);
        return INVALID_HANDLE_VALUE;
    }
    state.BaudRate=baudRate;
    success = SetCommState(port,&state);
    if (!success){
        printf("Could not set serial settings");
        CloseHandle(port);
        return INVALID_HANDLE_VALUE;
    }
    FlushFileBuffers(port);
    return port;
}

SSIZE_T serialWrite(HANDLE port, uint8_t* buffer, size_t nBytes){
    DWORD bytesTransmitted;
    BOOL success;
    success=WriteFile(port,buffer,nBytes,&bytesTransmitted,NULL);
    if (!success)
        return -1;
    return bytesTransmitted;
}

SSIZE_T serialRead(HANDLE port, uint8_t * buffer, size_t nBytes){
    DWORD bytesReceived;
    BOOL success;
    success=ReadFile(port,buffer,nBytes,&bytesReceived,NULL);
    if (!success)
        return -1;
    return bytesReceived;
}

BOOL closeSerial(HANDLE port){
    return CloseHandle(port);
}

BOOL initializeRT(void){
    serPort=serialOpen("COM5",115200);
    if (serPort==INVALID_HANDLE_VALUE){
        printf("closing\n");
        CloseHandle(serPort);
        return -1;
    }
    hTimer=startTimer(100);
    printf("RT has been started succesfully.\nPress any key to terminate RT.\n");
    return 1;
}

BOOL terminateRT(void){

    DeleteTimerQueueTimer(NULL,hTimer,NULL);
    closeSerial(serPort);
    return 1;
}

void oneStep(void){
    uint8_t bufferIn[25];
    char bufferOut[10];
    static float speed;
    static bool btn;
    static float throttle;
    float pedal;

    DWORD bytesReceived=serialRead(serPort,bufferIn,25);
    if (bytesReceived==25){
        float candidateSpeed;
        float candidatePedal;
        int candidateBtn;

        if (sscanf((char*)bufferIn,"%f:%f:%d",&candidateSpeed,&candidatePedal,&candidateBtn)){
            speed=candidateSpeed;
            pedal=candidatePedal;
            btn=(candidateBtn>0);
        }

    }
    PurgeComm(serPort,PURGE_RXCLEAR);
    throttle=controller(btn,speed,pedal);
    if ((throttle>=0)&&(throttle<=1)){
        sprintf(bufferOut,"%09.7f\n",throttle);
    }
    else{
        sprintf(bufferOut,"0.0000000\n");
    }
    #ifdef OUTPUT_VERBOSE
        printf("%s Tempomat %s, Speed; %f, Pedal: %f, Throttle; %f\n",(bytesReceived==25)?(""):("NO DATA - "),(btn)?("ON"):("OFF"),speed,pedal,throttle);
    #endif
    if (bytesReceived==25){
        serialWrite(serPort,(uint8_t*)bufferOut,10);
    }

}

#endif // IIT1_H_INCLUDED
