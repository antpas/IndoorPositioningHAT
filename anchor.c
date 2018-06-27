#include <string.h>
#include "libdw1000Spi.h"
#include "libdw1000.h"
#include "dwTestOps.h"
#include <stdio.h>
#include <wiringPiSPI.h>
#include <wiringPi.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include "communication.h"

// Setup RPi0 specific SPI interface 
#define ANTENNA_DELAY 16390
#define SPI_SPEED 4000000
#define CHANNEL 0
//Packet IDs
#define POLL 0
#define POLL_ACK 1
#define RANGE 2
#define RANGE_ACK 3
#define RANGE_FAILED 4
#define PAIR 5
#define PAIR_ACK 6
#define DONE_PAIR 7
#define MSG_LEN 28
#define REPLY_DELAY_US 5000
#define POLL_FREQ 100 //Poll every 0.1 second
#define PAIR_FREQ 100

dwDevice_t dwm_device;
dwDevice_t *dwm = &dwm_device;
bool receivedAck = false;
bool sentAck = false;
int expectedMsgId = POLL_ACK;
int msgId;
int devID;
uint8_t data[MSG_LEN] = {0};
bool vlc_send = false;
int send_vlc_int;
char * send_vlc_char;
int vlc_freq;
bool ir_send = false;
int send_ir_int;
char * send_ir_char;
int ir_freq;
time_t vlc_endwait, ir_endwait;
time_t vlc_start;
time_t ir_start;
time_t vlc_seconds, ir_seconds; 
uint64_t timePollAckReceivedTS, timeRangeSentTS, timePollSentTS, lastPoll;

// Set SPI on Pi
static dwOps_t ops = {
  .spiRead = spiReadDW,
  .spiWrite = spiWriteDW,
  .spiSetSpeed = spiSetSpeed,
  .delayms = delayms,
  .reset = reset
};

static dwDevice_t dev = {
  .ops = &ops,
};

void spiReadDW(dwDevice_t* dev, const void *header, size_t headerLength, void* data, size_t dataLength)
{
    //unsigned char* spiData = (unsigned char*)data;
    wiringPiSPISetup(CHANNEL, SPI_SPEED);
    wiringPiSPIDataRW(CHANNEL, data, dataLength);
}

void spiWriteDW(dwDevice_t* dev, const void *header, size_t headerLength, const void* data, size_t dataLength)
{
    unsigned char* spiData = (unsigned char*)data;

    wiringPiSPISetup(CHANNEL, SPI_SPEED);
    wiringPiSPIDataRW(CHANNEL, spiData, dataLength);
}

void spiSetSpeed(dwDevice_t* dev, dwSpiSpeed_t speed)
{
    wiringPiSPISetup(CHANNEL, SPI_SPEED);
}

void reset(dwDevice_t* dev)
{
    dwSoftReset(dwm);
}
void txcallback(dwDevice_t *dev)
{
    sentAck = true;
}
void rxcallback(dwDevice_t *dev)
{
    receivedAck = true;
}

uint32_t ir_rec_callback(uint32_t IR_id)
{
    printf("%u\n", IR_id);
    return IR_id;
}

uint32_t vlc_rec_callback(uint32_t VLC_id)
{
    printf("%u\n", VLC_id);
    return VLC_id;
}

uint64_t getTxTimeStamp()
{
    dwTime_t* time;
    dwGetTransmitTimestamp(dwm, time);
    return time->full;
}
uint64_t getRxTimeStamp()
{
    dwTime_t* time;
    dwGetReceiveTimestamp(dwm, time);
    return time->full;
}

uint32_t getId()
{   

    static uint32_t serial = 0;

    FILE *filp;
    char buf[512];
    char term;

    filp = fopen ("/proc/cpuinfo", "r");

    if (filp != NULL)
    {
      while (fgets(buf, sizeof(buf), filp) != NULL)
      {
         if (!strncasecmp("serial\t\t:", buf, 9))
         {
            sscanf(buf+9, "%Lx", &serial);
         }
      }

      fclose(filp);
    }
    return serial;
}

void setId(uint8_t * data, uint32_t id, int index)
{
    for(int i = 0; i < 4; i++)
    {
        data[i+index] = (id >> (i * 8)) & 0xFF;
    }
}

void sendPoll(uint32_t anchorId)
{
    while (millis() - lastPoll < POLL_FREQ)
    {
        return;
    }
    
    dwNewTransmit(dwm);
    data[0] = POLL;
    setId(data, anchorId, 17);
    dwSetData(dwm, data, MSG_LEN);
    dwStartTransmit(dwm);
    lastPoll = millis();
}

void receiver()
{
    dwNewReceive(dwm);
    dwReceivePermanently(dwm, 1);
    dwStartReceive(dwm);
}

void send_int_VLC(int sendNum, int freq)
{
	
    vlc_start = time(NULL);
    vlc_seconds = freq;

    vlc_endwait = vlc_start + vlc_seconds;
	vlc_send = true;
	send_vlc_int = sendNum;
	vlc_freq = freq;
}

void send_int_IR(int sendNum, int freq)
{
	
    ir_start = time(NULL);
    ir_seconds = freq;

    ir_endwait = ir_start + ir_seconds;
	ir_send = true;
	send_ir_int = sendNum;
	ir_freq = freq;
}

void setTimeStamp(uint8_t * data, uint64_t timestamp, int index)
{
    for(int i = 0; i < 5; i++)
    {
        data[i+index] = (timestamp >> (i * 8)) & 0xFF;
    }
}

void setPosition(float xPos, float yPos, int index)
{
    union {
        float xPosFloat;
        unsigned char xPosBytes[4];
    } xPosStruct;

    union {
        float yPosFloat;
        unsigned char yPosBytes[4];
    } yPosStruct;

    xPosStruct.xPosFloat = xPos;
    yPosStruct.yPosFloat = yPos;
    uint8_t* xPosUint = (uint8_t*)xPosStruct.xPosBytes;
    uint8_t* yPosUint = (uint8_t*)yPosStruct.yPosBytes;

    for(int i = 0; i < 4; i++)
    {
        data[i+index] = xPosUint[i];
        data[i+index + 4] = yPosUint[i];
    }
}

uint64_t setDelay(dwDevice_t* dev, unsigned int delayNum)
{

    dwTime_t delayDWM;
    dwTime_t *num = &delayDWM;
    num->full = delayNum;
    return dwSetDelay(dwm, num).full;
}

void delayms(dwDevice_t* dev, unsigned int delayNum)
{

    dwTime_t delayDWM;
    dwTime_t *num = &delayDWM;
    num->full = delayNum;
    dwSetDelay(dwm, num);
}

void sendRange()
{
    dwNewTransmit(dwm);
    data[0] = RANGE;
    timeRangeSentTS = setDelay(dwm, REPLY_DELAY_US);
    setTimeStamp(data, timePollSentTS, 1);
    setTimeStamp(data, timePollAckReceivedTS, 6);
    setTimeStamp(data, timeRangeSentTS, 11);
    setId(data, devID, 16);
    dwSetData(dwm, data, MSG_LEN);
    dwStartTransmit(dwm);
}

void setupAnchor()
{
    printf("Initialize DWM1000 ...\n");
    dwInit(dwm, &ops);
    int result = dwConfigure(dwm);
    if (result == 0) {
      printf("[OK]\r\n");
      dwEnableAllLeds(dwm);
      devID = getId();
    } else {
      printf("[ERROR]: %s\r\n", dwStrError(result));
    }

    dwTime_t delay = {.full = ANTENNA_DELAY/2};
    dwSetAntenaDelay(dwm, delay);

    dwAttachSentHandler(dwm, txcallback);
    dwAttachReceivedHandler(dwm, rxcallback);
    dwNewConfiguration(dwm);
    dwSetDefaults(dwm);
    dwEnableMode(dwm, MODE_SHORTDATA_FAST_ACCURACY);
    dwSetChannel(dwm, CHANNEL_2);
    dwSetPreambleCode(dwm, PREAMBLE_CODE_64MHZ_9);

    dwCommitConfiguration(dwm);
    receiver();
    sendPoll(devID);
}

void sendInitialData(float x, float y)
{
    //ID and position of anchor
    dwNewTransmit(dwm);
    data[0] = PAIR_ACK;
    setId(data, devID, 17);
    setPosition(x, y, 20); 
    dwSetData(dwm, data, MSG_LEN);
    dwStartTransmit(dwm);
}

void pairWithTag(float x, float y)
{
    while(1)
    {
        if(receivedAck == true)
        {
            receivedAck = false;
            dwGetData(dwm, data, MSG_LEN);
            msgId = data[0];
            if(msgId == PAIR)
            {
                sendInitialData(x, y);
            }
            if(msgId == DONE_PAIR)
            {
                return;
            }
        }
    }

    return;
}

void runAnchor()
{
    while(1)
    {
    	vlc_start = time(NULL);
        if(vlc_send == true && vlc_start > vlc_endwait)
        {
        	vlcSendInt(send_vlc_int);
        	send_int_VLC(send_vlc_int, vlc_freq);
        }

        ir_start = time(NULL);
        if(ir_send == true && ir_start > ir_endwait)
        {
        	irSendInt(send_ir_int);
        	send_int_IR(send_ir_int, ir_freq);
        }

        if(sentAck == true)
        {
                sentAck = false;
                msgId = data[0];      
                if(msgId == POLL)
                {
                    timePollSentTS = getTxTimeStamp();
                }
                else if(msgId == RANGE)
                {
                    timeRangeSentTS = getTxTimeStamp();
                }
        }
        if(receivedAck == true)
        {
            receivedAck = false;
            dwGetData(dwm, data, MSG_LEN);
            msgId = data[0];  
            if(msgId != expectedMsgId)
            {
                expectedMsgId = POLL_ACK;
                sendPoll(devID);
                return;
            }
            if(msgId == POLL_ACK)
            {
                timePollAckReceivedTS = getRxTimeStamp();
                expectedMsgId = RANGE_ACK;
                sendRange();
            }
            else if (msgId == RANGE_ACK)
            {
                expectedMsgId = POLL_ACK;
                sendPoll(devID);
            }
            else if(msgId == RANGE_FAILED)
            {
                expectedMsgId = POLL_ACK;
                sendPoll(devID);
            }
        }
    }
}
