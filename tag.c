#include <string.h>
#include "libdw1000Spi.h"
#include "libdw1000.h"
#include "dwTestOps.h"
#include <stdio.h>
#include <wiringPiSPI.h>
#include <wiringPi.h>
#include <stdlib.h>
#include <math.h>
#include "communication.h"

//gcc -o tag tag.c libdw1000.c libdw1000Spi.c -lpigpio -lm -lrt -lpthread -lwiringPi

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
#define RANGE_FREQ 100
#define MSG_LEN 20
#define PAIR_FREQ 100

dwDevice_t dwm_device;
dwDevice_t *dwm = &dwm_device;
bool receivedAck = false;
bool sentAck = false;
bool isAnchorArrayAllocated = false;
int expectedMsgId = POLL;
int msgId;
int devID;
bool dupFlag = false;
int numAnchorsRec = 0;
uint64_t lastPair;
float anchorPositions[500][2];
bool failedPck = false;
uint8_t data[MSG_LEN] = {0};
uint64_t timePollAckSentTS, timePollAckReceivedTS, timeRangeSentTS;
uint64_t timePollReceivedTS, timeRangeReceivedTS, timePollSentTS; 
uint64_t send1, reply1, send2, reply2;
double timeComputedRangeTS;
int anchorNum;
uint32_t *anchorList;
float * ranges;

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

typedef struct coordinate coordinate;
struct coordinate {
    float x;
    float y;
    float z;
};
coordinate * res;
coordinate * res2;

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

void delayms(dwDevice_t* dev, unsigned int delayNum)
{

    dwTime_t delayDWM;
    dwTime_t *num = &delayDWM;
    num->full = delayNum;
	dwSetDelay(dwm, num);
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

void vdiff( coordinate vector1,  coordinate vector2, coordinate * result) {
  result->x = vector1.x - vector2.x;
  result->y = vector1.y - vector2.y;
  result->z = vector1.z - vector2.z;
}

void vsum( coordinate vector1,  coordinate vector2, coordinate * result) {
  result->x = vector1.x + vector2.x;
  result->y = vector1.y + vector2.y;
  result->z = vector1.z + vector2.z;
}

void vmul(coordinate vector, coordinate * result, float n) {
  result->x = vector.x * n;
  result->y = vector.y * n;
  result->z = vector.z * n;
}

void vdiv( coordinate * vector,  float n) {
  vector->x = vector->x / n;
  vector->y = vector->y / n;
  vector->z = vector->z / n;
}

void vnorm( coordinate vector, float * result) {
  *result = sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
}

/* Return the dot product of two vectors. */
void dot( coordinate vector1,  coordinate vector2, float * result) {
  *result = vector1.x * vector2.x + vector1.y * vector2.y + vector1.z * vector2.z;
}

void cross( coordinate vector1,  coordinate vector2, coordinate * result) {
  result->x = vector1.y * vector2.z - vector1.z * vector2.y;
  result->y = vector1.z * vector2.x - vector1.x * vector2.z;
  result->z = vector1.x * vector2.y - vector1.y * vector2.x;
}

void computeDistance(struct coordinate a, struct coordinate b, float *distance) {
  *distance = sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
}

void vadd(coordinate * vector,int n) {
  vector->x = vector->x + n;
  vector->y = vector->y + n;
  if (vector->z + n < 0) {
    vector->z = vector->z + fabs(n);
  } else {
    vector->z = vector->z + n;
  }
}

void setId(uint8_t * data, uint32_t id, int index)
{
    for(int i = 0; i < 4; i++)
    {
        data[i+index] = (id >> (i * 8)) & 0xFF;
    }
}

void sendRangeACK(uint32_t anchorId)
{
    //memset(data, 0, sizeof(data));
	//data[MSG_LEN] = {0};
    dwNewTransmit(dwm);
    data[0] = RANGE_ACK;
    setId(data, anchorId, 17);
    dwSetData(dwm, data, MSG_LEN);
    dwStartTransmit(dwm);
}

void sendPollAck(uint32_t anchorId)
{
    //memset(data, 0, sizeof(data));
    //data[MSG_LEN] = {0};
    dwNewTransmit(dwm);
    data[0] = POLL_ACK;
    setId(data, anchorId, 17);
    delayms(dwm, 7000);
    dwSetData(dwm, data, MSG_LEN);
    dwStartTransmit(dwm);
}

void transmitRangeFailed(uint32_t anchorId)
{
    //memset(data, 0, sizeof(data));
    //data[MSG_LEN] = {0};
    dwNewTransmit(dwm);
    data[0] = RANGE_FAILED;
    setId(data, anchorId, 17);
    dwSetData(dwm, data, MSG_LEN);
    dwStartTransmit(dwm);
}
void receiver()
{
    dwNewReceive(dwm);
    dwReceivePermanently(dwm, 1);
    dwStartReceive(dwm);
}
void transmitter(uint8_t * data)
{
	dwNewTransmit(dwm);
	dwSetData(dwm, data, MSG_LEN);
	delayms(dwm, 2000);
	dwStartTransmit(dwm);
}

double getRange()
{
    send1 = timePollReceivedTS - timePollSentTS;
    reply1 = timePollAckSentTS - timePollReceivedTS;
    send2 = timeRangeReceivedTS - timePollAckSentTS;
    reply2 = timeRangeSentTS - timePollAckReceivedTS;
    uint64_t temp = ((send1 * send2) - (reply1 * reply2)) / ((send1 + send2) + (reply1 + reply2));
    timeComputedRangeTS = temp * 0.0046917639786159; // Speed of radio waves * timestamp resolution of DW1000
    return timeComputedRangeTS;

}
void setupTag(int num)
{
    anchorNum = num;
	printf("Initialize DWM1000 ...\n");
	dwInit(dwm, &ops);       // Init libdw
	//dwOpsInit(dwm);
	int result = dwConfigure(dwm); // Configure the dw1000 chip
	if (result == 0) {
	  printf("[OK]\r\n");
	  dwEnableAllLeds(dwm);
	  devID = dwGetDeviceId(dwm);
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

float * getPosition(uint8_t * data)
{
    float *coordinates = malloc(sizeof(float) * 2);
    //Receive x
    union {
        float xFloat;
        unsigned char xBytes[4];
    } receiveX;

    for(int i=20; i < 24; i++)
        receiveX.xBytes[i-20] = data[i];

    //Receive y
    union {
        float yFloat;
        unsigned char yBytes[4];
    } receiveY;

    for(int i=24; i < 28; i++)
        receiveY.yBytes[i-24] = data[i];

    coordinates[0] = receiveX.xFloat;
    coordinates[1] = receiveY.yFloat;

    return coordinates;
}

uint64_t getTimeStamp(uint8_t * data, int index)
{
    int timestamp = 0;
    for (int i=0; i < 5; i++)
    {
    	timestamp |= data[i+index] << (i*8);
    }
    return timestamp;
}
uint32_t getAnchorId(uint8_t * data, int index)
{
    int id = 0;
    for (int i=0; i < 4; i++)
    {
        id |= data[i+index] << (i*8);
    }
    return id;
}

void broadcastToAnchors()
{
    dwNewTransmit(dwm);
    data[0] = PAIR;
    delayms(dwm, 7000);
    dwSetData(dwm, data, MSG_LEN);
    dwStartTransmit(dwm);
}

void sendDonePair(uint32_t anchorId)
{
    dwNewTransmit(dwm);
    data[0] = DONE_PAIR;
    setId(data, anchorId, 17);
    dwSetData(dwm, data, MSG_LEN);
    dwStartTransmit(dwm);
}

void pairWithAnchors()
{
    uint32_t anchorId;
    int currPlace = 0;
    anchorList = (uint32_t *)malloc(sizeof(uint32_t)*anchorNum);
    ranges = (float *)malloc(sizeof(float)*anchorNum); 
    printf("Finding Anchor(s)\n\n");

    while(1)
    {
        if(millis() - lastPair < PAIR_FREQ)
        {
            continue;
        }
        broadcastToAnchors();
        lastPair = millis();
        if(receivedAck == true)
        {
            dwGetData(dwm, data, MSG_LEN);
            msgId = data[0];
            anchorPositions[currPlace][0] = data[20 + numAnchorsRec]; //x
            anchorPositions[currPlace][1] = data[24 + numAnchorsRec]; //y
            if(msgId == PAIR_ACK)
            {
                anchorId = getAnchorId(data, 17);
                if(currPlace == 0)
                {
                    anchorList[currPlace] = anchorId;
                    sendDonePair(anchorId);
                    currPlace = currPlace + 1;        
                    printf("Anchor ID: %u\n", anchorId);
                    printf("Coordinates: (%f, %f) \n", anchorPositions[currPlace][0], anchorPositions[currPlace][1]);
                    printf("\n");
                }
                else
                {
                    for(int i = 0; i < currPlace; i++)
                    {
                        if(anchorId == anchorList[i])
                        {
                            continue;
                        }
                    }
                    anchorList[currPlace] = anchorId;
                    sendDonePair(anchorId);
                    currPlace = currPlace + 1;
                    printf("Anchor ID: %u\n", anchorId);
                    printf("Coordinates: (%f, %f) \n", anchorPositions[currPlace][0], anchorPositions[currPlace][1]);
                    printf("\n");

                } 
            }
        }
        if(currPlace == anchorNum)
        {
            return; //All anchors paired and stored
        }
    }
}

bool addRangeToList(uint32_t anchorId, double distance)
{

    ranges[numAnchorsRec] = distance;
    anchorPositions[numAnchorsRec][0] = data[20 + numAnchorsRec]; //x
    anchorPositions[numAnchorsRec][1] = data[24 + numAnchorsRec]; //y
    numAnchorsRec = numAnchorsRec + 1;
    
    if(numAnchorsRec == anchorNum)
    {
        numAnchorsRec = 0;
        return true;
    }
    return false;
}

int trilateration(coordinate *  result1, coordinate *  result2, coordinate p1,  float r1,  coordinate p2, float r2,  coordinate p3,  float r3, float maxzero) {
    static coordinate ex, ey, ez;
    static float h, i, j;

    vdiff(p2, p1, &ex);
    vnorm(ex, &h);
    if (h <= maxzero) {
    return -1;
    }
    vdiv(&ex, h);

    vdiff(p3, p1,&ey);
    dot(ex, ey, &i);
    vmul(ex, &ez, i);

    vdiff(ey, ez, &ey);
    vnorm(ey, &j);
    if (j > maxzero) {
    vdiv(&ey, j);
    vdiff(p3, p1, &ez);
    dot(ey, ez, &j);
    } else
    j = 0.0;

    cross(ex, ey,&ez);

    h = (r1 * r1 - r2 * r2) / (2 * h) + h / 2;
    j = (r1 * r1 - r3 * r3 + i * i) / (2 * j) + j / 2 - h * i / j;
    i = r1 * r1 - h * h - j * j;
    if (i < -maxzero) {
    return -3;
    } else if (i > 0.0)
    i = sqrt(i);
    else
    i = 0.0;

    vmul(ex,&p2, h);
    vsum(p1, p2,result1);
    vmul(ey,result2, j);
    vsum(*result1,*result2 ,result1);
    *result2 = *result1;
    vmul(ez,&p1,i);
    vsum(*result1, p1 ,result1);
    vmul(ez,&p1, -i);
    vsum(*result2, p1, result2);

    return 0;
}

void tagRun()
{

    printf("Start Tag Run: \n");

    uint32_t anchorId;
    int currAnchor = 0;
    double distance;
    while(1)
    {
    	if(sentAck == true)
    	{
    		sentAck = false;
    		msgId = data[0];
    		if(msgId == POLL_ACK)
    		{
    			timePollAckSentTS = getTxTimeStamp();
    		}
    	}

    	if(receivedAck == true)
    	{
    		dwGetData(dwm, data, MSG_LEN);
    		msgId = data[0];
            anchorId = getAnchorId(data, 17);
    		if (msgId != expectedMsgId)
    		{
    			failedPck = true;
    		}
            if(msgId == POLL && anchorList[currAnchor] == anchorId)
            {
                failedPck = false;
                timePollReceivedTS = getRxTimeStamp();
                expectedMsgId = RANGE;
                sendPollAck(anchorId);
            }
            else if(msgId == RANGE && anchorList[currAnchor] == anchorId)
            {
            	timeRangeReceivedTS = getRxTimeStamp();
            	expectedMsgId = POLL;
            	if(failedPck == false)
            	{
            		timePollSentTS = getTimeStamp(data, 1);
                    timePollAckReceivedTS = getTimeStamp(data, 6);
                    timeRangeSentTS = getTimeStamp(data, 11);
                    distance = getRange();
                    sendRangeACK(anchorId);
                    currAnchor = currAnchor + 1;
                    if(addRangeToList(anchorId, distance) == true)
                    {
                        coordinate terms[3];

                        terms[0].x = anchorPositions[0][0];
                        terms[0].y = anchorPositions[0][1];
                        terms[0].z = 0;

                        terms[1].x = anchorPositions[1][0];
                        terms[1].y = anchorPositions[1][1];
                        terms[1].z = 0;

                        terms[2].x = anchorPositions[2][0];
                        terms[2].y = anchorPositions[2][1];
                        terms[2].z = 0;

                        trilateration(res, res2, terms[0], ranges[0], terms[1], ranges[1], terms[2], ranges[2], 0.0);
                        printf("(%.2f, %.2f)\n", &res->x, &res->y);
                        currAnchor = 0;
                    }
            	}
            	else
            	{
            		transmitRangeFailed(anchorId);
            	}
            }        
    	}
    }
}
