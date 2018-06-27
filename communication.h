#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

#include <stdio.h>
#include <string.h>
#include <pigpio.h>
#include <math.h>

#define MAX_COMMAND_SIZE 1024
#define MAX_PULSES 24000
#define IR_PIN 12
#define IR_OUT_PIN 13
#define VLC_PIN 14
#define VLC_OUT_PIN 15
#define OUTSIDE_CODE 0
#define INSIDE_CODE  1
#define MIN_MESSAGE_GAP 562
#define MAX_MESSAGE_END 9000
#define MAX_TRANSITIONS 500
#define IR_FREQ 38000 //38kHz
#define VLC_FREQ 20000000 //20Mhz
static volatile uint32_t ir_code = 0;
int type = -1;

typedef struct
{
    int state;
    int count;
    int level;
    uint16_t micros[MAX_TRANSITIONS];
} decode_t;

void     alert(int gpio, int level, uint32_t tick);
uint32_t decodeSig(decode_t * decode);
void     updateState(decode_t * decode, int level, uint32_t micros);
uint32_t ir_rec_callback(uint32_t IR_id);
uint32_t vlc_rec_callback(uint32_t VLC_id);

int IR_rec()
{
    if (gpioInitialise() < 0)
    {
      return 1 ;
    }
    type = 0;
    gpioSetMode(IR_PIN, PI_INPUT);
    gpioSetWatchdog(IR_PIN, 5);
    gpioSetAlertFunc(IR_PIN, alert);

    return ir_code;
}

int VLC_rec()
{
    if (gpioInitialise() < 0)
    {
      return 1 ;
    }
    type = 1;
    gpioSetMode(VLC_PIN, PI_INPUT);
    gpioSetWatchdog(VLC_PIN, 5);
    gpioSetAlertFunc(VLC_PIN, alert);

    return ir_code;
}

void alert(int gpio, int level, uint32_t tick)
{
    static int inited = 0;
    static decode_t activeHigh, activeLow;
    static uint32_t lastTick;
    uint32_t diffTick;

    if (!inited)
    {
      inited = 1;
      activeHigh.state = OUTSIDE_CODE; 
      activeHigh.level = PI_LOW;
      activeLow.state  = OUTSIDE_CODE; 
      activeLow.level  = PI_HIGH;
      lastTick = tick;
      return;
    }

    diffTick = tick - lastTick;
    if (level != PI_TIMEOUT) 
      lastTick = tick;
    updateState(&activeHigh, level, diffTick);
    updateState(&activeLow, level, diffTick);
}

void updateState(decode_t * decode, int level, uint32_t micros)
{

    if (decode->state == OUTSIDE_CODE)
    {
      if (level == decode->level)
      {
         if (micros > MIN_MESSAGE_GAP)
         {
            decode->state = INSIDE_CODE;
            decode->count = 0;
         }
      }
    }
    else
    {
      if (micros > MAX_MESSAGE_END)
      {
          if (!ir_code) ir_code = decodeSig(decode);

          decode->state = OUTSIDE_CODE;
      }
      else
      {
         if (decode->count < (MAX_TRANSITIONS-1))
         {
            if (level != PI_TIMEOUT)
               decode->micros[decode->count++] = micros;
         }
      }
    }
}

int compare(unsigned int oldval, unsigned int newval)
{

   if(oldval >=6000) {}

   else if(oldval >= 4000) {return 2;}

   else if((oldval >= 450 && oldval <= 700) && (newval >= 450 && newval <= 700)) {return 0;}

   else if((oldval >= 1500 && oldval <= 1800) && (newval >= 1500 && newval <= 1800)) {return 1;}

   else{}
}

uint32_t decodeSig(decode_t * decode)
{

    int i, result, value;
    //string bitString;
    unsigned char bitString[decode->count-2];

    if (decode->count < 6) {return 0;}

    for (i=0; i < (decode->count-2); i=i+2)
    {
      value = compare(decode->micros[i], decode->micros[i+1]);

      if(value == 1 || value == 0){
        bitString[i/8] |= (value << (i%8));
      }
      if (value == 2){ //End of valid signal
         break;
      }
    }
    char *endp = NULL;

    uint32_t n = strtoul(bitString, &endp, 2);
    if(type == 0)
    {
      ir_rec_callback(n);
      type = -1;
    }

    else if(type == 1)
    {
      vlc_rec_callback(n);
      type = -1;
    }

    return n;
}

static inline void addPulse(uint32_t onPins, uint32_t offPins, uint32_t duration, gpioPulse_t *irSignal, int *pulseCount)
{
  int index = *pulseCount;

  irSignal[index].gpioOn = onPins;
  irSignal[index].gpioOff = offPins;
  irSignal[index].usDelay = duration;

  (*pulseCount)++;
}

static inline void carrierFrequency(uint32_t outPin, double frequency, double dutyCycle, double duration, gpioPulse_t *irSignal, int *pulseCount)
{
  double waveTime = 1000000.0 / frequency; // 1000000 microseconds in a second
  int onDuration = (int)round(waveTime * dutyCycle);
  int offDuration = (int)round(waveTime * (1.0 - dutyCycle));

  int totalCycles = (int)round(duration / waveTime);
  int totalPulses = totalCycles * 2;

  int i;
  for (i = 0; i < totalPulses; i++)
  {
    if (i % 2 == 0)
    {
      addPulse(1 << outPin, 0, onDuration, irSignal, pulseCount);
    }
    else
    {
      addPulse(0, 1 << outPin, offDuration, irSignal, pulseCount);
    }
  }
}

static inline void gap(uint32_t outPin, double duration, gpioPulse_t *irSignal, int *pulseCount)
{
  addPulse(0, 0, duration, irSignal, pulseCount);
}

static inline int sendSig(const char *code, int freqIn, int outPin)
{           
  int freq = freqIn;           
  double dutyCycle = 0.5;        
  int oneGap = 1688;  
  int zeroGap = 562;  
  int leadingPulseDuration = 9000;
  int leadingGapDuration = 4500; 
  int onePulse = 1688;
  int zeroPulse = 562;
  int sendTrailingPulse = 1;  

  size_t codeLen = strlen(code);

  //err
  if (codeLen > MAX_COMMAND_SIZE)
  {
    return 1;
  }

  gpioPulse_t irSignal[MAX_PULSES];
  int pulseCount = 0;

  // Generate Code
  carrierFrequency(outPin, freq, dutyCycle, leadingPulseDuration, irSignal, &pulseCount);
  gap(outPin, leadingGapDuration, irSignal, &pulseCount);

  int i;
  for (i = 0; i < codeLen; i++)
  {
    if (code[i] == '0')
    {
      carrierFrequency(outPin, freq, dutyCycle, zeroPulse, irSignal, &pulseCount);
      gap(outPin, zeroGap, irSignal, &pulseCount);
    }
    else if (code[i] == '1')
    {
      carrierFrequency(outPin, freq, dutyCycle, onePulse, irSignal, &pulseCount);
      gap(outPin, oneGap, irSignal, &pulseCount);
    }
  }

  if (sendTrailingPulse)
  {
    carrierFrequency(outPin, freq, dutyCycle, onePulse, irSignal, &pulseCount);
  }

  gpioInitialise();
  gpioSetMode(outPin, PI_OUTPUT);
  gpioWaveClear();
  gpioWaveAddGeneric(pulseCount, irSignal);
  int waveID = gpioWaveCreate();

  if (waveID >= 0){
    int result = gpioWaveTxSend(waveID, PI_WAVE_MODE_ONE_SHOT);
  }
  else{
    printf("Error!\n %i", waveID);
  }

  while (gpioWaveTxBusy()){
    time_sleep(0.1);
  }

  if (waveID >= 0){
    gpioWaveDelete(waveID);
  }

  gpioTerminate();
  return 0;
}

//Char to Binary
void irSendStr(char* input)
{
  sendSig(input, IR_FREQ, IR_OUT_PIN);
}

//Char to Binary
void irSendInt(int input)
{
    int bits = sizeof(input) * 8;
    char *s = malloc(bits+1);  // +1 for '\0' terminator
    s[bits] = '\0';
    unsigned int u = *(unsigned int*)&input;
    int i;
    unsigned int mask = 1 << (bits-1); // fill in values right-to-left
    for (i = 0; i < bits; i++, mask >>= 1)
        s[i] = ((u & mask) != 0) + '0';

  sendSig(s, IR_FREQ, IR_OUT_PIN);
}

//Char to Binary
void vlcSendStr(char* input)
{
  sendSig(input, VLC_FREQ, VLC_OUT_PIN);
}

//Char to Binary
void vlcSendInt(int input)
{
    int bits = sizeof(input) * 8;
    char *s = malloc(bits+1);  // +1 for '\0' terminator
    s[bits] = '\0';
    unsigned int u = *(unsigned int*)&input;
    int i;
    unsigned int mask = 1 << (bits-1); // fill in values right-to-left
    for (i = 0; i < bits; i++, mask >>= 1)
        s[i] = ((u & mask) != 0) + '0';

  sendSig(s, VLC_FREQ, VLC_OUT_PIN);
}

#endif
