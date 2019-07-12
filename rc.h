// Spec of RC data

#define CHANNELS 8
#define RC_IN_PIN 27 
#define RC_IN_GPIO GPIO_NUM_27

typedef struct 
{
  uint16_t Ch1     : 11; 
  uint16_t Ch2     : 11;
  uint16_t Ch3     : 11;
  uint16_t Ch4     : 11;
  uint16_t Ch5     : 11;
  uint16_t Ch6     : 11;
  uint16_t Ch7     : 11;
  uint16_t Ch8     : 11;
  uint8_t spare    :  8;
}Payload;

#define RCdataSize 12

typedef union
{
  Payload chans;
  uint8_t data[RCdataSize];
} RCdataTY;

RCdataTY RCdata;

int16_t rcValue[CHANNELS];  // in us, center = 1500
uint8_t seqno;
