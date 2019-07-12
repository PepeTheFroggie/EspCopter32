#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <EEPROM.h>

#include "RC.h"

#define WIFI_CHANNEL 4
#define PWMOUT  // normal esc, uncomment for serial esc
#define LED 2
#define CALSTEPS 256 // gyro and acc calibration steps
#define externRC // use of external RC receiver in ppmsum mode
#define webServer // use of webserver to change PID

extern int16_t accZero[3];
extern float yawRate;
extern float rollPitchRate;
extern float P_PID;
extern float I_PID;
extern float D_PID;
extern float P_Level_PID;
extern float I_Level_PID;
extern float D_Level_PID;

volatile boolean recv;
//volatile int peernum = 0;
//esp_now_peer_info_t slave;

void recv_cb(const uint8_t *macaddr, const uint8_t *data, int len)
{
  recv = true;
  //Serial.print("recv_cb ");
  //Serial.println(len); 
  if (len == RCdataSize) 
  {
    for (int i=0;i<RCdataSize;i++) RCdata.data[i] = data[i];
  }
  /*
  if (!esp_now_is_peer_exist(macaddr))
  {
    Serial.println("adding peer ");
    esp_now_add_peer(macaddr, ESP_NOW_ROLE_COMBO, WIFI_CHANNEL, NULL, 0);
    peernum++;
  }
  */
};


#define ACCRESO 4096
#define CYCLETIME 4
#define MINTHROTTLE 1090
#define MIDRUD 1517
#define THRCORR 19

enum ang { ROLL,PITCH,YAW };

static int16_t gyroADC[3];
static int16_t accADC[3];
static int16_t gyroData[3];
static float angle[2]    = {0,0};  
extern int calibratingA;

//flysky
#define ROL 0
#define PIT 1
#define THR 2
#define RUD 3

//orangerx
//#define ROL 1
//#define PIT 2
//#define THR 0
//#define RUD 3

#define AU1 4
#define AU2 5
static int16_t rcCommand[] = {0,0,0};

#define GYRO     0
#define STABI    1
static int8_t flightmode;
static int8_t oldflightmode;

boolean armed = false;
uint8_t armct = 0;
int debugvalue = 0;

void setup() 
{
  Serial.begin(115200); Serial.println();

  MPU6050_init();
  MPU6050_readId(); // must be 0x68, 104dec
  
  EEPROM.begin(64);
  if (EEPROM.read(63) != 0x55) Serial.println("Need to do ACC calib");
  else ACC_Read(); // eeprom is initialized
  if (EEPROM.read(62) != 0xAA) Serial.println("Need to check and write PID");
  else PID_Read(); // eeprom is initialized

  
  WiFi.mode(WIFI_STA); // Station mode for esp-now 
  #if defined webServer
    setupwebserver();
    delay(500); 
  #endif


  #if defined externRC
    init_RC();
  #else
    Serial.printf("This mac: %s, ", WiFi.macAddress().c_str()); 
    Serial.printf(", channel: %i\n", WIFI_CHANNEL); 
    if (esp_now_init() != 0) Serial.println("*** ESP_Now init failed");
    esp_now_register_recv_cb(recv_cb);
  #endif

  delay(500); 
  pinMode(LED,OUTPUT);
  digitalWrite(LED,LOW);
  initServo();
}

uint32_t rxt; // receive time, used for falisave

void loop() 
{
  uint32_t now,diff; 
  now = millis(); // actual time

  #if defined webServer
    loopwebserver();
  #endif

  if (recv)
  {
    recv = false;  
    #if !defined externRC  
      buf_to_rc();
    #endif

    if (debugvalue == 4)
    {
      Serial.printf("%4d ",rcValue[0]); 
      Serial.printf("%4d ",rcValue[1]); 
      Serial.printf("%4d ",rcValue[2]); 
      Serial.printf("%4d ",rcValue[3]); 
      Serial.printf("%4d ",rcValue[4]); 
      Serial.printf("%4d ",rcValue[5]); 
      Serial.println();
    }
  
    if      (rcValue[AU1] < 1300) flightmode = GYRO;
    else                          flightmode = STABI;   
    if (oldflightmode != flightmode)
    {
      zeroGyroAccI();
      oldflightmode = flightmode;
    }

    if (armed) 
    {
      rcValue[THR]    -= THRCORR;
      rcCommand[ROLL]  = rcValue[ROL] - MIDRUD;
      rcCommand[PITCH] = rcValue[PIT] - MIDRUD;
      rcCommand[YAW]   = rcValue[RUD] - MIDRUD;
    }  
    else
    {  
      if (rcValue[THR] < MINTHROTTLE) armct++;
      if (armct >= 25) 
      { 
        digitalWrite(LED,HIGH); 
        armed = true;
      }
    }
    
    //Serial.println(rcValue[AU2]    );
    //Serial.print(rcValue[THR]    ); Serial.print("  ");
    //Serial.print(rcCommand[ROLL] ); Serial.print("  ");
    //Serial.print(rcCommand[PITCH]); Serial.print("  ");
    //Serial.print(rcCommand[YAW]  ); Serial.println();

    //diff = now - rxt;
    //Serial.print(diff); Serial.println();
    rxt = millis();

  }

  Gyro_getADC();
  
  ACC_getADC();

  getEstimatedAttitude();

  pid();

  mix();

  writeServo();
  
  // Failsave part
  if (now > rxt+90)
  {
    rcValue[THR] = MINTHROTTLE;
    rxt = now;
    //Serial.println("FS");
  }

  // parser part
  if (Serial.available())
  {
    char ch = Serial.read();
    // Perform ACC calibration
    if (ch == 10) Serial.println();
    else if (ch == 'A')
    { 
      Serial.println("Doing ACC calib");
      calibratingA = CALSTEPS;
      while (calibratingA != 0)
      {
        delay(CYCLETIME);
        ACC_getADC(); 
      }
      ACC_Store();
      Serial.println("ACC calib Done");
    }
    else if (ch == 'R')
    {
      Serial.println("Act PID :");
      Serial.print(P_PID); Serial.print("  ");
      Serial.print(I_PID); Serial.print("  ");
      Serial.print(D_PID); Serial.println();
      Serial.print(P_Level_PID); Serial.print("  ");
      Serial.print(I_Level_PID); Serial.print("  ");
      Serial.print(D_Level_PID); Serial.println();
    }
    else if (ch == 'D')
    {
      Serial.println("Loading default PID");
      yawRate = 5.0;
      rollPitchRate = 5.0;
      P_PID = 0.15;    // P8
      I_PID = 0.00;    // I8
      D_PID = 0.08; 
      P_Level_PID = 0.75;   // P8
      I_Level_PID = 0.01;   // I8
      D_Level_PID = 0.10;
      PID_Store();
    }
    else if (ch == 'W')
    {
      Serial.println(P_PID);
      char ch = Serial.read();
      int n = Serial.available();
      if (n == 3)
      {
        n = readsernum();        
        if      (ch == 'p') { P_PID       = float(n) * 0.01 + 0.004; Serial.print("pid P ");       Serial.print(P_PID); }
        else if (ch == 'i') { I_PID       = float(n) * 0.01 + 0.004; Serial.print("pid I ");       Serial.print(I_PID); }
        else if (ch == 'd') { D_PID       = float(n) * 0.01 + 0.004; Serial.print("pid D ");       Serial.print(D_PID); }
        else if (ch == 'P') { P_Level_PID = float(n) * 0.01 + 0.004; Serial.print("pid Level P "); Serial.print(P_Level_PID); }
        else if (ch == 'I') { I_Level_PID = float(n) * 0.01 + 0.004; Serial.print("pid Level I "); Serial.print(I_Level_PID); }
        else if (ch == 'D') { D_Level_PID = float(n) * 0.01 + 0.004; Serial.print("pid Level D "); Serial.print(D_Level_PID); }
        else Serial.println("unknown command");
      }
      else if (ch == 'S') { PID_Store(); Serial.print("stored in EEPROM"); }
      else 
      {
        Serial.println("Input format wrong");
        Serial.println("Wpxx, Wixx, Wdxx - write gyro PID, example: Wd13");
        Serial.println("WPxx, WIxx, WDxx - write level PID, example: WD21");
      }
    }
    else if (ch == '0') debugvalue = 0;
    else if (ch == '1') debugvalue = 1;
    else if (ch == '2') debugvalue = 2;
    else if (ch == '3') debugvalue = 3;
    else if (ch == '4') debugvalue = 4;
    else
    {
      Serial.println("A - acc calib");
      Serial.println("D - write default PID");
      Serial.println("R - read actual PID");
      Serial.println("Wpxx, Wixx, Wdxx - write gyro PID");
      Serial.println("WPxx, WIxx, WDxx - write level PID");
      Serial.println("WS - Store PID in EEPROM");
      Serial.println("Display data:");
      Serial.println("= - off");
      Serial.println("1 - Gyro values");
      Serial.println("2 - Acc values");
      Serial.println("3 - Angle values");
      Serial.println("4 - RC values");
    }
  }

  if (debugvalue == 1)
  {
    Serial.print(gyroADC[0]); Serial.print("  ");
    Serial.print(gyroADC[1]); Serial.print("  ");
    Serial.print(gyroADC[2]); Serial.println();
  }
  else if (debugvalue == 2)
  { 
    Serial.print(accADC[0]); Serial.print("  ");
    Serial.print(accADC[1]); Serial.print("  ");
    Serial.print(accADC[2]); Serial.println();
  }
  else if (debugvalue == 3)
  { 
    Serial.print(angle[0]); Serial.print("  ");
    Serial.print(angle[1]); Serial.println();
  }
  
  delay(CYCLETIME-1);
  //diff = millis() - now;
  //Serial.print(diff); Serial.println();
}

int readsernum()
{
  int num;
  char numStr[3];  
  numStr[0] = Serial.read();
  numStr[1] = Serial.read();
  return atol(numStr);
}
