
#if defined externRC
//------------------------------------------------
  #define PPMIN_CHANNELS 6  // dont raise this

  volatile uint32_t last = 0;
  volatile uint8_t  chan = 0;

  IRAM_ATTR void rxInt() 
  {
    uint32_t now,diff; 
    if (!gpio_get_level(RC_IN_GPIO)) return; // because edge triggering on esp32 doesent work yet
    
    now = micros();
    diff = now - last;
    last = now;

    if      (diff > 3000) { chan = 0; } // Sync gap
    else if (chan < CHANNELS)
    {
      if (950<diff && diff<2050)
      {
        rcValue[chan] = diff;
        chan++;
      }
      else chan = CHANNELS; // skip, corrupted signal.
    }
    if (chan == PPMIN_CHANNELS) recv = true;
  }
  
  void init_RC()
  {
    pinMode(RC_IN_PIN,INPUT);
    attachInterrupt(RC_IN_PIN,rxInt,CHANGE);
  }
  void end_RC()
  {
    detachInterrupt(RC_IN_PIN);
  }
  
//------------------------------------------------
#endif

void buf_to_rc()
{
  uint8_t seq;
  rcValue[0] = RCdata.chans.Ch1;
  rcValue[1] = RCdata.chans.Ch2;
  rcValue[2] = RCdata.chans.Ch3;
  rcValue[3] = RCdata.chans.Ch4;
  rcValue[4] = RCdata.chans.Ch5;
  rcValue[5] = RCdata.chans.Ch6;
  rcValue[6] = RCdata.chans.Ch7;
  rcValue[7] = RCdata.chans.Ch8;
  seqno = RCdata.chans.spare;
}

static uint16_t servo[4];

void mix()
{
  if (armed & (rcValue[THR] > MINTHROTTLE))
  {
    servo[0] = constrain(rcValue[THR] - axisPID[ROLL] + axisPID[PITCH] - axisPID[YAW],1000,2000);
    servo[1] = constrain(rcValue[THR] - axisPID[ROLL] - axisPID[PITCH] + axisPID[YAW],1000,2000);
    servo[2] = constrain(rcValue[THR] + axisPID[ROLL] + axisPID[PITCH] + axisPID[YAW],1000,2000);
    servo[3] = constrain(rcValue[THR] + axisPID[ROLL] - axisPID[PITCH] - axisPID[YAW],1000,2000);
  }
  else 
  { 
    axisPID[0] = 0; axisPID[1] = 0; axisPID[2] = 0;
    servo[0] = 1000; servo[1] = 1000; servo[2] = 1000; servo[3] = 1000;
  }
  /*
  Serial.print(servo[0]); Serial.print("  ");
  Serial.print(servo[1]); Serial.print("  ");
  Serial.print(servo[2]); Serial.print("  ");
  Serial.print(servo[3]); Serial.println();
  */
}


#if defined PWMOUT //----------------------------------------------

const int MotPin0 = 32;  //HR
const int MotPin1 = 33;  //VR
const int MotPin2 = 25;  //HL
const int MotPin3 = 26;  //VL
const int MotChannel0 = 0;
const int MotChannel1 = 1;   
const int MotChannel2 = 2;
const int MotChannel3 = 3;

void writeServo() 
{
  ledcWrite(MotChannel0, servo[0]);
  ledcWrite(MotChannel1, servo[1]);
  ledcWrite(MotChannel2, servo[2]);
  ledcWrite(MotChannel3, servo[3]);
}

void initServo() 
{
  ledcSetup(MotChannel0, 500, 11); // 500 hz PWM, 11-bit resolution
  ledcSetup(MotChannel1, 500, 11); // 500 hz PWM, 11-bit resolution
  ledcSetup(MotChannel2, 500, 11); // 500 hz PWM, 11-bit resolution
  ledcSetup(MotChannel3, 500, 11); // 500 hz PWM, 11-bit resolution
  ledcAttachPin(MotPin0, MotChannel0); 
  ledcAttachPin(MotPin1, MotChannel1); 
  ledcAttachPin(MotPin2, MotChannel2); 
  ledcAttachPin(MotPin3, MotChannel3); 
}

#else //----------------------------------------------

uint8_t outmsg[5];

void writeServo()
{
  outmsg[0] = 0xF5;
  outmsg[1] = constrain((servo[0]-1000)>>2,0,0xF4);
  outmsg[2] = constrain((servo[1]-1000)>>2,0,0xF4);
  outmsg[3] = constrain((servo[2]-1000)>>2,0,0xF4);
  outmsg[4] = constrain((servo[3]-1000)>>2,0,0xF4);
  Serial1.write(outmsg,5);
}

void initServo()
{
  Serial1.begin(128000);
}

#endif //----------------------------------------------
