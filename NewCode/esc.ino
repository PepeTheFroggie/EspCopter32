
uint16_t servo[6];
int maxm = 1700; // for testing
#define minm 950 // for testing

void mix()
{
  /*
  Serial.print(rcValue[THR]);   Serial.print("  ");
  Serial.print(axisPID[ROLL]);  Serial.print("  ");
  Serial.print(axisPID[PITCH]); Serial.print("  ");
  Serial.print(axisPID[YAW]);   Serial.println();
  /**/ 
  
  if (armed & (rcValue[THR] > MINTHROTTLE))
  {
    int thro = rcValue[THR] - 15;
    servo[0] = constrain(thro - axisPID[ROLL] + axisPID[PITCH] - axisPID[YAW],minm,maxm);
    servo[1] = constrain(thro - axisPID[ROLL] - axisPID[PITCH] + axisPID[YAW],minm,maxm);
    servo[2] = constrain(thro + axisPID[ROLL] + axisPID[PITCH] + axisPID[YAW],minm,maxm);
    servo[3] = constrain(thro + axisPID[ROLL] - axisPID[PITCH] - axisPID[YAW],minm,maxm);
  }
  else 
  { 
    //if (!armed) Serial.println("NotArmed");
    servo[0] = 1000; servo[1] = 1000; servo[2] = 1000; servo[3] = 1000;
  }
}

//----------------------------------------------

const int MotPin0 = 32; //12;  //HR
const int MotPin1 = 33; //13;  //VR
const int MotPin2 = 25; //15;  //HL
const int MotPin3 = 26; //14;  //VL
const int MotChannel0 = 0;
const int MotChannel1 = 1;   
const int MotChannel2 = 2;
const int MotChannel3 = 3;

void writeMot() 
{
  ledcWrite(MotChannel0, servo[0]);
  ledcWrite(MotChannel1, servo[1]);
  ledcWrite(MotChannel2, servo[2]);
  ledcWrite(MotChannel3, servo[3]);
}

void initMot() 
{
  ledcSetup(MotChannel0, 500, 11); // 500 hz PWM, 11-bit resolution
  ledcSetup(MotChannel1, 500, 11); // 500 hz PWM, 11-bit resolution
  ledcSetup(MotChannel2, 500, 11); // 500 hz PWM, 11-bit resolution
  ledcSetup(MotChannel3, 500, 11); // 500 hz PWM, 11-bit resolution
  ledcAttachPin(MotPin0, MotChannel0); 
  ledcAttachPin(MotPin1, MotChannel1); 
  ledcAttachPin(MotPin2, MotChannel2); 
  ledcAttachPin(MotPin3, MotChannel3); 
  ledcWrite(MotChannel0, 1000);
  ledcWrite(MotChannel1, 1000);
  ledcWrite(MotChannel2, 1000);
  ledcWrite(MotChannel3, 1000);
}
