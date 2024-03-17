
float Kp_rate = 0.19;    //P-gain - rate mode
float Ki_rate = 0.00;    //I-gain - rate mode
float Kd_rate = 0.008;   //D-gain - rate mode 
float Kp_yaw  = 0.15;    //Yaw P-gain
float Ki_yaw  = 0.00;    //Yaw I-gain
float Kd_yaw  = 0.00;    //Yaw D-gain 

float Kp_ang  = 1.10;    //Ang P-gain
float Ki_ang  = 0.01;    //Ang I-gain
float Kd_ang  = 0.01;    //Ang D-gain
float Kp_ayw  = 0.30;    //Ang Yaw P-gain
float Ki_ayw  = 0.01;    //Ang Yaw I-gain
float Kd_ayw  = 0.10;    //Ang Yaw D-gain

float thro_des, roll_des, pitch_des, yaw_des; // RC input 0 to 1

float error_roll, integral_roll, integral_roll_prev, derivative_roll;
float error_pitch,integral_pitch,integral_pitch_prev,derivative_pitch;
float error_yaw,  integral_yaw,  integral_yaw_prev,  derivative_yaw;
float error_roll_prev, error_pitch_prev, error_yaw_prev;
float GyroX_prev, GyroY_prev, GyroZ_prev;
float integral_ang_roll,  integral_ang_roll_prev;
float integral_ang_pitch, integral_ang_pitch_prev;
float integral_ang_yaw,   integral_ang_yaw_prev;

float roll_PID = 0;
float pitch_PID = 0;
float yaw_PID = 0;

float i_limit = 5.0; 
extern int16_t rcValue[];

void controlANG() 
{
  // deg err
  error_roll  = roll_des  - roll_IMU; 
  integral_ang_roll = integral_ang_roll_prev + error_roll*dt;
  if (rcValue[THR] < MINTHROTTLE) integral_ang_roll = 0;
  integral_ang_roll = constrain(integral_ang_roll, -i_limit, i_limit); 
  roll_des  = Kp_ang * error_roll;
//roll_des += Ki_ang * integral_ang_roll; 
  roll_des -= Kd_ang * GyroX;
  integral_ang_roll_prev = integral_ang_roll;
  
  error_pitch = pitch_des - pitch_IMU;
  integral_ang_pitch = integral_ang_pitch_prev + error_pitch*dt;
  if (rcValue[THR] < MINTHROTTLE) integral_ang_pitch = 0;
  integral_ang_pitch = constrain(integral_ang_pitch, -i_limit, i_limit); 
  pitch_des =  Kp_ang * error_pitch; 
//pitch_des += Ki_ang * integral_ang_pitch; 
  pitch_des -= Kd_ang * GyroY;
  integral_ang_pitch_prev = integral_ang_pitch;

  error_yaw = yaw_des - yaw_IMU;
  integral_ang_yaw = integral_ang_yaw_prev + error_yaw*dt;
  if (rcValue[THR] < MINTHROTTLE) integral_ang_yaw = 0;
  integral_ang_pitch = constrain(integral_ang_yaw, -i_limit, i_limit); 
  yaw_des =  Kp_ayw * error_yaw; 
//yaw_des += Ki_ayw * integral_ang_yaw; 
  yaw_des -= Kd_ayw * GyroZ;  
  integral_ang_yaw_prev = integral_ang_yaw;
  
  if (debugvalue == 'p')
  {
    Serial.print(error_yaw);        Serial.print("  ");
    //Serial.print(integral_ang_yaw); Serial.print("  ");
    Serial.print(GyroZ);   Serial.print("  ");
    Serial.print(yaw_des); Serial.println();     
  }
}

void controlRATE() 
{
  //Roll
  error_roll = roll_des - GyroX;
  integral_roll = integral_roll_prev + error_roll*dt;
  if (rcValue[THR] < MINTHROTTLE) integral_roll = 0;
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_roll = (error_roll - error_roll_prev)/dt; 
  //derivative_roll = (GyroX_prev - GyroX)/dt; 
  roll_PID  = Kp_rate*error_roll;
  roll_PID += Ki_rate*integral_roll; 
  roll_PID += Kd_rate*derivative_roll;
  integral_roll_prev = integral_roll;
  error_roll_prev = error_roll;
  GyroX_prev = GyroX;
  
  //Pitch
  error_pitch = pitch_des - GyroY;
  integral_pitch = integral_pitch_prev + error_pitch*dt;
  if (rcValue[THR] < MINTHROTTLE) integral_pitch = 0;
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_pitch = (error_pitch - error_pitch_prev)/dt; 
  //derivative_pitch = (GyroY_prev - GyroY)/dt; 
  pitch_PID  = Kp_rate*error_pitch;
  pitch_PID += Ki_rate*integral_pitch; 
  pitch_PID += Kd_rate*derivative_pitch;
  integral_pitch_prev = integral_pitch;
  error_pitch_prev = error_pitch;
  GyroY_prev = GyroY;

  //Yaw, stablize on rate from GyroZ
  error_yaw = yaw_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw*dt;
  if (rcValue[THR] < MINTHROTTLE) integral_yaw = 0;
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  yaw_PID = (Kp_yaw*error_yaw + Ki_yaw*integral_yaw );
  derivative_yaw = (error_yaw - error_yaw_prev)/dt; 
  //derivative_yaw = (GyroZ_prev - GyroZ)/dt; 
  yaw_PID  = Kp_yaw*error_yaw;
  yaw_PID += Ki_yaw*integral_yaw; 
  yaw_PID += Kd_yaw*derivative_yaw;
  integral_yaw_prev = integral_yaw;
  error_yaw_prev = error_yaw;
  GyroZ_prev = GyroZ;
}

void readpid()
{
  Kp_rate = EEPROM.readFloat( 8);
  Ki_rate = EEPROM.readFloat(12);
  Kd_rate = EEPROM.readFloat(16);
  Kp_yaw  = EEPROM.readFloat(20);
  Ki_yaw  = EEPROM.readFloat(24);
  Kd_yaw  = EEPROM.readFloat(28);
  Kp_ang  = EEPROM.readFloat(32);
  Ki_ang  = EEPROM.readFloat(36);
  Kd_ang  = EEPROM.readFloat(40);
}
void storepid()
{
  EEPROM.writeFloat( 8, Kp_rate);
  EEPROM.writeShort(12, Ki_rate);
  EEPROM.writeShort(16, Kd_rate);
  EEPROM.writeFloat(20, Kp_yaw);
  EEPROM.writeShort(24, Ki_yaw);
  EEPROM.writeShort(28, Kd_yaw);
  EEPROM.writeFloat(32, Kp_ang);
  EEPROM.writeShort(36, Ki_ang);
  EEPROM.writeShort(40, Kd_ang);
  EEPROM.commit();      
}
