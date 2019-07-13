
typedef union int16_ty
{
  int16_t d;
  byte    b[2];
};
typedef union float_ty
{
  float d;
  byte  b[4];
};

void write_int16(int pos, int16_t d)
{
  int16_ty loc;
  loc.d = d;
  EEPROM.write(pos++,loc.b[0]);
  EEPROM.write(pos++,loc.b[1]);
}
int16_t read_int16(int pos)
{
  int16_ty loc;
  loc.b[0] = EEPROM.read(pos++);
  loc.b[1] = EEPROM.read(pos++);
  return loc.d;
}

void write_float(int pos, float d)
{
  float_ty loc;
  loc.d = d;
  EEPROM.write(pos++,loc.b[0]);
  EEPROM.write(pos++,loc.b[1]);
  EEPROM.write(pos++,loc.b[2]);
  EEPROM.write(pos++,loc.b[3]);
}
float read_float(int pos)
{
  float_ty loc;
  loc.b[0] = EEPROM.read(pos++);
  loc.b[1] = EEPROM.read(pos++);
  loc.b[2] = EEPROM.read(pos++);
  loc.b[3] = EEPROM.read(pos++);
  return loc.d;
}

void commit()
{
  #if defined externRC
    end_RC();
    EEPROM.commit();
    delay(100);
    init_RC();
  #else
    EEPROM.commit();
   #endif
}

void ACC_Read()
{
  accZero[0] = read_int16(0);
  accZero[1] = read_int16(2);
  accZero[2] = read_int16(4);
}
void ACC_Store()
{
  write_int16(0, accZero[0]);
  write_int16(2, accZero[1]);
  write_int16(4, accZero[2]);
  EEPROM.write(63, 0x55);
  commit();
}

void PID_Read()
{
  yawRate       = read_float(10);
  rollPitchRate = read_float(14);
  P_PID         = read_float(18);
  I_PID         = read_float(22);
  D_PID         = read_float(26);
  P_Level_PID   = read_float(30);
  I_Level_PID   = read_float(34);
  D_Level_PID   = read_float(38);
}
void PID_Store()
{
  write_float(10,yawRate);
  write_float(14,rollPitchRate);
  write_float(18,P_PID);
  write_float(22,I_PID);
  write_float(26,D_PID);
  write_float(30,P_Level_PID);
  write_float(34,I_Level_PID);
  write_float(38,D_Level_PID);
  EEPROM.write(62, 0xAA);
  commit();
}
