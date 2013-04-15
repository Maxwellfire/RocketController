//Read the rotary switch


void MpuRead()
{
  // Wait for MPU interrupt or extra packet(s) available
  if (MpuInterrupt || (MpuFifoCount >= MpuPacketSize))
  {
    // Reset interrupt flag and get INT_STATUS byte
    MpuInterrupt = false;
    MpuIntStatus = mpu.getIntStatus();

    // Get current FIFO count
    MpuFifoCount = mpu.getFIFOCount();

    // Check for overflow (this should never happen unless our code is too inefficient)
    if ((MpuIntStatus & 0x10) || MpuFifoCount == 1024) 
    {
      // Reset so we can continue cleanly
      mpu.resetFIFO();
      Serial.println(F("FIFO overflow!"));
    } 
    else if (MpuIntStatus & 0x02) 
    {
      // Wait for correct available data length
      //  Should be a VERY short wait
      while (MpuFifoCount < MpuPacketSize)
      {
        MpuFifoCount = mpu.getFIFOCount();
      }
      // Read a packet from FIFO
      mpu.getFIFOBytes(MpuFifoBuffer, MpuPacketSize);
      
        // Track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      MpuFifoCount -= MpuPacketSize;

      mpu.dmpGetQuaternion(&Quaternion, MpuFifoBuffer);
      mpu.dmpGetAccel(&Acceleration, MpuFifoBuffer);
      mpu.dmpGetGravity(&Gravity, &Quaternion);
      mpu.dmpGetLinearAccel(&AccelerationReal, &Acceleration, &Gravity);
      mpu.dmpGetLinearAccelInWorld(&AccelerationWorld, &AccelerationReal, &Quaternion);
      mpu.dmpGetYawPitchRoll(Ypr, &Quaternion, &Gravity);
    }
  }
}

int ReadRotary()
{
  SwitchRotary = digitalRead(PIN_D_SwitchRot4) << 3;
  SwitchRotary += digitalRead(PIN_D_SwitchRot3) << 2;
  SwitchRotary += digitalRead(PIN_D_SwitchRot2) << 1;
  SwitchRotary += digitalRead(PIN_D_SwitchRot1);

  return SwitchRotary; 
}

// I2C


boolean I2cReadWord(byte Address, byte Register, word &Value)
{
  int Result;

  Value = 0;

  // Start transmission to device 
  Wire.beginTransmission(Address);

  // Write register Register to read from
  Wire.write(Register); 

  // End transmission
  Result = Wire.endTransmission();

  if (Result == 0)
  { 
    Wire.requestFrom((uint8_t)Address, (uint8_t)2);

    if (Wire.available())
      Value = Wire.read();

    Value <<= 8;

    if (Wire.available())
      Value |= Wire.read();
  }
  else
  {
    I2cShowError(Result);
  }

  return (Result == 0);
}

boolean I2cWriteByte(byte Address, byte Register, byte Value)
{
  int Result;

  // Start transmission to device 
  Wire.beginTransmission(Address);

  // Write register Register to read from
  Wire.write(Register); 

  // Write data
  Wire.write(Value);

  // End transmission
  Result = Wire.endTransmission();

  if (Result != 0)
    I2cShowError(Result);

  return (Result == 0);
}

void I2cShowError(int Result)
{
  if (Result > 0)
  {
#if (CODE_Debug)
    Serial.print("PROBLEM..... Result code is ");
    Serial.println(Result);
#endif
  }
}

////

void ButtonDebounce(int &ButtonState, int &ButtonValue, unsigned long &ButtonTimeStart, unsigned long &TimeCurrent)
{
  // Debounce Button
  switch (ButtonState)
  {
  case 0:
    // Looking for LOW
    if (ButtonValue == LOW)
    {
      ButtonState = 1;
      ButtonTimeStart = TimeCurrent;
    }
    break;

  case 1:
    // Staying LOW
    if (ButtonValue == LOW)
    {
      if ((int)TimeCurrent-ButtonTimeStart > 500)
        ButtonState = 2;
    }
    else
    {
      ButtonState = 0;
    }
    break;

  case 2:
    // One time ON
    ButtonState = 3;
    break;

  case 3:
    // Looking for HIGH
    if (ButtonValue == HIGH)
    {
      ButtonState = 0;
    }
    break;
  }
}

void OutputSet(int &OutputState, int &OutputOn, int OutputPin, unsigned long &OutputInterval, unsigned long &OutputTimeStart, unsigned long &TimeCurrent)
{
  switch (OutputState)
  {
  case 0:
    digitalWrite(OutputPin, LOW);
    break;

  case 1:
    if (TimeCurrent-OutputTimeStart > OutputInterval)
    {
      if (OutputOn == true)
      {
        digitalWrite(OutputPin, LOW);
        OutputOn = false;
      }
      else
      {
        digitalWrite(OutputPin, HIGH);
        OutputOn = true;
      }

      OutputTimeStart = TimeCurrent;
    }
    break;

  case 2:
    digitalWrite(OutputPin, HIGH);
  }
}




