
#include "Constants.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Servo.h>
#include <Wire.h>
#include <SdFat.h>

MPU6050 mpu;

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
uint8_t MpuIntStatus;   // holds actual interrupt status byte from MPU
uint16_t MpuPacketSize;    // expected DMP packet size (default is 42 bytes)
uint16_t MpuFifoCount;     // count of all bytes currently in FIFO
uint8_t MpuFifoBuffer[64]; // FIFO storage buffer
volatile bool MpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

// Orientation and motion variables
Quaternion Quaternion;          // [w, x, y, z]         quaternion container
VectorInt16 Acceleration;       // [x, y, z]            accel sensor measurements
VectorInt16 AccelerationReal;   // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 AccelerationWorld;  // [x, y, z]            world-frame accelerometer sensor measurements
VectorFloat Gravity;            // [x, y, z]            gravity vector
float Euler[3];                 // [psi, theta, phi]    Euler angle container
float Ypr[3];                   // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

void MpuDmpDataReadyCallback() 
{
  MpuInterrupt = true;
}

int StateArmed = STATE_ArmedOneStageLaunchAuto;
int StateMaster = STATE_Initialize;
int StateMasterOld = -1;

Servo Servo1;
Servo Servo2;

boolean IsOkMaster = true;

// FAT File Logger
// Change the value of chipSelect if your hardware does
// not use the default va  lue, SS_PIN.  Common values are:
// Arduino Ethernet shield: pin 4
// Sparkfun SD shield: pin 8
// Adafruit SD shields and modules: pin 10
// File 
#define SS_PIN 20

SdFat SdFatLogSystem;
SdFile SdFatLogFile;

// Button
int ButtonValue = 0;
int ButtonState = 0;
unsigned long ButtonTimeStart = 0;

// Rotary Switch
byte SwitchRotary = 0;

// LED and Buzzer
int LedGreenState = 0;
int LedGreenBlink = 0;
unsigned long LedGreenTimeStart = 0;
unsigned long LedGreenInterval = 1000;

int LedRedState = 0;
int LedRedBlink = 0;
unsigned long LedRedTimeStart = 0;
unsigned long LedRedInterval = 1000;

int LogState = 0;

unsigned long TimeCurrent = 0;

// Pressure
double Temperature = 0.0;  // C
double Pressure = 0.0;     // Pa
double PressureOld = 0.0;  // Pa
double Altitude = 0.0;     // m

double AltitudePeak = -1000.0;


// Flight Summary
unsigned long TimeArmedStart = 0;
unsigned long TimeFlightStart = 0;
unsigned long TimeStageOneBurnStart = 0;
unsigned long TimeStageOneCoastStart = 0;
unsigned long TimeStageTwoBurnStart = 0;
unsigned long TimeStageTwoCoastStart = 0;
unsigned long TimeParachuteRelease = 0;
unsigned long TimeNotMoving = 0;
unsigned long TimeFlightEnd = 0;

int ParachuteReleaseHow = 0;

void setup()
{
  // Sart Communications
  Wire.begin();
  Serial.begin(115200);

  // Initialize MPU6050
  Serial.println(F("Initializing MPU6050 ..."));
  mpu.initialize();
  IsOkMaster = mpu.testConnection();

  if (IsOkMaster)
  {
    uint8_t MpuDevStatus;      // return status after each device operation (0 = success, !0 = error)

    Serial.println(F("   MPU6050: connection successful"));

    // Load and configure the DMP
    Serial.println(F("   MPU6050: Initializing DMP ..."));
    MpuDevStatus = mpu.dmpInitialize();
    if (MpuDevStatus == 0) 
    {
      // Turn on the DMP, now that it's ready
      Serial.println(F("   MPU6050: Enabling DMP..."));
      mpu.setDMPEnabled(true);

      // Enable Arduino interrupt detection
      Serial.println(F("   MPU6050: Enabling interrupt detection (Arduino external interrupt 0)..."));
      attachInterrupt(0, MpuDmpDataReadyCallback, RISING);
      MpuIntStatus = mpu.getIntStatus();

      // Get expected DMP packet size for later comparison
      MpuPacketSize = mpu.dmpGetFIFOPacketSize();

      Serial.println(F("   MPU6050: DMP ready"));
    } 
    else 
    {
      IsOkMaster = false;

      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("   MPU6050: DMP Initialization failed (code "));
      Serial.print(MpuDevStatus);
      Serial.println(F(")"));
    }
  }
  else
  {
    Serial.println(F("   MPU6050: connection failed"));
  }

  // Configure LED for output
  pinMode(LED_PIN, OUTPUT);

  // Rotary switch pins as input
  pinMode(PIN_D_SwitchRot1, INPUT);
  pinMode(PIN_D_SwitchRot2, INPUT);
  pinMode(PIN_D_SwitchRot3, INPUT);
  pinMode(PIN_D_SwitchRot4, INPUT);

  //Leds as 
  pinMode(PIN_D_LedGreen, OUTPUT);
  digitalWrite(PIN_D_LedGreen, HIGH);
  pinMode(PIN_D_LedRedBuzzer, OUTPUT);
  digitalWrite(PIN_D_LedRedBuzzer, HIGH);

  // SPI SS pin
  pinMode(10, OUTPUT);

  //Servo Setup
  Servo1.attach(PIN_D_Servo1,600,2400);
  Servo2.attach(PIN_D_Servo2,600,2400);

  Servo1.write(SERVO_1_Position_Start);
  Servo2.write(SERVO_2_Position_Start);


#if (CODE_Debug > 0)
  Serial.println();
  Serial.println();
  Serial.println("||||||||||");
  Serial.println("");
#endif

#if (CODE_Debug > 0)
  Serial.println("***");
  Serial.println("Initializing");
  Serial.println("");
#endif

#if (CODE_Debug > 0)
  Serial.println("");
#endif

  IsOkMaster |= SdFatInitialize();
  delay(1000);

  ButtonValue = analogRead(PIN_A_Button1) > 500;
  ButtonState = 0;
  ButtonTimeStart = millis();

  LedGreenState = 0;
  LedGreenBlink = 0;
  LedGreenTimeStart = millis();

  LedRedState = 0;
  LedRedBlink = 0;
  LedRedTimeStart = millis();

#if (CODE_Debug > 0)
  Serial.println("");
  Serial.println("Initializing Complete");
  Serial.println("***");
  Serial.println("");
#endif
}

void loop()
{
  // Read sensors and buttons
  TimeCurrent = millis();   

  // Button
  ButtonValue = analogRead(PIN_A_Button1) > 500;      // read input value and store it in val
  ButtonDebounce(ButtonState, ButtonValue, ButtonTimeStart, TimeCurrent);

  // Rotary Switch
  SwitchRotary = ReadRotary();

  // Sensor Values
  MpuRead();

  switch (StateMaster)
  {
  case STATE_Initialize:
  case STATE_Final:
  case STATE_Charge:
  case STATE_Readout:
  case STATE_Error:
    AccelerationWorld.x = 0.0;
    AccelerationWorld.y = 0.0;
    AccelerationWorld.z = 0.0;
    break;

  case STATE_PreFlight:
  case STATE_ArmedOneStageLaunchManual:
  case STATE_ArmedOneStageLaunchAuto:
  case STATE_ArmedTwoStageLaunchManual:
  case STATE_IgnitionOne:
  case STATE_StageOneBurn:
  case STATE_StageOneCoast:
  case STATE_IgnitionTwo:
  case STATE_StageTwoBurn:
  case STATE_StageTwoCoast:
  case STATE_Recovery:
  case STATE_Logger:
    //Altitude = BmpPressure.CalculateAltitude(Pressure);

    if (Altitude > AltitudePeak)
      AltitudePeak = Altitude;
    break;
  }

  // Check for StateMaster change
  // Button
  switch(StateMaster)
  {
  case STATE_StageOneCoast:
  case STATE_StageOneBurn:
  case STATE_StageTwoCoast:
  case STATE_StageTwoBurn:
  case STATE_Recovery:
  case STATE_Logger:
    if (ButtonState == 2)
    {
      StateMaster = STATE_Final;
    }
    break;
  }

  // Acceleration and Time
  switch (StateMaster)
  {
  case STATE_Initialize:
    if (IsOkMaster)
    {
      switch (SwitchRotary)
      {
      default:
        if (SdFatOpenWrite() == true)
        {
          StateMaster = STATE_PreFlight;
        }
        else
        {
          StateMaster = STATE_Error;
        }
        break;

      case 12:
        StateMaster = STATE_Charge;
        break;

      case 15:
        StateMaster = STATE_Readout;
        break;
      }
    }
    else
    {
      StateMaster = STATE_Error;
    }
    break;

  case STATE_PreFlight:
    if (ButtonState == 2)
    {
      switch (SwitchRotary)
      {
      case 0:
        StateMaster = STATE_IgnitionTwo;
        break;

      case 1:
        StateArmed = STATE_ArmedOneStageLaunchManual;
        StateMaster = STATE_ArmedOneStageLaunchManual;
        break;

      case 2:
        StateArmed = STATE_ArmedOneStageLaunchAuto;
        StateMaster = STATE_ArmedOneStageLaunchAuto;
        break;

      case 3:
        StateArmed = STATE_ArmedTwoStageLaunchManual;
        StateMaster = STATE_ArmedTwoStageLaunchManual;
        break;

      case 14:
        StateMaster = STATE_Logger;
        break;
      }
    }
    break;

  case STATE_ArmedOneStageLaunchManual:
    StateMaster = STATE_IgnitionTwo;
    break;

  case STATE_ArmedOneStageLaunchAuto:
    if ((TimeCurrent - TimeArmedStart) > DELAY_Armed_To_LaunchAuto)
    {
      StateMaster = STATE_IgnitionTwo;

      Servo1.write(SERVO_1_Position_End);
    }
    break;

  case STATE_ArmedTwoStageLaunchManual:
    StateMaster = STATE_IgnitionOne;
    break;

  case STATE_IgnitionOne:
    if (AccelerationWorld.z < ACCELERATION_Ignition)
    {
      StateMaster = STATE_StageOneBurn;
    }
    break;

  case STATE_StageOneBurn:
    if (AccelerationWorld.z > 0.0)
    {
      StateMaster = STATE_StageOneCoast;
    }
    break;

  case STATE_StageOneCoast:
    if (TimeCurrent - TimeStageOneCoastStart >= DELAY_StageOneCoast_To_StageTwoIgnition)
    {
      StateMaster = STATE_IgnitionTwo;

      Servo1.write(SERVO_1_Position_End);
    }
    break;

  case STATE_IgnitionTwo:
    if (AccelerationWorld.z < ACCELERATION_Ignition)
    {
      StateMaster = STATE_StageTwoBurn;
    }
    break;

  case STATE_StageTwoBurn:
    if (AccelerationWorld.z > 0.0)
    {
      StateMaster = STATE_StageTwoCoast;
    }
    break;

  case STATE_StageTwoCoast:
    if (TimeCurrent - TimeStageTwoCoastStart >= DELAY_Coast_To_Parachute)
    {
      StateMaster = STATE_Recovery;

      Servo2.write(SERVO_2_Position_End);
      ParachuteReleaseHow = 1;
    }
    break;

  case STATE_Recovery:
    if (AccelerationWorld.z < 0.1)
    {
      if (TimeNotMoving == 0)
        TimeNotMoving = TimeCurrent;

      if (TimeCurrent - TimeNotMoving > DELAY_RecoveryNotMoving_To_Final)
      {
        StateMaster = STATE_Final;
      }
    }
    else
    {
      TimeNotMoving = 0;
    }
    break;

  case STATE_Final:
    break;

  case STATE_Charge:
    break;

  case STATE_Readout:
    if (ButtonState == 2)
    {
      StateMaster = STATE_Initialize;
    }
    break;

  case STATE_Logger:
    break;

  case STATE_Error:
    break;
  }

  // Altitude
  switch (StateMaster)
  {
  case STATE_Initialize:
  case STATE_PreFlight:
  case STATE_StageOneBurn:
  case STATE_StageTwoBurn:
  case STATE_Final:
  case STATE_Charge:
  case STATE_Readout:
  case STATE_Logger:
  case STATE_Error:
    break;

  case STATE_ArmedOneStageLaunchManual:
  case STATE_ArmedOneStageLaunchAuto:
  case STATE_ArmedTwoStageLaunchManual:
  case STATE_IgnitionOne:
  case STATE_StageOneCoast:
  case STATE_IgnitionTwo:
    if ((AltitudePeak - Altitude) > DEADBAND_Altitude_Flight)
    {
      StateMaster = STATE_Recovery;

      Servo2.write(SERVO_2_Position_End);
      ParachuteReleaseHow = 2;
    }
    break;

  case STATE_StageTwoCoast:
    if ((AltitudePeak - Altitude) > DEADBAND_Altitude_Final)
    {
      StateMaster = STATE_Recovery;

      Servo2.write(SERVO_2_Position_End);
      ParachuteReleaseHow = 2;
    }
    break;

  case STATE_Recovery:
    break;
  }

  // Process StateMaster
  if (StateMaster != StateMasterOld)
  {
    // First time in State

#if (CODE_Debug > 0)
    Serial.print("*** State = ");
    Serial.println(StateMaster);
    Serial.println();
#endif

    switch (StateMaster)
    {
    case STATE_Initialize:
      LedGreenState = 2;
      LedRedState = 2;

      LogState = 0;
      break;

    case STATE_PreFlight:
      LedGreenState = 1   ;
      LedGreenInterval = 1000;
      LedRedState = 0;

      LogState = 0;
      break;

    case STATE_ArmedOneStageLaunchManual:
      TimeArmedStart = TimeCurrent;

      LedGreenState = 2;
      LedRedState = 0;

      LogState = 1;
      break;

    case STATE_ArmedOneStageLaunchAuto:
      TimeArmedStart = TimeCurrent;

      LedGreenState = 2;
      LedRedState = 0;

      LogState = 1;
      break;

    case STATE_ArmedTwoStageLaunchManual:
      TimeArmedStart = TimeCurrent;

      LedGreenState = 2;
      LedRedState = 0;

      LogState = 1;
      break;

    case STATE_IgnitionOne:
      TimeFlightStart = TimeCurrent;

      LedGreenState = 2;
      LedRedState = 1;
      LedRedInterval = 1000;

      LogState = 1;
      break;

    case  STATE_StageOneBurn:
      TimeStageOneBurnStart = TimeCurrent;

      LedGreenState = 1;
      LedGreenInterval = 1000;
      LedRedState = 1;
      LedRedInterval = 1000;

      LogState = 1;
      break;

    case STATE_StageOneCoast:
      TimeStageOneCoastStart = TimeCurrent;

      LedGreenState = 1;
      LedGreenInterval = 250;
      LedRedState = 1;
      LedRedInterval = 1000;

      LogState = 1;
      break;

    case STATE_IgnitionTwo:
      switch (StateArmed)
      {
      case STATE_ArmedOneStageLaunchManual:
      case STATE_ArmedOneStageLaunchAuto:
        TimeFlightStart = TimeCurrent;
        break;

      case STATE_ArmedTwoStageLaunchManual:
        break;
      }

      LedGreenState = 2;
      LedRedState = 1;
      LedRedInterval = 250;

      LogState = 1;
      break;

    case STATE_StageTwoBurn:
      TimeStageTwoBurnStart = TimeCurrent;

      LedGreenState = 1;
      LedGreenInterval = 1000;
      LedRedState = 1;
      LedRedInterval = 250;

      LogState = 1;
      break;

    case STATE_StageTwoCoast:
      TimeStageTwoCoastStart = TimeCurrent;

      LedGreenState = 1;
      LedGreenInterval = 250;
      LedRedState = 1;
      LedRedInterval = 250;

      LogState = 1;
      break;

    case STATE_Recovery:
      TimeParachuteRelease = TimeCurrent;

      LedGreenState = 0;
      LedRedState = 2;

      LogState = 1;
      break;

    case STATE_Final:
      // Subtruct DELAY_RecoveryNotMoving_To_Finalg because it was needed to make sure rocket landed
      TimeFlightEnd = TimeCurrent - DELAY_RecoveryNotMoving_To_Final;

#if (CODE_Debug > 0)
      Serial.println("");
      Serial.println("");

      Serial.println(TimeFlightStart-TimeArmedStart);
      Serial.println(TimeStageOneBurnStart-TimeArmedStart);
      Serial.println(TimeStageOneCoastStart-TimeArmedStart);
      Serial.println(TimeStageTwoBurnStart-TimeArmedStart);
      Serial.println(TimeStageTwoCoastStart-TimeArmedStart);
      Serial.println(TimeParachuteRelease-TimeArmedStart);
      Serial.print(", ");
      Serial.println(ParachuteReleaseHow);
      Serial.println(TimeFlightEnd-TimeArmedStart);
      Serial.println();
#endif

      if (SdFatLogFile.isOpen())
      {
        SdFatLogFile.println();
        SdFatLogFile.println();

        SdFatLogFile.println(TimeFlightStart-TimeArmedStart);
        SdFatLogFile.println(TimeStageOneBurnStart-TimeArmedStart);
        SdFatLogFile.println(TimeStageOneCoastStart-TimeArmedStart);
        SdFatLogFile.println(TimeStageTwoBurnStart-TimeArmedStart);
        SdFatLogFile.println(TimeStageTwoCoastStart-TimeArmedStart);
        SdFatLogFile.print(TimeParachuteRelease-TimeArmedStart);
        SdFatLogFile.print(", ");
        SdFatLogFile.println(ParachuteReleaseHow);
        SdFatLogFile.println(TimeFlightEnd-TimeArmedStart);
        SdFatLogFile.println();

        SdFatLogFile.close();
      }

      LedGreenState = 1;
      LedGreenInterval = 100;
      LedRedState = 0;

      LogState = 0;
      break;

    case STATE_Charge:
      Servo1.detach();
      Servo2.detach();

      LedGreenState = 0;
      LedRedState = 0;

      LogState = 0;
      break;

    case STATE_Readout:
      LedGreenState = 2;
      LedRedState = 2;

      LogState = 0;

      {
        int FileLogNumberLast;

        FileLogNumberLast = SdFatReadFileLogNumberLast();
        if (FileLogNumberLast >= 0)
        {
          Serial.print("Last Log File Number: ");
          Serial.println(FileLogNumberLast);
          Serial.println("");
        }

        Serial.print("Enter File Number: ");
      }
      break;

    case STATE_Logger:
      LedGreenState = 1;
      LedGreenInterval = 2000;
      LedRedState = 1;
      LedRedInterval = 2000;

      LogState = 1;
      break;

    case STATE_Error:
      LedGreenState = 1;
      LedGreenInterval = 100;
      LedRedState = 1;
      LedRedInterval = 100;

      LogState = 0;  

      Servo2.write(SERVO_2_Position_End);
      ParachuteReleaseHow = 10;

      if (SdFatLogFile.isOpen())
      {
        SdFatLogFile.close();
      }
      break;
    }
  }
  else
  {
    // Every time in State

      switch (StateMaster)
    {
    case STATE_Initialize:
    case STATE_PreFlight:
    case STATE_ArmedOneStageLaunchManual:
    case STATE_ArmedOneStageLaunchAuto:
    case STATE_ArmedTwoStageLaunchManual:
    case STATE_IgnitionOne:
    case STATE_StageOneCoast:
    case STATE_StageOneBurn:
    case STATE_StageTwoCoast:
    case STATE_StageTwoBurn:
    case STATE_Recovery:
    case STATE_Logger:
    case STATE_Final:
    case STATE_Charge:
      break;

    case STATE_Readout:
      if (Serial.available() > 0)
      {
        int FileLogNumber;

        FileLogNumber = Serial.read() - (char)'0';

        if ((FileLogNumber >= 1) && (FileLogNumber <= 9))
        {
          SdFatReadBack(FileLogNumber);
        }

        Serial.print("Enter File Number: ");
      }
      break;

    case STATE_Error:
      break;
    }
  }

  // Output Light and Buzzer
  LedSet(LedGreenState, LedGreenBlink, PIN_D_LedGreen, LedGreenInterval, LedGreenTimeStart, TimeCurrent);
  LedSet(LedRedState, LedRedBlink, PIN_D_LedRedBuzzer, LedRedInterval, LedRedTimeStart, TimeCurrent);

  // Write to storage
  if (LogState == 1)
  {
#if (CODE_Debug > 0)
    Serial.print("T=");
    Serial.print(TimeCurrent-TimeArmedStart);
    Serial.print("\tS=");
    Serial.print(StateMaster);
    Serial.print("\tAx=");
    Serial.print(AccelerationWorld.x, 2);
    Serial.print("\tAy=");
    Serial.print(AccelerationWorld.y, 2);
    Serial.print("\tAz=");
    Serial.print(AccelerationWorld.z, 2);
    Serial.print("\tYaw=");
    Serial.print(Ypr[0], 2);
    Serial.print("\tPitch=");
    Serial.print(Ypr[1], 2);
    Serial.print("\tRoll=");
    Serial.print(Ypr[2], 2);
    Serial.print("\tT=");
    Serial.print(Temperature);
    Serial.print("\tP=");
    Serial.print(Pressure);
    Serial.print("\tH=");
    Serial.print(Altitude, 2);
    Serial.println("");

#endif

    SdFatLogFile.print(TimeCurrent-TimeArmedStart);
    SdFatLogFile.print("\t");
    SdFatLogFile.print(StateMaster);
    SdFatLogFile.print("\t");
    SdFatLogFile.print(AccelerationWorld.x, 5);
    SdFatLogFile.print("\t");
    SdFatLogFile.print(AccelerationWorld.y, 5);
    SdFatLogFile.print("\t");
    SdFatLogFile.print(AccelerationWorld.z, 5);
    SdFatLogFile.print("\t");
    SdFatLogFile.print(Ypr[0], 5);
    SdFatLogFile.print("\t");
    SdFatLogFile.print(Ypr[1], 5);
    SdFatLogFile.print("\t");
    SdFatLogFile.print(Ypr[2], 5);
    SdFatLogFile.print("\t");
    SdFatLogFile.print(Temperature);
    SdFatLogFile.print("\t");
    SdFatLogFile.print(Pressure);
    SdFatLogFile.print("\t");
    SdFatLogFile.print(Altitude, 5);
    SdFatLogFile.print("\t");
    SdFatLogFile.print("\r\n");
  }
  // Remember latest StateMaster
  StateMasterOld = StateMaster;
}















