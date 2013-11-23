#include "Constants.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "HMC5883L.h"
#include <Servo.h>
#include <Wire.h>
#include <SdFat.h>

MPU6050 mpu;
HMC5883L mag;

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

//Magnetometer variables
int16_t MagnetometerX, MagnetometerY, MagnetometerZ;

void MpuDmpDataReadyCallback() 
{
  MpuInterrupt = true;
}

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
int LedTeensyState = 0;
int LedTeensyBlink = 0;
unsigned long LedTeensyTimeStart = 0;
unsigned long LedTeensyInterval = DELAY_Flash_Slow;

int LedFlightState = 0;
int LedFlightBlink = 0;
unsigned long LedFlightTimeStart = 0;
unsigned long LedFlightInterval = DELAY_Flash_Slow;

int LedStageState = 0;
int LedStageBlink = 0;
unsigned long LedStageTimeStart = 0;
unsigned long LedStageInterval = DELAY_Flash_Slow;

int LedMotorState = 0;
int LedMotorBlink = 0;
unsigned long LedMotorTimeStart = 0;
unsigned long LedMotorInterval = DELAY_Flash_Slow;

int BuzzerState = 0;
int BuzzerBlink = 0;
unsigned long BuzzerTimeStart = 0;
unsigned long BuzzerInterval = DELAY_Flash_Slow;

int LogState = 0;

unsigned long TimeCurrent = 0;
unsigned long TimeLastStateTransition = 0;

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
  Serial.println(F("Initializing MPU6050..."));
  mpu.initialize();
  IsOkMaster = mpu.testConnection();

  if (IsOkMaster)
  {
    uint8_t MpuDevStatus;      // return status after each device operation (0 = success, !0 = error)
    Serial.println(F("   MPU6050: connection successful"));

    // Load and configure the DMP
    Serial.println(F("   MPU6050: Initializing DMP..."));
    MpuDevStatus = mpu.dmpInitialize();
    if (MpuDevStatus == 0) 
    {
      // Turn on the DMP, now that it's ready
      Serial.println(F("   MPU6050: Enabling DMP..."));
      mpu.setDMPEnabled(true);

      // Enable Arduino interrupt detection
      Serial.println(F("   MPU6050: Enabling interrupt detection (Arduino external interrupt 0)..."));
      attachInterrupt(6, MpuDmpDataReadyCallback, RISING);
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

  if (IsOkMaster)
  {
    Serial.println("Initializing HMC5883L...");
    mag.initialize();

    if(mag.testConnection() == true)
    {
      Serial.println("   HMC5883L: connection successful");
    }
    else
    {
      IsOkMaster = false;
      Serial.println("   HMC5883L: connection failed");
    }
  }

  // Configure LED for output
  pinMode(LED_PIN, OUTPUT);

  // Rotary switch pins as input
  pinMode(PIN_D_SwitchRot1, INPUT);
  pinMode(PIN_D_SwitchRot2, INPUT);
  pinMode(PIN_D_SwitchRot3, INPUT);
  pinMode(PIN_D_SwitchRot4, INPUT);

  //Leds as 
  pinMode(PIN_D_LedTeensy, OUTPUT);
  digitalWrite(PIN_D_LedTeensy, LOW);
  pinMode(PIN_D_LedFlight, OUTPUT);
  digitalWrite(PIN_D_LedFlight, LOW);
  pinMode(PIN_D_LedStage, OUTPUT);
  digitalWrite(PIN_D_LedStage, LOW);
  pinMode(PIN_D_LedMotor, OUTPUT);
  digitalWrite(PIN_D_LedMotor, LOW);
  pinMode(PIN_D_Buzzer, OUTPUT);
  digitalWrite(PIN_D_Buzzer, LOW);

  // SPI SS pin
  pinMode(10, OUTPUT);

  //Servo Setup
  Servo1.attach(PIN_D_Servo1);
  Servo2.attach(PIN_D_Servo2);

  Servo1.write(SERVO_1_Position_Start);
  Servo2.write(SERVO_2_Position_Start);

  Serial.println("Initializing SdFat...");

  IsOkMaster |= SdFatInitialize();
  delay(1000);

  ButtonValue = digitalRead(PIN_D_Button1);
  ButtonTimeStart = millis();

  LedTeensyTimeStart = millis();
  LedFlightTimeStart = millis();
  LedStageTimeStart = millis();
  LedMotorTimeStart = millis();
  BuzzerTimeStart = millis();

  Serial.println("");
  Serial.println("INITIALIZING COMPLETE");
  Serial.println("_________________________");
  Serial.println("");

  // Time
  TimeCurrent = millis();   
  TimeLastStateTransition = TimeCurrent;   
}

void loop()
{
  // Read sensors and buttons
  TimeCurrent = millis();
#if (CODE_Test <= 0)
  TimeLastStateTransition = millis();
#endif

  // Button
  ButtonValue = digitalRead(PIN_D_Button1);      // read input value and store it in val
  ButtonDebounce(ButtonState, ButtonValue, ButtonTimeStart, TimeCurrent);

  // Rotary Switch
  SwitchRotary = ReadRotary();

  // Sensor Values
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
  case STATE_ArmedStageTwoLaunchAuto:
  case STATE_FlightOneIgnitionWait:
  case STATE_FlightOneBurn:
  case STATE_FlightOneCoast:
  case STATE_FlightTwoIgnitionWaitFromArmed:
  case STATE_FlightTwoIgnitionWaitFromOne:
  case STATE_FlightTwoBurn:
  case STATE_FlightTwoCoast:
  case STATE_Recovery:
  case STATE_Logger:
    //MpuRead();
    mag.getHeading(&MagnetometerX, &MagnetometerY, &MagnetometerZ);

    if (Altitude > AltitudePeak)
      AltitudePeak = Altitude;
    break;
  }

  // Button
  switch(StateMaster)
  {
  case STATE_FlightOneCoast:
  case STATE_FlightOneBurn:
  case STATE_FlightTwoCoast:
  case STATE_FlightTwoBurn:
  case STATE_Recovery:
  case STATE_Logger:
    if (ButtonState == 2)
    {
      StateMaster = STATE_Final;
    }
    break;
  }

  // Main Switch
  //  State Transitions
  //  Servo
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
          IsOkMaster = false;
          StateMaster = STATE_Error;
        }
        break;

      case 13:
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
      case 0:    // Manually Lanch Stage Two (One Stage Rocket)
        TimeArmedStart = TimeCurrent;
        StateMaster = STATE_FlightTwoIgnitionWaitFromArmed;
        break;

      case 1:    // Automatically Lanch Stage Two (One Stage Rocket)
        TimeArmedStart = TimeCurrent;
        StateMaster = STATE_ArmedStageTwoLaunchAuto;
        break;

      case 2:    // Manually Lanch Stage One (Two Stage Rocket)
        TimeArmedStart = TimeCurrent;
        StateMaster = STATE_FlightOneIgnitionWait;
        break;

      case 14:  // Log Sensors
        TimeArmedStart = TimeCurrent;
        StateMaster = STATE_Logger;
        break;
      }
    }
    break;

  case STATE_ArmedStageTwoLaunchAuto:
    if ((TimeCurrent - TimeArmedStart) > DELAY_Armed_To_LaunchAuto)
    {
      StateMaster = STATE_FlightTwoIgnitionWaitFromArmed;

      Servo1.write(SERVO_1_Position_End);
    }
    break;

  case STATE_FlightOneIgnitionWait:
    if ((AccelerationWorld.z < ACCELERATION_Ignition) ||
      (TimeCurrent-TimeLastStateTransition > CODE_TestTime))
    {
      StateMaster = STATE_FlightOneBurn;
    }
    break;

  case STATE_FlightOneBurn:
    if ((AccelerationWorld.z > 0.0) ||
      (TimeCurrent-TimeLastStateTransition > CODE_TestTime))
    {
      StateMaster = STATE_FlightOneCoast;
    }
    break;

  case STATE_FlightOneCoast:
    if ((TimeCurrent - TimeStageOneCoastStart >= DELAY_StageOneCoast_To_StageTwoIgnition) ||
      (TimeCurrent-TimeLastStateTransition > CODE_TestTime))
    {
      StateMaster = STATE_FlightTwoIgnitionWaitFromOne;

      Servo1.write(SERVO_1_Position_End);
    }
    break;

  case STATE_FlightTwoIgnitionWaitFromArmed:
  case STATE_FlightTwoIgnitionWaitFromOne:
    if ((AccelerationWorld.z < ACCELERATION_Ignition) ||
      (TimeCurrent-TimeLastStateTransition > CODE_TestTime))
    {
      StateMaster = STATE_FlightTwoBurn;
    }
    break;

  case STATE_FlightTwoBurn:
    if ((AccelerationWorld.z > 0.0) ||
      (TimeCurrent-TimeLastStateTransition > CODE_TestTime))
    {
      StateMaster = STATE_FlightTwoCoast;
    }
    break;

  case STATE_FlightTwoCoast:
    if ((TimeCurrent - TimeStageTwoCoastStart >= DELAY_Coast_To_Parachute) ||
      (TimeCurrent-TimeLastStateTransition > CODE_TestTime))
    {
      StateMaster = STATE_Recovery;

      Servo2.write(SERVO_2_Position_End);
      ParachuteReleaseHow = 1;
    }
    break;

  case STATE_Recovery:
    if ((AccelerationWorld.z < 0.1 && AccelerationWorld.z > -0.1) ||
      (TimeCurrent-TimeLastStateTransition > CODE_TestTime))
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
  case STATE_Recovery:
  case STATE_Final:
  case STATE_Charge:
  case STATE_Readout:
  case STATE_Logger:
  case STATE_Error:
    break;

  case STATE_ArmedStageTwoLaunchAuto:
  case STATE_FlightOneIgnitionWait:
  case STATE_FlightOneBurn:
  case STATE_FlightOneCoast:
  case STATE_FlightTwoIgnitionWaitFromArmed:
  case STATE_FlightTwoIgnitionWaitFromOne:
  case STATE_FlightTwoBurn:
  case STATE_FlightTwoCoast:
    if ((AltitudePeak - Altitude) > DEADBAND_Altitude_Flight)
    {
      StateMaster = STATE_Recovery;

      Servo2.write(SERVO_2_Position_End);

      ParachuteReleaseHow = 2;
    }
    break;
  }

  // First time in State
  if (StateMaster != StateMasterOld)
  {
#if (CODE_Debug > 0)
    Serial.print("*** State = ");
    Serial.println(StateMaster);
    Serial.println();
#endif

    TimeLastStateTransition = TimeCurrent;

    switch (StateMaster)
    {
    case STATE_Initialize:  
      LedTeensyState = 0;
      LedTeensyInterval = 0;
      LedFlightState = 0;
      LedFlightInterval = 0;
      LedStageState = 0;
      LedStageInterval = 0;
      LedMotorState = 0;
      LedMotorInterval = 0;
      BuzzerState = 0;
      BuzzerInterval = 0;

      LogState = 0;
      break;

    case STATE_PreFlight:
      LedTeensyState = 2;
      LedTeensyInterval = 0;
      LedFlightState = 1;
      LedFlightInterval = DELAY_Flash_Fast;
      LedStageState = 0;
      LedStageInterval = 0;
      LedMotorState = 0;
      LedMotorInterval = 0;
      BuzzerState = 0;
      BuzzerInterval = 0;

      LogState = 0;
      break;

    case STATE_ArmedStageTwoLaunchAuto:
      LedTeensyState = 2;
      LedTeensyInterval = 0;
      LedFlightState = 2;
      LedFlightInterval = 0;
      LedStageState = 0;
      LedStageInterval = 0;
      LedMotorState = 0;
      LedMotorInterval = 0;
      BuzzerState = 1;
      BuzzerInterval = DELAY_Flash_Fast;

      LogState = 1;
      break;

    case STATE_FlightOneIgnitionWait:
      TimeFlightStart = TimeCurrent;

      LedTeensyState = 2;
      LedTeensyInterval = 0;
      LedFlightState = 2;
      LedFlightInterval = 0;
      LedStageState = 1;
      LedStageInterval = DELAY_Flash_Fast;
      LedMotorState = 1;
      LedMotorInterval = DELAY_Flash_Fast;
      BuzzerState = 1;
      BuzzerInterval = DELAY_Flash_Fast;

      LogState = 1;
      break;

    case  STATE_FlightOneBurn:
      TimeStageOneBurnStart = TimeCurrent;

      LedTeensyState = 2;
      LedTeensyInterval = 0;
      LedFlightState = 2;
      LedFlightInterval = 0;
      LedStageState = 1;
      LedStageInterval = DELAY_Flash_Fast;
      LedMotorState = 2;
      LedMotorInterval = 0;
      BuzzerState = 0;
      BuzzerInterval = 0;

      LogState = 1;
      break;

    case STATE_FlightOneCoast:
      TimeStageOneCoastStart = TimeCurrent;

      LedTeensyState = 2;
      LedTeensyInterval = 0;
      LedFlightState = 2;
      LedFlightInterval = 0;
      LedStageState = 1;
      LedStageInterval = DELAY_Flash_Fast;
      LedMotorState = 0;
      LedMotorInterval = 0;
      BuzzerState = 0;
      BuzzerInterval = 0;

      LogState = 1;
      break;

    case STATE_FlightTwoIgnitionWaitFromArmed:
      TimeFlightStart = TimeCurrent;

      LedTeensyState = 2;
      LedTeensyInterval = 0;
      LedFlightState = 2;
      LedFlightInterval = 0;
      LedStageState = 2;
      LedStageInterval = 0;
      LedMotorState = 1;
      LedMotorInterval = DELAY_Flash_Fast;
      BuzzerState = 1;
      BuzzerInterval = DELAY_Flash_Fast;

      LogState = 1;
      break;

    case STATE_FlightTwoIgnitionWaitFromOne:
      LedTeensyState = 2;
      LedTeensyInterval = 0;
      LedFlightState = 2;
      LedFlightInterval = 0;
      LedStageState = 2;
      LedStageInterval = 0;
      LedMotorState = 1;
      LedMotorInterval = DELAY_Flash_Fast;
      BuzzerState = 0;
      BuzzerInterval = 0;

      LogState = 1;
      break;

    case STATE_FlightTwoBurn:
      TimeStageTwoBurnStart = TimeCurrent;

      LedTeensyState = 2;
      LedTeensyInterval = 0;
      LedFlightState = 2;
      LedFlightInterval = 0;
      LedStageState = 2;
      LedStageInterval = 0;
      LedMotorState = 2;
      LedMotorInterval = 0;
      BuzzerState = 0;
      BuzzerInterval = 0;

      LogState = 1;
      break;

    case STATE_FlightTwoCoast:
      TimeStageTwoCoastStart = TimeCurrent;

      LedTeensyState = 2;
      LedTeensyInterval = 0;
      LedFlightState = 2;
      LedFlightInterval = 0;
      LedStageState = 2;
      LedStageInterval = 0;
      LedMotorState = 0;
      LedMotorInterval = 0;
      BuzzerState = 0;
      BuzzerInterval = 0;

      LogState = 1;
      break;

    case STATE_Recovery:
      TimeParachuteRelease = TimeCurrent;

      LedTeensyState = 2;
      LedTeensyInterval = 0;
      LedFlightState = 2;
      LedFlightInterval = 0;
      LedStageState = 1;
      LedStageInterval = DELAY_Flash_Slow;
      LedMotorState = 0;
      LedMotorInterval = 0;
      BuzzerState = 0;
      BuzzerInterval = 0;

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

      LedTeensyState = 2;
      LedTeensyInterval = 0;
      LedFlightState = 0;
      LedFlightInterval = 0;
      LedStageState = 1;
      LedStageInterval = DELAY_Flash_Slow;
      LedMotorState = 0;
      LedMotorInterval = 0;
      BuzzerState = 1;
      BuzzerInterval = DELAY_Flash_Slow;

      LogState = 0;
      break;

    case STATE_Charge:
      Servo1.detach();
      Servo2.detach();

      LedTeensyState = 1;
      LedTeensyInterval = DELAY_Flash_Slow;
      LedFlightState = 0;
      LedFlightInterval = 0;
      LedStageState = 0;
      LedStageInterval = 0;
      LedMotorState = 0;
      LedMotorInterval = 0;
      BuzzerState = 0;
      BuzzerInterval = 0;

      LogState = 0;
      break;

    case STATE_Readout:

      LedTeensyState = 1;
      LedTeensyInterval = DELAY_Flash_Fast;
      LedFlightState = 0;
      LedFlightInterval = 0;
      LedStageState = 0;
      LedStageInterval = 0;
      LedMotorState = 0;
      LedMotorInterval = 0;
      BuzzerState = 0;
      BuzzerInterval = 0;

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
      LedTeensyState = 2;
      LedTeensyInterval = 0;
      LedFlightState = 0;
      LedFlightInterval = 0;
      LedStageState = 0;
      LedStageInterval = 0;
      LedMotorState = 0;
      LedMotorInterval = 0;
      BuzzerState = 0;
      BuzzerInterval = 0;

      LogState = 1;
      break;

    case STATE_Error:
      LedTeensyState = 1;
      LedTeensyInterval = DELAY_Flash_Slow;
      LedFlightState = 1;
      LedFlightInterval = DELAY_Flash_Slow;
      LedStageState = 1;
      LedStageInterval = DELAY_Flash_Slow;
      LedMotorState = 1;
      LedMotorInterval = DELAY_Flash_Slow;
      BuzzerState = 2;
      BuzzerInterval = 0;

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
    case STATE_ArmedStageTwoLaunchAuto:
    case STATE_FlightOneIgnitionWait:
    case STATE_FlightOneCoast:
    case STATE_FlightOneBurn:
    case STATE_FlightTwoCoast:
    case STATE_FlightTwoBurn:
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

  OutputSet(LedTeensyState, LedTeensyBlink, PIN_D_LedTeensy, LedTeensyInterval, LedTeensyTimeStart, TimeCurrent);
  OutputSet(LedFlightState, LedFlightBlink, PIN_D_LedFlight, LedFlightInterval, LedFlightTimeStart, TimeCurrent);
  OutputSet(LedStageState, LedStageBlink, PIN_D_LedStage, LedStageInterval, LedStageTimeStart, TimeCurrent);
  OutputSet(LedMotorState, LedMotorBlink, PIN_D_LedMotor, LedMotorInterval, LedMotorTimeStart, TimeCurrent);
  OutputSet(BuzzerState, BuzzerBlink, PIN_D_Buzzer, BuzzerInterval, BuzzerTimeStart, TimeCurrent);

  // Write to storage
  if (LogState == 1)
  { 

#if (CODE_Debug > 0)
    Serial.print("T=");
    Serial.print(TimeCurrent-TimeArmedStart);
    Serial.print("\tS=");
    Serial.print(StateMaster);
    Serial.print("\tAx=");
    Serial.print(Acceleration.x, 2);
    Serial.print("\tAy=");
    Serial.print(Acceleration.y, 2);
    Serial.print("\tAz=");
    Serial.print(Acceleration.z, 2);
    Serial.print("\tYaw=");
    Serial.print(Ypr[0], 2);
    Serial.print("\tPitch=");
    Serial.print(Ypr[1], 2);
    Serial.print("\tRoll=");
    Serial.print(Ypr[2], 2);
    Serial.print("\tMX=");
    Serial.print(MagnetometerX, 5);
    Serial.print("\tMY=");
    Serial.print(MagnetometerY, 5);
    Serial.print("\tMZ=");
    Serial.print(MagnetometerZ, 5);
    Serial.print("\tT=");
    Serial.print(Temperature);
    Serial.print("\tP=");
    Serial.print(Pressure);
    Serial.print("\tH=");
    Serial.print(Altitude, 2);
    Serial.print("\t");
    Serial.print(SwitchRotary);
    Serial.println("");


    Serial.println("Test");
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
    SdFatLogFile.print(MagnetometerX, 5);
    SdFatLogFile.print("\t");
    SdFatLogFile.print(MagnetometerY, 5);
    SdFatLogFile.print("\t");
    SdFatLogFile.print(MagnetometerZ, 5);
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


























