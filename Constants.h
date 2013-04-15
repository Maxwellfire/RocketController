#define CODE_Debug             1

#define CODE_Test              1
#define CODE_TestTime          5000

#define TIME_Delay_I2c_ReadWrite   5

// Pins
#define PIN_D_LedTeensy       6
#define PIN_D_SwitchRot1      7
#define PIN_D_SwitchRot2      8
#define PIN_D_SwitchRot3      9 
#define PIN_D_SwitchRot4      10 
#define PIN_D_Servo1          11
#define PIN_D_Servo2          12
#define PIN_D_Button1         13
#define PIN_D_Buzzer          14
#define PIN_D_LedFlight       15
#define PIN_D_LedStage        16
#define PIN_D_LedMotor        17


// States describing rocket flight
#define STATE_Initialize                     0      //T:0  F:O  S:O  M:O  B:0
#define STATE_PreFlight                      1      //T:1  F:ff S:O  M:O  B:0

#define STATE_ArmedStageTwoLaunchAuto        12     //T:1  F:1  S:0  M:O  B:ff

#define STATE_FlightOneIgnitionWait          20     //T:1  F:1  S:ff  M:ff B:ff
#define STATE_FlightOneBurn                  25     //T:1  F:1  S:ff  M:1  B:0
#define STATE_FlightOneCoast                 26     //T:1  F:1  S:ff  M:O  B:0

#define STATE_FlightTwoIgnitionWaitFromArmed 30     //T:1  F:1  S:1  M:ff B:ff
#define STATE_FlightTwoIgnitionWaitFromOne   31     //T:1  F:1  S:1  M:ff B:0
#define STATE_FlightTwoBurn                  35     //T:1  F:1  S:1  M:1  B:0
#define STATE_FlightTwoCoast                 36     //T:1  F:1  G:1  M:O  B:0

#define STATE_Recovery                       40     //T:1  F:1  S:sf M:0  B:0

#define STATE_Final                          50     //T:1  F:0  S:sf M:0  B:sf

#define STATE_Charge                         110    //T:sF F:0  S:0  M:0  B:0
#define STATE_Readout                        120    //T:F  F:0  S:0  M:0  B:0
#define STATE_Logger                         130    //T:1  F:0  S:0  M:0  B:0

#define STATE_Error                          999    //T:sF F:sf R:sf G:sf B:1

// Delay in milliseconds
#define DELAY_Armed_To_LaunchAuto                20000
#define DELAY_StageOneCoast_To_StageTwoIgnition  2000
//
#define DELAY_Coast_To_Parachute                 5000
#define DELAY_RecoveryNotMoving_To_Final         3000

#define DELAY_Flash_Slow                         1000
#define DELAY_Flash_Fast                         250

#define EWMA_Lembda                0.99

// Pressure DeadBand
#define ACCELERATION_Ignition     -1.5

// Pressure DeadBand
#define DEADBAND_Altitude_Final    3.0
#define DEADBAND_Altitude_Flight   5.0

// Servo Positions in degrees
#define SERVO_1_Position_Start    120
#define SERVO_1_Position_End      60
#define SERVO_2_Position_Start    120
#define SERVO_2_Position_End      60

#define BMP085_Ewma_Lambda       0.1










