#define CODE_Debug             0
#define CODE_Debug_BMA180      0
#define CODE_Debug_BMP085      0

#define TIME_Delay_I2c_ReadWrite   5

// Pins
#define PIN_D_SwitchRot1      2
#define PIN_D_SwitchRot2      3
#define PIN_D_SwitchRot3      4 
#define PIN_D_SwitchRot4      5 
#define PIN_D_LedGreen        6
#define PIN_D_LedRedBuzzer    7
#define PIN_D_Servo1          8
#define PIN_D_Servo2          9

#define PIN_A_Button1         0

// States describing rocket flight
#define STATE_Initialize                 0
#define STATE_PreFlight                  1
#define STATE_Armed1StageTimer           10
#define STATE_ArmedOneStageLaunchManual  11
#define STATE_ArmedOneStageLaunchAuto    12
#define STATE_ArmedTwoStageLaunchManual  13
#define STATE_IgnitionOne                20
#define STATE_StageOneBurn               21
#define STATE_StageOneCoast              22
#define STATE_IgnitionTwo                30
#define STATE_StageTwoBurn               31
#define STATE_StageTwoCoast              32
#define STATE_Recovery                   40
#define STATE_Final                      50
#define STATE_Charge                     110
#define STATE_Readout                    120
#define STATE_Logger                     130
#define STATE_Error                      999

// Delay in milliseconds
#define DELAY_Armed_To_LaunchAuto                20000
#define DELAY_StageOneCoast_To_StageTwoIgnition  2000
//
#define DELAY_Coast_To_Parachute                 5000
#define DELAY_RecoveryNotMoving_To_Final         3000

#define EWMA_Lembda                0.99

// Pressure DeadBand
#define ACCELERATION_Ignition     -1.5

// Pressure DeadBand
#define DEADBAND_Altitude_Final    3.0
#define DEADBAND_Altitude_Flight   15.0

// Servo Positions in degrees
#define SERVO_1_Position_Start    180
#define SERVO_1_Position_End      0
#define SERVO_2_Position_Start    180
#define SERVO_2_Position_End      0

#define BMP085_Ewma_Lambda       0.1










