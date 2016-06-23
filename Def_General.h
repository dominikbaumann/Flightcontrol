// Def_General.h
//General Definitions
#include "RTIMU.h"
#include "RTFusionRTQF.h" 

#define SERIAL_PORT_SPEED 9600

//Definitions for Control.ino
//Defines the parameters for the controllers
typedef struct{
  double kp;
  double ki;
  double integral;
}str_con;

// definition of the controller parameter
str_con controllerRoll, controllerYaw, controllerPitch;


//Definitions for the RCIn.ino
//Parameters of the remote
typedef struct{
  long int rudder;
  long int aileron;
  long int elevator;
}str_rem;

//definition of the struct
str_rem remote;


//Definitions for the Sensor.ino
//Fusion object for getting the pose by the sensor through the library
RTFusionRTQF fusion; 

//struct for the euler angles of the aircraft measured by the mpu 92/65
//besides euler angles, also time stamp and angular velocity of yaw are important
typedef struct{
  long int yaw;
  long int pitch;
  long int roll;
  long int yawRate;
  long int timeStamp;
  long int timeStamp_last;
}str_eul;

//definition of the struct
str_eul angles;



//Definitions for the Servo.ino

//Define min and max values for angle and servo signals
static const long int ANGLE_MAX = 90000;    //max angle is 90, scaled by 1000
static const long int ANGLE_MIN = (-90000);   //min angle is -90, scaled by 1000
//min and max values can differ depending on the used servos. Have to be adjusted for own servos.
static const long int MS_MAX1 = 450;  //max length of the servo signal for original servos in ms
static const long int MS_MAX2 = 545;  //max length of the servos signal for new servos in ms
static const long int MS_MIN = 24;    //min length of the servo signal in ms
