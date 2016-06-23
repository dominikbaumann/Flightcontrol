//FlightControl
//Main Sketch 
#include "Def_General.h"
#include <EEPROM.h>
#include <Wire.h>
#include <ServoIn.h>
#include <Timer1.h>
#include "I2Cdev.h"
#include "RTIMUSettings.h"
#include "CalLib.h"
#include <avr/wdt.h>

void setup(){
  Serial.begin(SERIAL_PORT_SPEED);
  Wire.begin();
  init_sensor(&angles);
  sensor_calibrateCompass();
  init_control(&controllerRoll, &controllerYaw, &controllerPitch);
  init_servo();
  init_rcin(&remote);
  wdt_enable(WDTO_2S);    //enable watchdog timer, reset if board doesn't respond in 2s
}

void loop(){
  sensor_readData();
  rcin_update(&remote);
  control_roll(&angles, &controllerRoll, &remote);
  control_yaw(&angles, &controllerYaw, &remote);
  control_pitch(&angles, &controllerPitch, &remote);
  wdt_reset();    //reset watchdog timer, board is working
}

