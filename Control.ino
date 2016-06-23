//Control
//Initialize the controllers and its parameters
void init_control(str_con *controllerRoll, str_con *controllerYaw, str_con *controllerPitch){
  controllerRoll->kp = 5;
  controllerRoll->ki = 0.00075;
  controllerRoll->integral = 0;
  controllerYaw->kp = 20;
  controllerYaw->ki = 0.002;
  controllerYaw->integral = 0;
  controllerPitch->kp = -5;
  controllerPitch->ki = 0;
  controllerPitch->integral = 0;

  //pin 4 and 5 for the switch, 7 and 12 for additional inputs of remote
  pinMode(5, INPUT);
  pinMode(4, INPUT);
  digitalWrite(5, HIGH);
  digitalWrite(4, HIGH);
  pinMode(12, INPUT);
  pinMode(7, INPUT);
}

//functions to control the angles of the aircraft

void control_roll(str_eul *angles, str_con *controllerRoll, str_rem *remote){
  long int roll = angles->roll;
  long int angle = 0;
  //if value given by the remote is 0, remote is turned off, desired angle should be 0° then
  if(remote->aileron != 0)angle = map(remote->aileron, 1000, 2000, -90000, 90000);
  long int error = angle - roll;
  long int proportional = (controllerRoll->kp)*(error);
  controllerRoll->integral += (controllerRoll->ki)*error*((angles->timeStamp)-(angles->timeStamp_last));
  //anti-reset windup
  if(controllerRoll->integral>90000) controllerRoll->integral = 90000;
  if(controllerRoll->integral<-90000) controllerRoll->integral = -90000;
  long int integral = (controllerRoll->integral);
  long int regulate = proportional+integral;
  servo_driveRoll(regulate);
}

void control_yaw(str_eul *angles, str_con *controllerYaw, str_rem *remote){
  long int yawRate = angles->yawRate;
  long int angVel = 0;
  long int angle = 0;
  if(remote->rudder!=0){
    angle = map(remote->rudder,1000,2000,-90000,90000);
    //map remote signals to angle velocities. for small amplitudes -3 to 3 °/s, for big amplitudes -90 to 90°/s 
    if((remote->rudder > 1200) &&(remote->rudder <1500))angVel = map(remote->rudder, 1200, 1500, -3000, 0);
    else if((remote->rudder >1600)&&(remote->rudder < 1800))angVel = map(remote->rudder,1600,1800,0,3000);
    else if(remote->rudder>=1800)angVel = map(remote->rudder,1800,2000,30000,90000);
    else angVel = map(remote->rudder, 1000,1200,-90000,-3000);
  }
  long int proportional = (controllerYaw->kp)*yawRate;
  controllerYaw->integral+=(controllerYaw->ki)*yawRate*((angles->timeStamp)-(angles->timeStamp_last));
  if((controllerYaw->integral)>90000) (controllerYaw->integral) = 90000;
  if((controllerYaw->integral)<-90000) (controllerYaw->integral) = -90000;
  long int integral = (controllerYaw->integral);
  long int regulate = proportional + integral;
  long int regulateVel = (angVel-yawRate);    //Kp is set to 1 for this controller for first tests
  if(((remote->rudder > 1500) && (remote->rudder < 1600))/*&&((remote->aileron > 1490) && (remote->aileron < 1550))*/) servo_driveYaw(regulate*1000);
  else{
    controllerYaw->integral = 0;
      if(pulseIn(12,HIGH)<1500)  servo_driveYaw(angle);
      else servo_driveYaw(regulateVel);
  
  }
}

void control_pitch(str_eul *angles, str_con *controllerPitch, str_rem *remote){
  long int pitch = angles->pitch;
  long int angle = 0;
  if(remote->elevator!=0)angle = map(remote->elevator, 1000, 2000, -90000, 90000);
  long int error = angle - pitch;
  long int proportional = (controllerPitch->kp)*(error);
  controllerPitch->integral += error*((angles->timeStamp)-(angles->timeStamp_last));
  long int integral = (controllerPitch->ki)*(controllerPitch->integral);
  if(integral>90000) integral = 90000;
  if(integral<-90000) integral = -90000;
  long int regulate = proportional + integral;
  if(pulseIn(7,HIGH)>1500)servo_drivePitch(regulate); 
  else servo_drivePitch(angle);
}

