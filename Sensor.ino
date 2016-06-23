// Sensor.ino
// reads the data of the MPU 92/65, including gyro, accel and compass

RTIMU *imu;                                           //object for data of the sensors
RTIMUSettings settings;                               // the settings object
CALLIB_DATA calData;                                  // the calibration data
int sampleCount;
unsigned long lastRate;                               // first cycle

void init_sensor(str_eul *angles)
{
    int errcode;
    imu = RTIMU::createIMU(&settings);                // create the imu object

    lastRate = millis();
    sampleCount = 0;

    // Slerp power controls the fusion and can be between 0 and 1
    // 0 means that only gyros are used, 1 means that only accels/compass are used
    // In-between gives the fusion mix.
    
    fusion.setSlerpPower(0.02);
    
    // use of sensors in the fusion algorithm can be controlled here
    // change any of these to false to disable that sensor
    
    fusion.setGyroEnable(true);
    fusion.setAccelEnable(true);
    fusion.setCompassEnable(true);

    angles->roll = 0;
    angles->pitch = 0;
    angles->yaw = 0;
    angles->yawRate = 0;
    angles->timeStamp = 0;
    angles->timeStamp_last = 0;
}

void sensor_calibrateCompass(){
    calLibRead(0, &calData);                           // pick up existing mag data if there   

  calData.magValid = false;
  for (int i = 0; i < 3; i++) {
    calData.magMin[i] = 10000000;                    // init mag cal data
    calData.magMax[i] = -10000000;
  }
  
  imu->IMUInit();
  imu->setCalibrationMode(true);  // make sure we get raw data
}

void sensor_readData(){
    unsigned long now = millis();
    unsigned long delta;
    int loopCount = 1;
  
    while (imu->IMURead()) {                                // get the latest data if ready yet
        // this flushes remaining data in case we are falling behind
        if (++loopCount >= 10)
            continue;
        fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
        sampleCount++;
        if ((delta = now - lastRate) >= 10) {
            sampleCount = 0;
            lastRate = now;
            long int yawRate = imu->getGyro().z();
            long int timeStamp = imu->getTimestamp();
            sensor_dataOut((RTVector3&)fusion.getFusionPose(), &angles, yawRate, timeStamp);
        }
    }
}

void sensor_dataOut(RTVector3& pose, str_eul *angles, long int yawRate, long int timeStamp){
  angles->timeStamp_last = angles->timeStamp;
  if(pose.x()*RTMATH_RAD_TO_DEGREE<-90)angles->pitch=(pose.x() * RTMATH_RAD_TO_DEGREE + 180)*1000;
  else if(pose.x()*RTMATH_RAD_TO_DEGREE>90)angles->pitch=(pose.x() * RTMATH_RAD_TO_DEGREE -180)*1000;
  else angles->pitch = pose.x() * RTMATH_RAD_TO_DEGREE * 1000;
  if(pose.y()*RTMATH_RAD_TO_DEGREE<-90)angles->roll=(pose.y() * RTMATH_RAD_TO_DEGREE + 180)*1000;
  else if(pose.y()*RTMATH_RAD_TO_DEGREE>90)angles->roll=(pose.y() * RTMATH_RAD_TO_DEGREE -180)*1000;
  else angles->roll = pose.y() * RTMATH_RAD_TO_DEGREE * 1000;
  if(pose.z()*RTMATH_RAD_TO_DEGREE<-90)angles->yaw=(pose.z() * RTMATH_RAD_TO_DEGREE + 180)*1000;
  else if(pose.z()*RTMATH_RAD_TO_DEGREE>90)angles->yaw=(pose.z() * RTMATH_RAD_TO_DEGREE -180)*1000;
  else angles->yaw = pose.z() * RTMATH_RAD_TO_DEGREE * 1000;
  angles->yawRate = yawRate;
  angles->timeStamp = timeStamp;
}


