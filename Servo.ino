//Servo
void init_servo()
{
  cli();    //disable interrupts
  TC4H = 1;
  OCR4A = 18;
  OCR4B = 18;
  OCR4D = 18;   //compare match register
  TC4H = 3;
  OCR4C = 0xFF;
  TC4H = 0;

  TIMSK4 |= _BV(TOIE4);
  TCCR4A |= _BV(PWM4A) | _BV(PWM4B) | _BV(COM4A1) | _BV(COM4B1) | _BV(COM4B0) | _BV(COM4A0); // enable fast PWM mode
  TCCR4C = _BV(PWM4D) | _BV(COM4D1) | _BV(COM4D0) | _BV(COM4A0S) | _BV(COM4B0S) | _BV(COM4A1S) | _BV(COM4B1S);
  TCCR4B = _BV(CS42) | _BV(CS40);    //Prescaler 16

  TCCR3A &= ~0x03;
  TCCR3B = _BV(WGM32) | _BV(CS31);    //Prescaler 8, CTC mode
  OCR3AH = 0x9C;
  OCR3AL = 0x93;
  TIMSK3 |= _BV(OCIE3A);
  sei();    //enable interrupts
  pinMode(6, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(13, OUTPUT);
}

// function to regulate the aircraft by the servo through the output of the controller
void  servo_driveRoll(long int angle)
{
  if(angle>ANGLE_MAX)angle = ANGLE_MAX;   
  if(angle<ANGLE_MIN)angle = ANGLE_MIN;
  long int servo = map(angle,ANGLE_MIN,ANGLE_MAX,MS_MIN,MS_MAX1);  
  TC4H = servo>>8;
  OCR4B = servo;
  TC4H = 0;
}

void servo_driveYaw(long int angle)
{
  if(angle>ANGLE_MAX)angle = ANGLE_MAX;   
  if(angle<ANGLE_MIN)angle = ANGLE_MIN;
  long int servo = map(angle,ANGLE_MIN,ANGLE_MAX,MS_MIN,MS_MAX2);  
  TC4H = servo>>8;
  OCR4A = servo;
  TC4H = 0;
}

void servo_drivePitch(long int angle)
{
  if(angle>ANGLE_MAX)angle = ANGLE_MAX;   
  if(angle<ANGLE_MIN)angle = ANGLE_MIN;
  long int servo = map(angle,ANGLE_MIN,ANGLE_MAX,MS_MIN,MS_MAX2);  
  TC4H = servo>>8;
  OCR4D = servo;
  TC4H = 0;
}

ISR(TIMER4_OVF_vect){
  TCCR4B &= ~0x0F;
}

ISR(TIMER3_COMPA_vect){
  TCCR4B = _BV(CS42) | _BV(CS40); 
}

