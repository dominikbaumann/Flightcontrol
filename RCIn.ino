//RCIn
uint16_t g_values[3];                    // output buffer for ServoIn
uint8_t  g_workIn[SERVOIN_WORK_SIZE(3)]; // buffer for the ServoIn class

rc::ServoIn g_ServoIn(g_values, g_workIn, 3);
void init_rcin(str_rem *remote){
  // Initialize timer1
  rc::Timer1::init();
  
  // Define input pin
  pinMode(8, INPUT);
  pinMode(9, INPUT);
  pinMode(11, INPUT);
  
  // allow pin change interrupts for pins 8, 9 and 11
  PCMSK0 = (1 << PCINT4) | (1 << PCINT5) | (1 << PCINT7);
  
  // enable pin change interrupt 
  PCICR = (1 << PCIE0);
  
  // start listening
  g_ServoIn.start();

  // start value: middle position
  remote->aileron = 1500;
  remote->elevator = 1500;
  remote->rudder = 1500;
}

void rcin_update(str_rem *remote){
  g_ServoIn.update();
  remote->aileron = g_values[0];
  remote->elevator = g_values[1];
  remote->rudder = g_values[2];
}

// Interrupt handling code

static uint8_t lastB = 0;

// Pin change interrupt
ISR(PCINT0_vect)
{
  uint8_t newB = PINB;
  uint8_t chgB = newB ^ lastB; // bitwise XOR will set all bits that have changed
  lastB = newB;
  
    // find out which pin has changed
   if (chgB)
 {
    // find out which pin has changed
    if (chgB & _BV(PINB4))
    {
      g_ServoIn.pinChanged(0, newB & _BV(PINB4));
    }
    if (chgB & _BV(PINB5))
    {
      g_ServoIn.pinChanged(1, newB & _BV(PINB5));
    }
    if (chgB & _BV(PINB7))
    {
      g_ServoIn.pinChanged(2, newB & _BV(PINB7));
    }
  }
}
