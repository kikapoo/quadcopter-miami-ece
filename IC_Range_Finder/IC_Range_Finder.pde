#define RANGE_PULSE 800
#define TWENTY_MS  1600000
#include <plib.h>
int Distance=0;
int T3_OVFL_CNT = 0;
long int Captured = 0;
void setup() {
  Serial.begin(9600);
  range_init();
  
  while(!mIC1CaptureReady()) ;
  range_start();
//interrupts();
}

void loop() {
   //if(IFS0&0x00000020) 
    // range_start();
   //delay(1000);
   
   while(!range_available());
   
   Serial.println(T3_OVFL_CNT);
   Distance = range_get();
   Serial.println(Distance);
   
   delay(1000); 
   range_start();
   
  
}


void range_init(void){
  pinMode(2, INPUT);
  pinMode(10, OUTPUT);
  OC5CON = 0x0000;// Turn off the OC4 when performing the setup
  OC5R = 0;// Initialize primary Compare register
  OC5RS = 0;// Initialize secondary Compare register
  OC5CON = 0x0006;// Configure for PWM mode without Fault pin enabled
  OC5CONSET = 0x8020;// Enable OC4 in 32-bit mode. 

  IFS0CLR = 0x00001000;// Clear the T3 interrupt flag
  T2CON = 0x0; // Stop any 16/32-bit Timer4 operation
  T3CON = 0x0;

  OpenCapture1(IC_EVERY_FALL_EDGE | IC_INT_1CAPTURE | IC_TIMER3_SRC | IC_FEDGE_RISE | IC_ON | IC_CAP_32BIT); 
  IFS0CLR = 0x00000020;// Clear the IC1 interrupt flag
  T2CONSET = 0x0008;// Enable 32-bit Timer mode
  PR2 = TWENTY_MS;// Set period
                 // Configure Timer3 interrupt. Note that in PWM mode, the corresponding source timer 
                 // interrupt flag is asserted. OC interrupt is not generated in PWM mode.
  TMR2 = 0;
  TMR3 = 0;
  T2CONSET = 0x8000;// Enable Timer2
  IPC3SET = 0x00000004; // Set T3 priority level = 1
  IPC3SET = 0x00000001; // Set T3 sub-priority level = 1
}

void range_start(void){
  IC1CONSET = 0x000080;
  OC5RS = RANGE_PULSE;  //Send one pulse out to start PING
  IFS0CLR = 0x00001000;// Clear the T3 interrupt flag
  IFS0CLR = 0x00000020;// Clear the IC1 interrupt flag
  IEC0SET = 0x00001000;//Enable the T3 interrupt 
  
  
}
int range_available(void){
  if((IFS0 & 0x00000020)&&(!(IEC0&0x00001000)))  //if Capture event and timer interupt off
    return 1;
  else
    return 0;
}

long int range_get(void){
  int wasted;
  int range;
  
  range = IC1BUF;
  range += T3_OVFL_CNT*TWENTY_MS-RANGE_PULSE;
  range = range/11942-2;
  
  while(IC1CON&0x0008)
    wasted = IC1BUF;
  
  IC1CONCLR = 0x000080;
  return range;
}


extern "C"{ 
void __ISR(_TIMER_3_VECTOR,ipl3) OVFLcnt(void)
{
        
    IFS0CLR = 0x00001000;// Clear the T3 interrupt flag
 
    if(IFS0&0x00000020) //Captured
       IEC0CLR = 0x00001000;//Capture occured turn off T3 interrupt
    else
       ++T3_OVFL_CNT; //No capture but there was a timer overflow
    
    if(OC5R==RANGE_PULSE){
      OC5RS = 0x0; //Turn off output pulse
      T3_OVFL_CNT=0;
    }

 }
// void __ISR(_INPUT_CAPTURE_1_VECTOR,ipl3) IC1captured(void){
//   IFS0CLR = 0x00000020;// Clear the IC1 interrupt flag
//   Captured = 1;
//   
// }
 
}
