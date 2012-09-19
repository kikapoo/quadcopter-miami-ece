#define TWENTY_MS  1600000  // 20ms/periphial clock(80MHz) --> .02/(1/80,000,000) 
#define TEN_MS  TWENTY_MS/2
#define FIVE_MS  TWENTY_MS/4
#define ONE_MS  TWENTY_MS/20


#define MIN_MOTOR TWENTY_MS/20  //OFF But Armed 1ms pulse minimum to arm ESC
#define MAX_MOTOR TWENTY_MS/10  //Full Speed 2ms pulse max output to ESC

//#define MIN_MOTOR 80000  //OFF But Armed 0x00013E66
//#define MAX_MOTOR 160000  //Full Speed 0x00027CCC

#define LEFT_MOTOR OC1RS
#define RIGHT_MOTOR OC2RS
#define FRONT_MOTOR OC3RS
#define REAR_MOTOR OC4RS

#define PID_CONSTANT  2000
int RollAccPID=0, PitchAccPID=0;



void ArmMotor(void){

    OC1CON = 0x0000;// Turn off the OC1 when performing the setup
    OC1R = MIN_MOTOR;// Initialize primary Compare register
    OC1RS = MIN_MOTOR;// Initialize secondary Compare register
    OC1CON = 0x0006;// Configure for PWM mode without Fault pin enabled
    
    OC2CON = 0x0000;// Turn off the OC2 when performing the setup
    OC2R = MIN_MOTOR;// Initialize primary Compare register
    OC2RS = MIN_MOTOR;// Initialize secondary Compare register
    OC2CON = 0x0006;// Configure for PWM mode without Fault pin enabled
    
    OC3CON = 0x0000;// Turn off the OC3 when performing the setup
    OC3R = MIN_MOTOR;// Initialize primary Compare register
    OC3RS = MIN_MOTOR;// Initialize secondary Compare register
    OC3CON = 0x0006;// Configure for PWM mode without Fault pin enabled
    
    OC4CON = 0x0000;// Turn off the OC4 when performing the setup
    OC4R = MIN_MOTOR;// Initialize primary Compare register
    OC4RS = MIN_MOTOR;// Initialize secondary Compare register
    OC4CON = 0x0006;// Configure for PWM mode without Fault pin enabled
    
    
    
    
    T2CONSET = 0x0008;// Enable 32-bit Timer mode
    
    PR2 = TWENTY_MS; // period of 20ms = 50Hz
    T2CONSET = 0x8000;// Enable Timer2
    
    OC1CONSET = 0x8020;// Enable OC1 in 32-bit mode.
    OC2CONSET = 0x8020;// Enable OC2 in 32-bit mode.
    OC3CONSET = 0x8020;// Enable OC3 in 32-bit mode.
    OC4CONSET = 0x8020;// Enable OC4 in 32-bit mode. 
}

//Timer4 and 5 are used to time the overal loop
void start_GDt_Clock(void){
  TMR4 = 0;
  TMR5 = 0;
  T4CONSET = 0x0008;// Enable 32-bit Timer mode
  PR4 = FIVE_MS; // period of 5ms = 200Hz
//  PR4 = ONE_MS; // period of 1ms = 1000Hz

  T4CONSET = 0x8000;// Enable Timer4
  
  IFS0CLR = 0x00100000;// Clear the T5 interrupt flag

}


inline void updateMotor(void){
 //Program ESC
//     LEFT_MOTOR = (MIN_MOTOR+yaw)+throttle;   
//     RIGHT_MOTOR = (MIN_MOTOR+yaw)+throttle;
//     FRONT_MOTOR = (MIN_MOTOR-yaw)+throttle;
//     REAR_MOTOR = (MIN_MOTOR-yaw)+throttle;
 
 //Generic PID Test
     LEFT_MOTOR = (MIN_MOTOR)+throttle;  
     RIGHT_MOTOR = (MIN_MOTOR)+throttle;
    // LEFT_MOTOR = (MIN_MOTOR+yaw)+throttle+(1000.0* E_roll);   
    // RIGHT_MOTOR = (MIN_MOTOR+yaw)+throttle-(1000.0*E_roll);
     FRONT_MOTOR = (MIN_MOTOR-yaw)+throttle+(4000.0*E_pitch);
     REAR_MOTOR = (MIN_MOTOR-yaw)+throttle-(4000.0*E_pitch);


//Actual PID
//     RollAccPID +=  E_roll*PID_CONSTANT;
//     PitchAccPID +=  E_pitch*PID_CONSTANT;
//
//     LEFT_MOTOR = MIN_MOTOR+throttle+yaw+RollAccPID;   
//     RIGHT_MOTOR = MIN_MOTOR+throttle+yaw-RollAccPID;
//     FRONT_MOTOR = MIN_MOTOR+throttle-yaw+PitchAccPID ;
//     REAR_MOTOR = MIN_MOTOR+throttle-yaw-PitchAccPID;
  
// 
//      Serial.print(LEFT_MOTOR);
//      Serial.print(", ");
//      Serial.println(RIGHT_MOTOR);
//
//      Serial.println(roll);
//      Serial.print(", ");
//      Serial.println(pitch,4);
//  

}
