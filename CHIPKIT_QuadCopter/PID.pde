
void initPID(void){

  RollPID.SetOutputLimits(-PID_BOUND,  PID_BOUND);
  PitchPID.SetOutputLimits(-PID_BOUND,  PID_BOUND);

  RollPID.SetSampleTime(LOOP_TIME);  //20 ms for 50Hz
  PitchPID.SetSampleTime(LOOP_TIME);//20 ms for 50Hz

  RollPID.SetMode(MANUAL);
  PitchPID.SetMode(MANUAL);
  
}

inline void updatePID(void){
       
     RollPID.Compute();
     PitchPID.Compute(); 
 }
