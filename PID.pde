
void initPID(void){

  RollPID.SetOutputLimits(-2.0,  2.0);
  PitchPID.SetOutputLimits(-2.0,  2.0);

  RollPID.SetSampleTime(LOOP_TIME*5);
  PitchPID.SetSampleTime(LOOP_TIME*5);

  RollPID.SetMode(AUTOMATIC);
  PitchPID.SetMode(AUTOMATIC);
  
}

inline void updatePID(void){
       
     RollPID.Compute();
     PitchPID.Compute(); 
     

  
  
}
