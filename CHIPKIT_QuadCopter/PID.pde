
void initPID(void){

  RollPID.SetOutputLimits(-2.0,  2.0);
  PitchPID.SetOutputLimits(-2.0,  2.0);

  RollPID.SetSampleTime(0.02);//LOOP_TIME*5);
  PitchPID.SetSampleTime(0.02);//LOOP_TIME*5);

  RollPID.SetMode(AUTOMATIC);
  PitchPID.SetMode(AUTOMATIC);
  
}

inline void updatePID(void){
       
     RollPID.Compute();
     PitchPID.Compute(); 
     

  
  
}
