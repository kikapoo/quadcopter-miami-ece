
void initPID(void){

  RollPID.SetOutputLimits(-2.0,  2.0);
  PitchPID.SetOutputLimits(-2.0,  2.0);

  RollPID.SetSampleTime(20);  //20 ms for 50Hz
  PitchPID.SetSampleTime(20);//20 ms for 50Hz

  RollPID.SetMode(MANUAL);
  PitchPID.SetMode(MANUAL);
  
}

inline void updatePID(void){
       
     RollPID.Compute();
     PitchPID.Compute(); 
 }
