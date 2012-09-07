#include <PS2X_lib.h>  //for v1.6

PS2X ps2x; // create PS2 Controller Class

//right now, the library does NOT support hot pluggable controllers, meaning 
//you must always either restart your Arduino after you conect the controller, 
//or call config_gamepad(pins) again after connecting the controller.
int error = 0; 
byte type = 0;
byte vibrate = 0;
int printCount = 0;
///////////////////////////
typedef union  {
   char B[4];
   int  I;
   float F; }
TX_data;

TX_data pitch;
//byte *pitchPtr;

TX_data roll;
//byte *rollPtr;

TX_data throttle;
//byte *throttlePtr;

TX_data yaw;
int yawFix, throttleFix;
int yawAnalog, throttleAnalog;
int rateINT;
float rateFLOAT;
float RPscale, YTscale;
byte syncBytes[4];


void setup(){
  //roll pitch throttle
syncBytes[0]=0x55;
syncBytes[1]=0xAA;
syncBytes[2]=0xB5;
syncBytes[3]=0x5C; 
  
 Serial1.begin(2400);
 Serial.begin(57600);

 //CHANGES for v1.6 HERE!!! **************PAY ATTENTION*************
  
 error = ps2x.config_gamepad(13,11,10,12, true, true);   //setup pins and settings:  GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
 
 if(error == 0){
   Serial.println("Found Controller, configured successful");
   Serial.println("Try out all the buttons, X will vibrate the controller, faster as you press harder;");
  Serial.println("holding L1 or R1 will print out the analog stick values.");
  Serial.println("Go to www.billporter.info for updates and to report bugs.");
 }
   
  else if(error == 1)
   Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");
   
  else if(error == 2)
   Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");
   
  else if(error == 3)
   Serial.println("Controller refusing to enter Pressures mode, may not support it. ");
   
   //Serial.print(ps2x.Analog(1), HEX);
   
   type = ps2x.readType(); 
     switch(type) {
       case 0:
        Serial.println("Unknown Controller type");
       break;
       case 1:
        Serial.println("DualShock Controller Found");
       break;
       case 2:
         Serial.println("GuitarHero Controller Found");
       break;
     }

    int start = ps2x.Button(PSB_START);
    while(!start)
    {
      ps2x.read_gamepad();
      start = ps2x.Button(PSB_START);
    }
    throttleFix = 0;
    yawFix = 0;
}

void loop(){
  if(printCount > 1000) {
    printCount = 0;
  }
  
   /* You must Read Gamepad to get new values
   Read GamePad and set vibration values
   ps2x.read_gamepad(small motor on/off, larger motor strenght from 0-255)
   if you don't enable the rumble, use ps2x.read_gamepad(); with no values
   
   you should call this at least once a second
   */
   ps2x.read_gamepad();
   
   
 if(error == 1) //skip loop if no controller found
  return; 
  
 
 else { //DualShock Controller
   //Check aggressive/less aggressive on Roll Pitch
   if(ps2x.Button(PSB_R2 )){
     if(ps2x.Button(PSB_R1))
       RPscale = 1;
     else
       RPscale = .25;
    }else{ 
      if(ps2x.Button(PSB_R1))
       RPscale = 4.0;
   }
   //Check aggressive/less aggressive on Yaw Throttle
   if(ps2x.Button(PSB_L2 )){
     if(ps2x.Button(PSB_L1 ))
       YTscale = 1;
     else
       YTscale = .25;
   }else{
     if(ps2x.Button(PSB_L1 ))
       YTscale = 4.0;
   }
   //Calculate analog adjustments to Throttle/Yaw/Roll/Pitch 
   throttleAnalog = (128-ps2x.Analog(PSS_LY))*4000/128*YTscale;
   yawAnalog = (ps2x.Analog(PSS_LX)-128)*1000/128*YTscale;
   
    //Pitch
    pitch.F = ((((float)ps2x.Analog(PSS_RY)/128.0)-1.0)*.52359)*RPscale;
    
    //Roll
    roll.F = ((float)(((ps2x.Analog(PSS_RX)/128.0)-1.0)*.52359))*RPscale;

    if(ps2x.Button(PSB_GREEN))
      throttleFix = 80000; 
    
    if(ps2x.Button(PSB_CROSS ))
      throttleFix = 0; 


    //Throttle Fixed
     if(ps2x.Button(PSB_PAD_UP)){
       while (ps2x.Button(PSB_PAD_UP))  {
           ps2x.read_gamepad();
        if(ps2x.ButtonReleased(PSB_PAD_UP)){
           throttleFix += 2000; 
         }
       }
   }
   
   
     if(ps2x.Button(PSB_PAD_DOWN))
   {
     while (ps2x.Button(PSB_PAD_DOWN)) {
       ps2x.read_gamepad();
       if(ps2x.ButtonReleased(PSB_PAD_DOWN)) {
         throttleFix -= 2000; 
       }
     }
   }
   
    //YAW Fix
    if(ps2x.Button(PSB_PAD_LEFT))
   {
     while (ps2x.Button(PSB_PAD_LEFT))  {
       ps2x.read_gamepad();
       if(ps2x.ButtonReleased(PSB_PAD_LEFT)) 
       {
       yawFix += 200; 
       }
     }
   }
   
   
     if(ps2x.Button(PSB_PAD_RIGHT))
   {
     while (ps2x.Button(PSB_PAD_RIGHT)) {
       ps2x.read_gamepad();
       if(ps2x.ButtonReleased(PSB_PAD_RIGHT)) {
         yawFix -= 200; 
       }
     }
   }
     if(ps2x.Button(PSB_SELECT)){
        syncBytes[2]=0xC5;
        syncBytes[3]=0x5B; 
     }else{
        syncBytes[2]=0xB5;
        syncBytes[3]=0x5C; 
     }

    throttle.I = throttleFix+throttleAnalog;
    yaw.I = yawFix+yawAnalog;

    //Send sync bytes
    Serial.println("Sync Bytes");
    for(int i = 0; i < 4;i++) {
      Serial1.write(syncBytes[i]);
      Serial.print(syncBytes[i],HEX);
    }
    
    Serial.println("");
    
    //Send roll Bytes
    Serial.println("Roll Bytes");
    for(int i = 0;i < 4; i++ ){
      Serial1.write(roll.B[i]);
      Serial.print(roll.B[i],HEX);
    }

    Serial.print("  ");
    Serial.print(roll.F);
    Serial.println("  ");
    
    //Send pitch bytes
    Serial.println("Pitch Bytes");
    for(int i = 0;i < 4; i++) {
      Serial1.write(pitch.B[i]);
      Serial.print(pitch.B[i],HEX);
    }
    Serial.print("  ");
    Serial.print(pitch.F);
    Serial.println("  ");
    
    //Send throttle bytes
    //if(printCount == 1000) {
    Serial.print("Throttle Bytes");
   // }
    
    for(int i = 0;i < 4; i++) {
      Serial1.write(throttle.B[i]);
      Serial.print(throttle.B[i],DEC);
      }
    Serial.print("  ");
    Serial.print(throttle.I);
    Serial.println("  ");
    Serial.println("");

    Serial.print("Yaw Bytes");

    for(int i = 0;i < 4; i++) {
      Serial1.write(yaw.B[i]);
      Serial.print(yaw.B[i],DEC);
      }
    Serial.print("  ");
    Serial.print(yaw.I);
    Serial.println("  ");
    Serial.println("");

   delay(50);
  
   printCount++;

   }
}

