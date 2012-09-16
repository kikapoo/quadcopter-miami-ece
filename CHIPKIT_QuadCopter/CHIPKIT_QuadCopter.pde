/*

MinIMU-9-Arduino-AHRS
Pololu MinIMU-9 + Arduino AHRS (Attitude and Heading Reference System)

Copyright (c) 2011 Pololu Corporation.
http://www.pololu.com/

MinIMU-9-Arduino-AHRS is based on sf9domahrs by Doug Weibel and Jose Julio:
http://code.google.com/p/sf9domahrs/

sf9domahrs is based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose
Julio and Doug Weibel:
http://code.google.com/p/ardu-imu/

MinIMU-9-Arduino-AHRS is free software: you can redistribute it and/or modify it
under the terms of the GNU Lesser General Public License as published by the
Free Software Foundation, either version 3 of the License, or (at your option)
any later version.

MinIMU-9-Arduino-AHRS is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for
more details.

You should have received a copy of the GNU Lesser General Public License along
with MinIMU-9-Arduino-AHRS. If not, see <http://www.gnu.org/licenses/>.

*/
#include <plib.h>
// Uncomment the below line to use this axis definition: 
   // X axis pointing forward
   // Y axis pointing to the right 
   // and Z axis pointing down.
// Positive pitch : nose up
// Positive roll : right wing down
// Positive yaw : clockwise
//int SENSOR_SIGN[9] = {1,1,1,-1,-1,-1,1,1,1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer
// Uncomment the below line to use this axis definition: 
   // X axis pointing forward
   // Y axis pointing to the left 
   // and Z axis pointing up.
// Positive pitch : nose down
// Positive roll : right wing down
// Positive yaw : counterclockwise
int SENSOR_SIGN[9] = {1,-1,-1,-1,1,1,1,-1,-1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer

// tested with Arduino Uno with ATmega328 and Arduino Duemilanove with ATMega168

#include <Wire.h>
#include <PID_v1.h>

#define LOOP_FREQ 100  //choose 50,100, or 200
#define LOOP_TIME 1000/LOOP_FREQ  //in ms
// LSM303 accelerometer: 8 g sensitivity
// 3.8 mg/digit; 1 g = 256
#define GRAVITY 256  //this equivalent to 1G in the raw data coming from the accelerometer 

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

// L3G4200D gyro: 2000 dps full scale
// 70 mdps/digit; 1 dps = 0.07
#define Gyro_Gain_X 0.07 //X axis Gyro gain
#define Gyro_Gain_Y 0.07 //Y axis Gyro gain
#define Gyro_Gain_Z 0.07 //Z axis Gyro gain
#define Gyro_Scaled_X(x) ((x)*ToRad(Gyro_Gain_X)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) ((x)*ToRad(Gyro_Gain_Y)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) ((x)*ToRad(Gyro_Gain_Z)) //Return the scaled ADC raw data of the gyro in radians for second


#define Kp_ROLLPITCH 0.02
#define Ki_ROLLPITCH 0.00002
#define Kp_YAW 1.2
#define Ki_YAW 0.00002



int gyro_x;
int gyro_y;
int gyro_z;
int accel_x;
int accel_y;
int accel_z;


// LSM303 magnetometer calibration constants; use the Calibrate example from
// the Pololu LSM303 library to find the right values for your board
//#define M_X_MIN -796
//#define M_Y_MIN -457
//#define M_Z_MIN -424
//#define M_X_MAX 197
//#define M_Y_MAX 535
//#define M_Z_MAX 397




//#define STATUS_LED 13 

float G_Dt=1.0/LOOP_FREQ;  // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

long timer=0;   //general purpuse timer
long timer_old;

int AN[6]; //array that stores the gyro and accelerometer data
int AN_OFFSET[6]={0,0,0,0,0,0}; //Array that stores the Offset of the sensors

int throttle = 0;

// Euler angles
float roll;
float pitch;
//float yaw;
int yaw=0;

float D_roll = 0;
float D_pitch = 0;
float D_yaw = 0;

float E_roll;
float E_pitch;
float E_yaw;



unsigned int counter=0;
unsigned int rangeCounter=0;
long int Range=0;

//byte gyro_sat=0;


//Define the aggressive and conservative Tuning Parameters
float aggKp=4, aggKi=0.2, aggKd=1;
//float consKp=.5, consKi=0.025, consKd=0.125;

  float consKp=1.0, consKi=0.00, consKd=0.00;
  float initKp=1.0, initKi=0.00, initKd=0.00;

//float initKp=.5, initKi=0.025, initKd=0.125;



//Specify the links and initial tuning parameters
PID RollPID(&roll, &E_roll, &D_roll, initKp, initKi, initKd, DIRECT);
PID PitchPID(&pitch, &E_pitch, &D_pitch, initKp, initKi, initKd, DIRECT);


void setup()
{ 
  Serial.begin(115200);
  
  
  //pinMode (STATUS_LED,OUTPUT);  // Status LED
  
  I2C_Init();


  Serial.println("Pololu MinIMU-9 + Arduino AHRS");
  //digitalWrite(STATUS_LED,LOW);
  delay(1500);
  
  Gyro_Init();
  Accel_Init();
//  Compass_Init();
  
  delay(20);
  Serial.println("Offset:");
  for(int i=0;i<64;i++)    // We take some readings...
    {
    Read_Accel();
     
    Read_Gyro();
    
    
    for(int y=0; y<6; y++)   // Cumulate values
      AN_OFFSET[y] += AN[y];
    delay(20);
    
    }
    
  for(int y=0; y<6; y++)
    AN_OFFSET[y] = AN_OFFSET[y]/64;
    
  AN_OFFSET[5]-=GRAVITY*SENSOR_SIGN[5];
  
  
//  for(int y=0; y<6; y++)
//    Serial.println(AN_OFFSET[y]);
  
//  delay(2000);
//  digitalWrite(STATUS_LED,HIGH);
    

  delay(20);
  counter=0;
  initPID();  //Initialize PID
  radio_init(); // Initialize radio (serial1)
//  range_init();  //Initialize Range finder (PWM 5)
  ArmMotor();    //Initialize Motor signal (PWM 1-4)
  start_GDt_Clock();  //Initilize update clock (Timer 4 and 5 in 32 bit mode)
 
  timer=micros();

}

void loop() //Main Loop
{
  radio_check();
  if(IFS0&0x00100000)  // Main loop runs at LOOP_FREQ **see top of code to set (use 50/100/200 for good results)
  {
    IFS0CLR = 0x00100000;
    counter++;
    rangeCounter++;
    timer_old = timer;
    timer=micros();

    G_Dt = (timer-timer_old)/1000000.0;    // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
 //   G_Dt = LOOP_TIME/1000.0;    // Use when loop time is very consistant to avoid math or extra overhead.
   
    // *** DCM algorithm
    // Data adquisition
    Read_Gyro();   // This read gyro data
    Read_Accel();     // Read I2C accelerometer
    
    if (counter > (LOOP_FREQ/50))  // Read compass data at 50Hz... (5 loop runs)
      {
      counter=0;
 //     Read_Compass();    // Read I2C magnetometer
 //     Compass_Heading(); // Calculate magnetic heading  
      
      updatePID();
      updateMotor();   


    }
    if (rangeCounter > (LOOP_FREQ/10))  // Read Range data at 10Hz... (10 loop runs)
      {
        rangeCounter = 0;
        range_start();
        //add code for GPS and BMP085 here!!!!
       // Serial.println("PING...");
      }
    if(range_available()){  //Is there a captured time to get from range finder?
      Range = range_get();
      //Serial.print("Range = ");
      //Serial.println(Range);
    }
    
  if(counter == 0){
     // Serial.println(G_Dt,9);
      Serial.print(roll);
      Serial.print(", ");
      Serial.print(D_roll);
       Serial.print("      ");
      Serial.print(pitch);
      Serial.print(", ");
      Serial.print(D_pitch);
      Serial.print("      ");
      Serial.print(throttle);
      Serial.print("      ");
      Serial.println(E_pitch);
    
  }
//    MadgwickAHRSupdate( Gyro_Scaled_X(gyro_x), Gyro_Scaled_X(gyro_y), Gyro_Scaled_X(gyro_z), accel_x, accel_y,  accel_z, c_magnetom_x,  c_magnetom_y, c_magnetom_z);
      MadgwickAHRSupdateIMU(Gyro_Scaled_X(gyro_x), Gyro_Scaled_X(gyro_y), Gyro_Scaled_X(gyro_z),accel_x, accel_y, accel_z);

    // DCM Calculations...
//    Matrix_update(); 
//    Normalize();
//    Drift_correction();
//    Euler_angles();
   

  }
   
}
