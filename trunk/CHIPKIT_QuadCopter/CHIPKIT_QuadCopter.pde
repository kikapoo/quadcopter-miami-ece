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
#include <L3G4200D.h>
//#include <LSM303.h>
#include <ADXL345.h>
L3G4200D gyro;
//LSM303 compass;
ADXL345 compass;
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
int SENSOR_SIGN[9] = {
  1,-1,-1,-1,1,1,1,-1,-1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer

// tested with Arduino Uno with ATmega328 and Arduino Duemilanove with ATMega168

#include <Wire.h>
#include <PID_v1.h>

#define FILTER_ROLL_PITCH 0

#define LOOP_FREQ 50  //choose 50,100, or 200
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



float gyro_x;
float gyro_y;
float gyro_z;
float accel_x;
float accel_y;
float accel_z;
float temp;

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
float AN_OFFSET[6]={0,0,0,0,0,0}; //Array that stores the Offset of the sensors

int throttle = 0;
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame

// Euler angles
float roll;
float pitch;
float yaw;
//int yaw=0;

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

float consKp=1.5, consKi=0.00, consKd=0.3500;
float initKp=.9, initKi=1.5, initKd=.135;

//float initKp=.5, initKi=0.025, initKd=0.125;

#if FILTER_ROLL_PITCH
#define FILTER_SIZE_2 5
float filter_Roll[FILTER_SIZE_2];
float filter_Pitch[FILTER_SIZE_2];
float sum_Roll,sum_Pitch;
int FC1 = 0;
#endif


//Specify the links and initial tuning parameters
PID RollPID(&roll, &E_roll, &D_roll, initKp, initKi, initKd, DIRECT);
PID PitchPID(&pitch, &E_pitch, &D_pitch, initKp, initKi, initKd, DIRECT);


void setup()
{ 
  Serial.begin(115200);

  I2C_Init();
  Serial.println("Senior Design Quadcopter");
  delay(1500);

  Gyro_Init();
 // Accel_Init();
  //  Compass_Init();
  compass.powerOn();
  compass.set_bw(ADXL345_BW_12);
  compass.setAxisOffset(0, 0, 0);
  delay(20);
  Serial.println("Offsets: Please wait 1 minute.");
  collect_offsets();
  compass.setAxisOffset(-AN_OFFSET[3]/4, -AN_OFFSET[4]/4,-(AN_OFFSET[5]-256)/4);
  Serial.println("Offsets=");
  for(int i = 0; i<6;i++)
    Serial.println(AN_OFFSET[i],9);

//AN_OFFSET[0]=12.512920380;
//AN_OFFSET[1]=-10.521800041;
//AN_OFFSET[2]=-4.051559925;
//AN_OFFSET[3]=9.877433777;
//AN_OFFSET[4]=-11.827033043;
//AN_OFFSET[5]=-10.572357178;


  initPID();  //Initialize PID
  radio_init(); // Initialize radio (serial1)
  //  range_init();  //Initialize Range finder (PWM 5)
  ArmMotor();    //Initialize Motor signal (PWM 1-4)
  start_GDt_Clock();  //Initilize update clock (Timer 4 and 5 in 32 bit mode)

  G_Dt = 1.0/800.0;
}

void loop() //Main Loop
{
  radio_check();
  if(IFS0&0x00100000)  // Main loop runs at LOOP_FREQ **see top of code to set (use 50/100/200 for good results)
  {
    IFS0CLR = 0x00100000;
    //    rangeCounter++;

    // *** DCM algorithm
    // Data adquisition
    //     Read_Compass();    // Read I2C magnetometer
    //     Compass_Heading(); // Calculate magnetic heading  

    Read_Accel();     // Read I2C accelerometer
    Update_Matrix();

    updatePID();
    updateMotor();   


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
      //      Serial.print(roll);
      //      Serial.print(", ");
      //      Serial.print(D_roll);
      //       Serial.print("      ");
      Serial.print("!");
      Serial.print("ANG:");
      Serial.print(roll,3);
      Serial.print(",");
      Serial.print(pitch,3);
      Serial.print(",");
      Serial.print(yaw,3);
      Serial.print(",");
      Serial.print(D_roll,3);
      Serial.print(",");
      Serial.print(D_pitch,3);
      Serial.print(",");
      Serial.print(throttle);
      Serial.print(",");
      Serial.print(E_pitch);
      Serial.print(",");
      // Serial.print(" M ");  
      //  Serial.print("X: ");
      //  Serial.print(AN[3]);
      //  Serial.print(" Y: ");
      //  Serial.print(AN[4]);
      //  Serial.print(" Z: ");
      //  Serial.print(AN[5]);
      //  
      //  Serial.print(" G ");
      //  Serial.print("X: ");
      //  Serial.print(AN[0]);
      //  Serial.print(" Y: ");
      //  Serial.print(AN[1]);
      //  Serial.print(" Z: ");
      //  Serial.println(AN[2]);
      Serial.println(temp);
      
    }


  }

}

