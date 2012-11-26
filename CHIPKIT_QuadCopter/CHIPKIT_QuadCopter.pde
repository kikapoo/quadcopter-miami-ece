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
//#include <ADXL345.h>
#include <LSM303.h>
#include <Wire.h>
#include <PID_v1.h>

L3G4200D gyro;
//ADXL345 compass;
LSM303 compass;

#define PRINT_OUTPUTS 1
#define TUNPID  0

#define ACCEL_NOISE_CHECK 1 //Checks the magnitude of accel and ensures it is within tolerance

#define SAMPLE_FREQ	800 // sample frequency of Gyro in Hz (use 100, 200, 400, 800)
#define betaDef		0.07f    // 2 * proportional gain wieght of Accel for IMU update
#define G_Dt 1.0f/SAMPLE_FREQ   // Gyro Sample time 
float halfT = G_Dt/2;

#define LOOP_FREQ 50  //choose 50,100, or 200 Hz
#define LOOP_TIME 1000/LOOP_FREQ  //Loop update time in ms
#define PID_BOUND 2.0f  //+/- PID limit for Roll and Pitch PIDs
// LSM303 accelerometer: 8 g sensitivity
// 3.8 mg/digit; 1 g = 256
#define GRAVITY 256  //this equivalent to 1G in the raw data coming from the accelerometer 

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

// L3G4200D gyro: 2000 dps full scale
// 70 mdps/digit; 1 dps = 0.07
#define Gyro_Gain 0.07 //Gyro gain
#define Gyro_Gain_Rad  Gyro_Gain*0.01745329252
int SENSOR_SIGN[9] = {1,-1,-1,1,1,1,1,1,1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer
//int SENSOR_SIGN[9] = {1,-1,-1,-1,1,1,1,-1,-1};

float gyro_x;
float gyro_y;
float gyro_z;
float accel_x;
float accel_y;
float accel_z;

float P_TERM;  // Used in varying Kp for Zeigler-Nichols method for tunning PID

int AN[6]; //array that stores the gyro and accelerometer data
float AN_OFFSET[9]={
  0,0,0,0,0,0}; //Array that stores the Offset of the sensors

volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame

// Euler angles of Attitude calculation
float roll;
float pitch;
float yaw;

int throttle = 0;

float D_roll = 0;   //Desired roll from Xmitter
float D_pitch = 0;  //Desired pitch from Xmitter
float D_yaw = 0;    //Desired yaw from Xmitter

float E_roll;       //Error in roll for PID
float E_pitch;      //Error in pitch for PID
float E_yaw;        //Error in yaw for PID

unsigned int counter=0;
unsigned int rangeCounter=0;  //Counter for deciding when the next range should be calculated
long int Range=0;            //Range value from Ultrasonic sensor

//Define the aggressive and conservative Tuning Parameters
float aggKp=4, aggKi=0.2, aggKd=1;
float consKp=1.7, consKi=1.00, consKd=0.7000;
//float initKp=0.9, initKi=1.50, initKd=.135; //Ki=1.5, 
float initKp=2.1, initKi=1.0, initKd=.9; //Ki=1.5, 

//Specify the links and initial tuning parameters
PID RollPID(&roll, &E_roll, &D_roll, initKp, initKi, initKd, DIRECT);
PID PitchPID(&pitch, &E_pitch, &D_pitch, consKp, consKi, consKd, DIRECT);
PID YawPID(&yaw, &E_yaw, &D_yaw, consKp, consKi, consKd, DIRECT);


void setup()
{ 
  Serial.begin(115200);

  I2C_Init();
  Serial.println("Senior Design Quadcopter");
  delay(1500);

  Gyro_Init();    //Initialize Gyro
  Accel_Init();
 // compass.powerOn();  //Initialize Accel
//  compass.set_bw(ADXL345_BW_12);
  //Clear the internal offset registers of the accelerometer
//  compass.setAxisOffset(0, 0, 0);
  delay(20);
  Serial.println("Offsets: Please wait 1 minute.");
  collect_offsets();
  //Write new values of offsets to Accel offset registers
 // compass.setAxisOffset(-AN_OFFSET[3]/4, -AN_OFFSET[4]/4,-(AN_OFFSET[5]-GRAVITY)/4);

  Serial.println("Offsets=");
  for(int i = 0; i<6;i++)
    Serial.println(AN_OFFSET[i],9);

  initPID();  //Initialize PID
  radio_init(); // Initialize radio (serial1)
  //  range_init();  //Initialize Range finder (PWM 5)
  ArmMotor();    //Initialize Motor signal (PWM 1-4)
  start_GDt_Clock();  //Initilize update clock (Timer 4 and 5 in 32 bit mode)
  initializeKinematics();

  //  G_Dt = 1.0/sampleFreq;
}

void loop() //Main Loop
{
  radio_check();
  if(IFS0&0x00100000)  // Main loop runs at LOOP_FREQ **see top of code to set (use 50/100/200 for good results)
  {
    IFS0CLR = 0x00100000;
    //    rangeCounter++;

    // *** DCM algorithm
    Read_Accel();     // Read I2C accelerometer
    Update_Matrix();  //Read Gryo and update Quaternions

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
#if PRINT_OUTPUTS == 1
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
//       Serial.print(" M ");  
//        Serial.print("X: ");
//        Serial.print(AN[3]);
//        Serial.print(" Y: ");
//        Serial.print(AN[4]);
//        Serial.print(" Z: ");
//        Serial.print(AN[5]);
      //  
      //  Serial.print(" G ");
      //  Serial.print("X: ");
      //  Serial.print(AN[0]);
      //  Serial.print(" Y: ");
      //  Serial.print(AN[1]);
      //  Serial.print(" Z: ");
      //  Serial.println(AN[2]);
      Serial.print(",");
      Serial.println(P_TERM);

    }
#endif

  }

}


