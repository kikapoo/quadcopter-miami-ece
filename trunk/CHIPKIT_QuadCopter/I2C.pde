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

#define FILTER_ACCEL 0
#define FILTER_SIZE 10

#if FILTER_ACCEL == 1
float filter_X[FILTER_SIZE];
float filter_Y[FILTER_SIZE];
float filter_Z[FILTER_SIZE];
float sum_X,sum_Y,sum_Z;
#endif

int FC = 0;

void I2C_Init()
{
  Wire.begin();
}

byte Gyro_Init()
{
  gyro.writeReg(L3G4200D_CTRL_REG4, 0x20); // 2000 dps full scale
  gyro.writeReg(L3G4200D_CTRL_REG2, 0x09); // H.P. Freq selection @100=0.01Hz  @200=0.02Hz @400=0.05Hz @800=.1Hz

  //gyro.writeReg(L3G4200D_CTRL_REG5, 0x13); // Allow LOW Pass Filtering 2 and High Pass Filter to remove drift 
  //gyro.writeReg(L3G4200D_CTRL_REG5, 0x53); // Allow LOW Pass Filtering 2 and High Pass Filter to remove drift  FIFO Enables 
  gyro.writeReg(L3G4200D_CTRL_REG5, 0x42); // Allow LOW Pass Filtering 2 FIFO Enabled

  gyro.writeReg(L3G4200D_FIFO_CTRL_REG, 0x40); // FIFO in STREAM mode

  

  #if SAMPLE_FREQ == 100
  gyro.writeReg(L3G4200D_CTRL_REG1, 0x0F); // normal power mode, all axes enabled, 100 Hz
  #endif

  #if SAMPLE_FREQ == 200
  gyro.writeReg(L3G4200D_CTRL_REG1, 0x4F); // normal power mode, all axes enabled, 200 Hz
  #endif

  #if SAMPLE_FREQ == 400
  gyro.writeReg(L3G4200D_CTRL_REG1, 0x8F); // normal power mode, all axes enabled, 400 Hz 20 BW
  #endif

  #if SAMPLE_FREQ == 800
  gyro.writeReg(L3G4200D_CTRL_REG1, 0xCF); // normal power mode, all axes enabled, 800 Hz 20 BW
  #endif
}


// Reads x,y and z accelerometer registers
inline void Read_Accel()
{
  compass.readAccel(&AN[3],&AN[4] ,&AN[5]);  
#if FILTER_ACCEL == 1
  sum_X = (sum_X - filter_X[FC]) + AN[3];
  filter_X[FC] = AN[3];

  sum_Y = (sum_Y - filter_Y[FC]) + AN[4];
  filter_Y[FC] = AN[4];

  sum_Z = (sum_Z - filter_Z[FC]) + AN[5];
  filter_Z[FC] = AN[5];

  AN[3] = sum_X/FILTER_SIZE;
  AN[4] = sum_Y/FILTER_SIZE;
  AN[5] = sum_Z/FILTER_SIZE;
  
  #endif

  accel_x = AN[3];
  accel_y = AN[4];
  accel_z = AN[5];

  #if FILTER_ACCEL == 1
  FC = (FC + 1)%FILTER_SIZE;  
  #endif
}


inline void Update_Matrix(void){
  byte FIFOcnt;
    
  union RX_data {
    byte buf[6*32];
    int16_t Data[32][3];
  }
  FIFO_data;

  //Check for noise on accel--------
  #if ACCEL_NOISE_CHECK == 1
  int mag = sqrt(accel_x*accel_x+accel_y*accel_y+accel_z*accel_z);
  //Gravity with in tolerance
  if((mag>(1.5*GRAVITY)) || (mag<(.5*GRAVITY)))
    accel_x=accel_y=accel_z=0.0;
  //Gravity in Z direction has most weight
  if((accel_z/mag)<.5)
      accel_x=accel_y=accel_z=0.0;  
  #endif
  //--------------------------------

  FIFOcnt = gyro.readFIFOdepth();
  gyro.readFIFO(FIFO_data.buf, FIFOcnt);
  
  for(int i=0; i<FIFOcnt;i++){
    gyro_x = (FIFO_data.Data[i][0]-AN_OFFSET[0])*Gyro_Gain_Rad;//0.07*0.01745329252; 
    gyro_y = (FIFO_data.Data[i][1]-AN_OFFSET[1])*Gyro_Gain_Rad;//0.07*0.01745329252; 
    gyro_z = (FIFO_data.Data[i][2]-AN_OFFSET[2])*Gyro_Gain_Rad;//0.07*0.01745329252; 
    if(i==0)
      MadgwickAHRSupdateIMU(gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z);
    else
      MadgwickAHRSupdateGyroIMU(gyro_x,gyro_y,gyro_z);
  }

  //Source
  //http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  roll = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2));
  pitch =  asin(2*(q0*q2-q1*q3));
  yaw = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));


}

void collect_offsets(void){
  union RX_data {
    byte buf[6*32];
    int16_t Data[32][3];
  }
  FIFO_data;
  
  float FIFOcnt;
  float total=0;
  float AccTotal=0;
  
  while(total<50000){
    Read_Accel();
    FIFOcnt = gyro.readFIFOdepth();
    gyro.readFIFO(FIFO_data.buf, FIFOcnt);
    Serial.println((int)total);
   
    for(int i=0; i<FIFOcnt;i++){
      AN_OFFSET[0] += FIFO_data.Data[i][0]; 
      AN_OFFSET[1] += FIFO_data.Data[i][1]; 
      AN_OFFSET[2] += FIFO_data.Data[i][2]; 

    }
    total += FIFOcnt;
    AN_OFFSET[3] += AN[3];
    AN_OFFSET[4] += AN[4];
    AN_OFFSET[5] += AN[5];
    AccTotal++;
    delay(20);
  }
  AN_OFFSET[0] /= total; 
  AN_OFFSET[1] /= total; 
  AN_OFFSET[2] /= total; 

  AN_OFFSET[3] /= AccTotal; 
  AN_OFFSET[4] /= AccTotal; 
  AN_OFFSET[5] /= AccTotal; 

}
