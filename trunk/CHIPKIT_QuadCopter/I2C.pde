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
#define FILTER_GYRO 0
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

  //gyro.writeReg(L3G4200D_CTRL_REG1, 0x0F); // normal power mode, all axes enabled, 100 Hz
  //gyro.writeReg(L3G4200D_CTRL_REG1, 0x8F); // normal power mode, all axes enabled, 400 Hz 20 BW
  //gyro.writeReg(L3G4200D_CTRL_REG1, 0x4F); // normal power mode, all axes enabled, 200 Hz
  gyro.writeReg(L3G4200D_CTRL_REG1, 0xCF); // normal power mode, all axes enabled, 800 Hz 20 BW

}

//void Accel_Init()
//{
//  compass.writeAccReg(LSM303_CTRL_REG4_A, 0x30); // 8 g full scale
//  //  compass.writeAccReg(LSM303_CTRL_REG2_A, 0x10); // Enable H.P. filter @50=1Hz @100=2Hz @400=8Hz @1000=20Hz
//
//  compass.writeAccReg(LSM303_CTRL_REG1_A, 0x27); // normal power mode, all axes enabled, 50 Hz
//  // compass.writeAccReg(LSM303_CTRL_REG1_A, 0x2F); // normal power mode, all axes enabled, 100 Hz
//  //compass.writeAccReg(LSM303_CTRL_REG1_A, 0x37); // normal power mode, all axes enabled, 400 Hz
//
//}

// Reads x,y and z accelerometer registers
inline void Read_Accel()
{
  //compass.readAcc();
#if FILTER_ACCEL == 1
  compass.readAcc();
  sum_X = (sum_X - filter_X[FC]) + compass.a.x;
  filter_X[FC] = compass.a.x;

  sum_Y = (sum_Y - filter_Y[FC]) + compass.a.y;
  filter_Y[FC] = compass.a.y;

  sum_Z = (sum_Z - filter_Z[FC]) + compass.a.z;
  filter_Z[FC] = compass.a.z;

  AN[3] = sum_X/FILTER_SIZE;
  AN[4] = sum_Y/FILTER_SIZE;
  AN[5] = sum_Z/FILTER_SIZE;
  
  accel_x = SENSOR_SIGN[3] * (AN[3] - AN_OFFSET[3]);
  accel_y = SENSOR_SIGN[4] * (AN[4] - AN_OFFSET[4]);
  accel_z = SENSOR_SIGN[5] * (AN[5] - AN_OFFSET[5]);

  #else
  compass.readAccel(&AN[3],&AN[4] ,&AN[5]);  
//  AN[3] = compass.a.x;
//  AN[4] = compass.a.y;
//  AN[5] = compass.a.z;
  
  #endif

  accel_x = SENSOR_SIGN[3] * (AN[3]);
  accel_y = SENSOR_SIGN[4] * (AN[4]);
  accel_z = SENSOR_SIGN[5] * (AN[5]);
  
  #if FILTER_ACCEL == 1
  FC = (FC + 1)%FILTER_SIZE;  
  #endif
}

//void Compass_Init()
//{
//  compass.init();
//  compass.writeMagReg(LSM303_MR_REG_M, 0x00); // continuous conversion mode
//  compass.writeMagReg(LSM303_CRA_REG_M, 0x1C); // 75 Hz
//
//  // 15 Hz default
//}

inline void Update_Matrix(void){
  byte FIFOcnt;
    
  union RX_data {
    byte buf[6*32];
    int16_t Data[32][3];
  }
  FIFO_data;


  FIFOcnt = gyro.readFIFOdepth();
  gyro.readFIFO(FIFO_data.buf, FIFOcnt);
  //  Serial.println((int)FIFOcnt);
  for(int i=0; i<FIFOcnt;i++){
    gyro_x = (FIFO_data.Data[i][0]-AN_OFFSET[0])*0.00122173047639603070384658353794;//0.07*0.01745329252; //gyro.g.x;
    gyro_y = (FIFO_data.Data[i][1]-AN_OFFSET[1])*0.00122173047639603070384658353794;//0.07*0.01745329252; //gyro.g.y;
    gyro_z = (FIFO_data.Data[i][2]-AN_OFFSET[2])*0.00122173047639603070384658353794;//0.07*0.01745329252; //gyro.g.z;
    if(i==0)
      MadgwickAHRSupdateIMU(gyro_x,-gyro_y,-gyro_z,accel_x,accel_y,accel_z);
    else
      MadgwickAHRSupdateIMU(gyro_x,-gyro_y,-gyro_z,0.0,0.0,0.0);
  }

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
   Serial.println((int)FIFOcnt);
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
  
  AN_OFFSET[5]-=GRAVITY*SENSOR_SIGN[5];
}


//void Read_Compass()
//{
//  compass.readMag();
//  
//  magnetom_x = SENSOR_SIGN[6] * compass.m.x;
//  magnetom_y = SENSOR_SIGN[7] * compass.m.y;
//  magnetom_z = SENSOR_SIGN[8] * compass.m.z;
//}



