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

#define FILTER_ACCEL 1
#define FILTER_GYRO 1
#define FILTER_SIZE 20

#if FILTER_ACCEL == 1
int filter_X[FILTER_SIZE];
int filter_Y[FILTER_SIZE];
int filter_Z[FILTER_SIZE];
int sum_X,sum_Y,sum_Z;
#endif

#if FILTER_GYRO == 1
int filter_GX[FILTER_SIZE];
int filter_GY[FILTER_SIZE];
int filter_GZ[FILTER_SIZE];
int sum_GX,sum_GY,sum_GZ;
#endif

int FC = 0;


void I2C_Init()
{
  Wire.begin();
}

void Gyro_Init()
{
  gyro.writeReg(L3G4200D_CTRL_REG4, 0x20); // 2000 dps full scale
 gyro.writeReg(L3G4200D_CTRL_REG2, 0x09); // H.P. Freq selection @100=0.01Hz  @200=0.02Hz @400=0.05Hz @800=.1Hz
 gyro.writeReg(L3G4200D_CTRL_REG5, 0x13); // Allow LOW Pass Filtering 2 and High Pass Filter to remove drift 

 //gyro.writeReg(L3G4200D_CTRL_REG5, 0x02); // Allow LOW Pass Filtering 2 

  gyro.writeReg(L3G4200D_CTRL_REG1, 0x0F); // normal power mode, all axes enabled, 100 Hz
  //gyro.writeReg(L3G4200D_CTRL_REG1, 0x8F); // normal power mode, all axes enabled, 400 Hz 20 BW
  //gyro.writeReg(L3G4200D_CTRL_REG1, 0x4F); // normal power mode, all axes enabled, 200 Hz


}

void Read_Gyro()
{
  gyro.read();
  #if FILTER_GRYRO == 1
  sum_GX = (sum_GX - filter_GX[FC]) + gyro.g.x;
  filter_GX[FC] = gyro.g.x;
  
  sum_GY = (sum_GY - filter_GY[FC]) + gyro.g.y;
  filter_GY[FC] = gyro.g.y;
  
  sum_GZ = (sum_GZ - filter_GZ[FC]) + gyro.g.z;
  filter_GZ[FC] = gyro.g.z;
  
  AN[0] = sum_GX/FILTER_SIZE;
  AN[1] = sum_GY/FILTER_SIZE;
  AN[2] = sum_GZ/FILTER_SIZE;
  #else
  AN[0] = gyro.g.x;
  AN[1] = gyro.g.y;
  AN[2] = gyro.g.z;
  #endif
  gyro_x = SENSOR_SIGN[0] * (AN[0] - AN_OFFSET[0]);
  gyro_y = SENSOR_SIGN[1] * (AN[1] - AN_OFFSET[1]);
  gyro_z = SENSOR_SIGN[2] * (AN[2] - AN_OFFSET[2]);
}

void Accel_Init()
{
  compass.writeAccReg(LSM303_CTRL_REG4_A, 0x30); // 8 g full scale
//  compass.writeAccReg(LSM303_CTRL_REG2_A, 0x10); // Enable H.P. filter @50=1Hz @100=2Hz @400=8Hz @1000=20Hz

 // compass.writeAccReg(LSM303_CTRL_REG1_A, 0x27); // normal power mode, all axes enabled, 50 Hz
  compass.writeAccReg(LSM303_CTRL_REG1_A, 0x2F); // normal power mode, all axes enabled, 100 Hz
  //compass.writeAccReg(LSM303_CTRL_REG1_A, 0x37); // normal power mode, all axes enabled, 400 Hz

}

// Reads x,y and z accelerometer registers
void Read_Accel()
{
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
  
  #else
  
  AN[3] = compass.a.x;
  AN[4] = compass.a.y;
  AN[5] = compass.a.z;
  
  #endif

  accel_x = SENSOR_SIGN[3] * (AN[3] - AN_OFFSET[3]);
  accel_y = SENSOR_SIGN[4] * (AN[4] - AN_OFFSET[4]);
  accel_z = SENSOR_SIGN[5] * (AN[5] - AN_OFFSET[5]);
  #if FILTER_ACCEL == 1
  FC = (FC + 1)%FILTER_SIZE;  
  #endif
}

void Compass_Init()
{
  compass.init();
  compass.writeMagReg(LSM303_MR_REG_M, 0x00); // continuous conversion mode
  compass.writeMagReg(LSM303_CRA_REG_M, 0x1C); // 75 Hz

  // 15 Hz default
}

//void Read_Compass()
//{
//  compass.readMag();
//  
//  magnetom_x = SENSOR_SIGN[6] * compass.m.x;
//  magnetom_y = SENSOR_SIGN[7] * compass.m.y;
//  magnetom_z = SENSOR_SIGN[8] * compass.m.z;
//}

