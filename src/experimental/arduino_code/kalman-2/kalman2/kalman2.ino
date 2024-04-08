#include <Wire.h>
#include "KalmanFilter.h"

Kalman kalman_x;//, kalman_y;
int cal_int=0, refresh_rate = 4000;
double gyro_x, gyro_y, gyro_z, gyro_x_cal, gyro_z_cal, gyro_y_cal;
double acc_x, acc_y, acc_z;
int comp_x, comp_y, comp_z;
double angle_x, angle_y, angle_z;
byte set_angles=0;
double acc_angle_x, acc_angle_y, acc_angle_z, total_vector;
long timer;

void setup() 
{
  Serial.begin(115200);
  Wire.begin();
  Serial.println("Setting up gyro registers...");
  setup_reg();
  Serial.print("Calibrating the gyro ");
  calibrate();
  Serial.println("Calibrated ");
  kalman_x.init_processCov(0.04,0.0004);
  kalman_x.init_measureCov(0.0246,0.000254);  
  filter();
  timer = micros();
}

void loop()
{
 while(micros() - timer < refresh_rate);
 timer = micros();
 read_data();
 filter();
 /*Serial.print(kalman_x.K[0][0]);
 Serial.print(" , ");
 Serial.print(kalman_x.K[1][1]);*/
 kalman_x.kalmanOutput(acc_angle_x, gyro_x);
 Serial.print(kalman_x.new_X[0]);
 Serial.print(" , ");
 Serial.print(kalman_x.z[0]);
 Serial.print(" , ");
 Serial.print(kalman_x.new_X[1]);
 Serial.print(" , ");
 Serial.print(kalman_x.z[1]);
 Serial.println("");
 }

void setup_reg()
{
  //Set full scale to 2000 dps and sampling rate to 8kHz.
  Wire.beginTransmission(0x68);
  Wire.write(0x16);
  Wire.write(0x18);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x3E);
  Wire.write(0x03);
  Wire.endTransmission();

  //write to power control reg //00001000
  Wire.beginTransmission(0x53);
  Wire.write(0x2D);
  Wire.write(0x08);
  Wire.endTransmission();
  //write to bandwidth/rate reg //00001100
  Wire.beginTransmission(0x53);
  Wire.write(0x2C);
  Wire.write(0x0C);
  Wire.endTransmission();
  //write to data format control reg //00001111
  Wire.beginTransmission(0x53);
  Wire.write(0x31);
  Wire.write(0x0F);
  Wire.endTransmission();
    
  //CRA7 //01111000
  Wire.beginTransmission(0x1E);
  Wire.write(0x00);
  Wire.write(0x78);
  Wire.endTransmission();
  //MODE reg //00000000
  Wire.beginTransmission(0x1E);
  Wire.write(0x02);
  Wire.write(0x00);
  Wire.endTransmission();
}

void calibrate()
{
  for(cal_int=0; cal_int<2000; cal_int++)
  {
    if(cal_int%125==0)Serial.print(".");
    read_data();
    gyro_x_cal+=gyro_x;
    gyro_y_cal+=gyro_y;
    gyro_z_cal+=gyro_z;
  }
  gyro_x_cal/=2000;
  gyro_y_cal/=2000;
  gyro_z_cal/=2000;
  delay(5);
}


void read_data()
{
  Wire.beginTransmission(0x68);
  Wire.write(0x1D);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  while(Wire.available()<6);
  gyro_x = Wire.read()<<8|Wire.read();
  gyro_y = Wire.read()<<8|Wire.read();
  gyro_z = Wire.read()<<8|Wire.read();
  if(cal_int==2000)
  {
    gyro_x=(gyro_x-gyro_x_cal)/14.375;
    gyro_y=(gyro_y-gyro_y_cal)/14.375;
    gyro_z=(gyro_z-gyro_z_cal)/14.375;
  }

  Wire.beginTransmission(0x53);
  Wire.write(0x32);
  Wire.endTransmission();
  Wire.requestFrom(0x53,6);
  while(Wire.available()<6);
  acc_x = Wire.read()|(Wire.read()<<8);
  acc_y = Wire.read()|(Wire.read()<<8);
  acc_z = Wire.read()|(Wire.read()<<8);
  acc_x -= -58;
  acc_y -= 11;
  
  /*Wire.beginTransmission(0x1E);
  Wire.write(0x03);
  Wire.endTransmission();
  Wire.requestFrom(0x1E,6);
  while(Wire.available()<6);
  comp_x = Wire.read()<<8|Wire.read();
  comp_z = Wire.read()<<8|Wire.read();
  comp_y = Wire.read()<<8|Wire.read();*/
}

void filter()
{
  
  total_vector = sqrt(pow(abs(acc_x),2) + pow(abs(acc_y),2) + pow(abs(acc_z),2));
  if(abs(acc_y) < total_vector)acc_angle_x = asin((float)acc_y/total_vector)* 57.296;       
  if(abs(acc_x) < total_vector)acc_angle_y = asin((float)acc_x/total_vector)* -57.296;       
  if(set_angles==0)
  {
    kalman_x.init_main(refresh_rate, acc_angle_x, 0);
    //kalman_y.init_main(refresh_rate, acc_angle_y, 0);
    Serial.println("INITIALIZED ANGLES");
    set_angles = 1;
  }
}
