#include "KalmanFilterTOPTHL.h"
#include <Wire.h>

#define Q_angle 0.000004    //0.000004        //0.00000135
#define Q_bias  0.0004      //0.0004       //0.183
#define time_us 4000
#define R_angle 0.93        //0.93    //0.028

int gyro_x, gyro_y, gyro_z;
//int comp_x, comp_y, comp_z;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
long acc_x, acc_y, acc_z;
double total_vector;
double angle_x, angle_y;
double acc_angle_x, acc_angle_y;
byte set_angles = 0;
int cal_int;
long timer;
KalmanFilter KalmanX, KalmanY;

void setup(){
	Serial.begin(57600);
	Wire.begin();
	Serial.println("Setting up gyro registers...");
    setup_reg();
    Serial.print("Calibrating the gyro ");
    calibrate();
    Serial.println("Calibrated ");
    read_data();
    filter();

	KalmanX.init_main(time_us, angle_x);
	//KalmanY.init_main(time_us, angle_y);
  KalmanX.init_processCov(Q_angle, Q_bias);
  //KalmanY.init_processCov(Q_angle, Q_bias);
  KalmanX.init_measureCov(R_angle);
  //KalmanY.init_measureCov(R_angle);

  timer = micros();
}

void loop(){
	while(micros() - timer < time_us);
	timer = micros();

  //measurement
	read_data();
	filter();

	KalmanX.Output(acc_angle_x, gyro_x);
  Serial.println(KalmanX.K[0]); 
  //KalmanY.Output(acc_angle_y, gyro_y);  
  //Serial.print(KalmanX.PredCov[0][0]);
  //Serial.print("    ");
  //Serial.print(KalmanX.PredCov[1][1]);
  //Serial.print("    "); 
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

void filter(){
  
  total_vector = sqrt(pow(abs(acc_x),2) + pow(abs(acc_y),2) + pow(abs(acc_z),2));
  if(abs(acc_y) < total_vector)acc_angle_x = asin((float)acc_y/total_vector)* 57.296;       
  if(abs(acc_x) < total_vector)acc_angle_y = asin((float)acc_x/total_vector)* -57.296;       
  if(set_angles==0){
    angle_x = acc_angle_x;
    angle_y = acc_angle_y;
    set_angles = 1;
  }
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
  acc_x -= 10;
  acc_y -= 70;
  
  /*Wire.beginTransmission(0x1E);
  Wire.write(0x03);
  Wire.endTransmission();
  Wire.requestFrom(0x1E,6);
  while(Wire.available()<6);
  comp_x = Wire.read()<<8|Wire.read();
  comp_z = Wire.read()<<8|Wire.read();
  comp_y = Wire.read()<<8|Wire.read();*/
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

void LPF(){
	
}
