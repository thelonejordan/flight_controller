#include "KalmanFilterTOPTHL.h"

void KalmanFilter::debug(double val){
  Serial.print("last X1  :  ");
  Serial.println(last_X[0]);
  Serial.print("last X2  :  ");
  Serial.println(last_X[1]);
  
  Serial.print("last Cov00  :  ");
  Serial.println(lastCov[0][0]);
  Serial.print("last Cov01  :  ");
  Serial.println(lastCov[0][1]);
  Serial.print("last Cov10  :  ");
  Serial.println(lastCov[1][0]);
  Serial.print("last Cov11  :  ");
  Serial.println(lastCov[1][1]);

  Serial.print("pred X1  :  ");
  Serial.println(pred_X[0]);
  Serial.print("pred X2  :  ");
  Serial.println(pred_X[1]);
  
  Serial.print("pred Cov00  :  ");
  Serial.println(predCov[0][0]);
  Serial.print("pred Cov01  :  ");
  Serial.println(predCov[0][1]);
  Serial.print("pred Cov10  :  ");
  Serial.println(predCov[1][0]);
  Serial.print("pred Cov11  :  ");
  Serial.println(predCov[1][1]);

  while(1);
}

void KalmanFilter::init_main(int time_us, float init_X_0){
      dt = time_us/1000000.0;
      last_X[0] = init_X_0;
      last_X[1] = 0;
      for(int i=0; i<2; i++)
      {
        for(int j=0; j<2; j++)
        {
          lastCov[i][j] = 0;
        }
      }
    }

void KalmanFilter::init_processCov(float sigma_X_0, float sigma_X_1)
    {
      //Process Noise Covariance
      Q[0][0] = sigma_X_0;
      Q[0][1] = 0;
      Q[1][0] = 0;
      Q[1][1] = sigma_X_1;
      lastCov[0][0] = dt*dt*sigma_X_0;
    }

void KalmanFilter::init_measureCov(float R_00)
    {
      //Measurement Noise Covariance
      R = R_00;
    }

void KalmanFilter::predict(double gyro)
    {
      //Predict
      //predX = F*last_X + B*u
      pred_X[0] = last_X[0] - dt*last_X[1] + dt*gyro;
      pred_X[1] = last_X[1];

      //predCov = F*lastCov*F' + Q
      predCov[0][0] = lastCov[0][0] - dt*(lastCov[0][1] + lastCov[1][0] - dt*lastCov[1][1]) + Q[0][0];
      predCov[0][1] = lastCov[0][1] - dt*lastCov[1][1]                                      + Q[0][1];
      predCov[1][0] = lastCov[1][0] - dt*lastCov[1][1]                                      + Q[1][0];
      predCov[1][1] = lastCov[1][1]                                                         + Q[1][1];
    }

void KalmanFilter::update(double acc)
    {
      //Update
      //y = z - H*pred_X
      y = acc - pred_X[0];

      //K = predCov*H'/S
      K[0] = predCov[0][0]/(predCov[0][0] + R);
      K[1] = predCov[1][1]/(predCov[0][0] + R);
  
      //new_X = pred_X + K*y
      new_X[0] = pred_X[0] + K[0]*y;
      new_X[1] = pred_X[1] + K[1]*y;

      //newCov = (I - K*H)predCov
      newCov[0][0] = (1 - K[0])*predCov[0][0];
      newCov[0][1] = (1 - K[0])*predCov[0][1];
      newCov[1][0] = predCov[1][0] - K[1]*predCov[0][0];
      newCov[1][1] = predCov[1][1] - K[1]*predCov[0][1];
    }

void KalmanFilter::Output(double z_0, double z_1){
      predict(z_1);
      update(z_0);

      for(int i=0; i<2; i++)
      {
        last_X[i] = new_X[i];
        for(int j=0; j<2; j++)
        {
          lastCov[i][j] = newCov[i][j];
        }
      }
    }  
