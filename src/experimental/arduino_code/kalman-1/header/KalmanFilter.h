#ifndef _KalmanFilter_h
#define _KalmanFilter_h

class Kalman
{
  private:
    //LAST STATE
    double last_X[2];
    double lastCov[2][2];
    //PREDICTED STATE
    double pred_X[2];
    double predCov[2][2];
    //Control Input u
    double u[2]; 
    //Control Input Model
    double B[2][2];
    //Process Noise Covariance
    double Q[2][2];
    //Measurement z
    double z[2];
    //Innovation/Measurement residual
    double y[2];
    //Measurement Noise Covariance
    double R[2][2];
    //Innovation Covariance
    double S[2][2];
    //Kalman Gain K
    double K[2][2];
    //time
    double dt;
    void kalmanFilter_predict()
    {
      //Predict
      pred_X[0] = last_X[0] + dt*last_X[1] + B[0][0]*u[0] + B[0][1]*u[1];
      pred_X[1] = last_X[1]                + B[1][0]*u[0] + B[1][1]*u[1];

      predCov[0][0] = lastCov[0][0] + dt*(dt*lastCov[1][1] + lastCov[0][1] + lastCov[1][0]) + Q[0][0];
      predCov[0][1] = lastCov[0][1] + dt*lastCov[1][1]                                      + Q[0][1];
      predCov[1][0] = lastCov[1][0] + dt*lastCov[1][1]                                      + Q[1][0];
      predCov[1][1] = lastCov[1][1]                                                         + Q[1][1];
    }
    
    void kalmanFilter_update()
    {
      //Update
      y[0] = z[0] - pred_X[0];
      y[1] = z[1] - pred_X[1];

      S[0][0] = predCov[0][0] + R[0][0];
      S[0][1] = predCov[0][1] + R[0][1];
      S[1][0] = predCov[1][0] + R[1][0];
      S[1][1] = predCov[1][1] + R[1][1];

      K[0][0] = predCov[0][0]*S[1][1] - predCov[0][1]*S[1][0];
      K[0][1] = predCov[0][1]*S[0][0] - predCov[0][0]*S[0][1];
      K[1][0] = predCov[1][0]*S[1][1] - predCov[1][1]*S[1][0];
      K[1][1] = predCov[1][1]*S[0][0] - predCov[1][0]*S[0][1];
  
      new_X[0] = pred_X[0] + K[0][0]*y[0] + K[0][1]*y[1];
      new_X[1] = pred_X[1] + K[1][0]*y[0] + K[1][1]*y[1];

      newCov[0][0] = (1 - K[0][0])*predCov[0][0] + K[0][1]*predCov[1][0];
      newCov[0][1] = (1 - K[0][0])*predCov[0][1] + K[0][1]*predCov[1][1];
      newCov[1][0] = K[1][0]*predCov[0][0] + (1 - K[1][1])*predCov[1][0];
      newCov[1][1] = K[1][0]*predCov[0][1] + (1 - K[1][1])*predCov[1][1];
    }
    
  public :
    //NEW(UPDATED) STATE
    double new_X[2];
    double newCov[2][2];

    void init_main(int time_us, float init_X_0, float init_X_1)
    {
      dt = time_us/1000000.0;
      last_X[0] = init_X_0;
      last_X[1] = init_X_1;
      for(int i=0; i<2; i++)
      {
        for(int j=0; j<2; j++)
        {
          lastCov[i][j] = 0;
        }
      }
    }
    
    void init_processCov(float sigma_X_0, float sigma_X_1)
    {
      //Process Noise Covariance
      Q[0][0] = sigma_X_0*sigma_X_0;
      Q[0][1] = sigma_X_0*sigma_X_1;
      Q[1][0] = Q[0][1];
      Q[1][1] = sigma_X_1*sigma_X_1;
    }

    void init_measureCov(float sigma_Z_0, float sigma_Z_1)
    {
      //Measurement Noise Covariance
      R[0][0] = sigma_Z_0*sigma_Z_0;
      R[0][1] = sigma_Z_0*sigma_Z_1;
      R[1][0] = R[0][1];
      R[1][1] = sigma_Z_1*sigma_Z_1;
    }
    
    void kalmanOutput(double z_0, double z_1){
      kalmanFilter_predict();

      //Take measurements
      z[0] = z_0;
      z[1] = z_1;
        
      kalmanFilter_update();

      for(int i=0; i<2; i++)
      {
        last_X[i] = new_X[i];
        for(int j=0; j<2; j++)
        {
          lastCov[i][j] = newCov[i][j];
        }
      }
    }  
};
#endif
