//Kalman Filter
//TOPTHL

#ifndef _KalmanFilterTOPTHL_h
#define _KalmanFilterTOPTHL_h

class KalmanFilter
{
  public:
    //LAST STATE 
    double last_X[2];
    double lastCov[2][2];
    //PREDICTED STATE (PRIORI)
    double pred_X[2];
    double predCov[2][2];
    //NEW(UPDATED) STATE (POSTERIORI)
    double new_X[2];
    double newCov[2][2];
    //Process Noise Covariance
    double Q[2][2];
    //Measurement z
    double z[2];
    //Innovation/Measurement residual
    double y;
    //Measurement Noise Covariance
    double R;
    //Kalman Gain K
    double K[2];
    //time interval
    double dt;
    void debug(double val);
    
    void init_main(int time_us, float init_X_0);
    
    void init_processCov(float sigma_X_0, float sigma_X_1);
    
    void init_measureCov(float R_00);

    void predict(double gyro);
    
    void update(double acc);
        
    void Output(double z_0, double z_1);
 
};
#endif
