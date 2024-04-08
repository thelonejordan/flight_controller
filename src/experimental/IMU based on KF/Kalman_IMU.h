#ifndef _Kalman_IMU_h
#define _Kalman_IMU_h

#include <math.h>

class Kalman_IMU{
	public:
		double pred_angle;                          // Predicted State
		double pred_rate;
		double predCov[2][2];						// Predicted State Covariance
		double u[2];                                // Control Input u
		double B[2][2];								// Control Input Model
		double Q[2][2];								// Process Noise Covariance
		double angle_measure;						// Measurement z
		double rate_measure;
		double angle_diff;							// Innovation/Measurement Residual
		double rate_diff;
		double R[2][2];								// Measurement Noise Covariance
		double S[2][2];								// Innovation Covariance
		double dt;									// Time
		double K[2][2];								// Kalman Gain K
		double angle;                               // Updated State
		double rate;
		double Cov[2][2];						    // Updated State Covariance
	
		void predict(){
			//Predict
			pred_angle = angle + dt*rate ;//+ B[0][0]*u[0] + B[0][1]*u[1];
			pred_rate = rate                ;//+ B[1][0]*u[0] + B[1][1]*u[1];

			predCov[0][0] = Cov[0][0] + dt*(dt*Cov[1][1] + Cov[0][1] + Cov[1][0]) + Q[0][0];
			predCov[0][1] = Cov[0][1] + dt*Cov[1][1]                              + Q[0][1];
			predCov[1][0] = Cov[1][0] + dt*Cov[1][1]                              + Q[1][0];
			predCov[1][1] = Cov[1][1]                                             + Q[1][1];
		}
	
		void update(){
			//Update
			angle_diff = angle_measure - pred_angle;
			rate_diff = rate_measure - pred_rate;

			S[0][0] = predCov[0][0] + R[0][0];
			S[0][1] = predCov[0][1] + R[0][1];
			S[1][0] = predCov[1][0] + R[1][0];
			S[1][1] = predCov[1][1] + R[1][1];

			double divisor = S[0][0]*S[1][1] - S[0][1]*S[1][0];
		
			K[0][0] = (predCov[0][0]*S[1][1] - predCov[0][1]*S[1][0])/divisor;
			K[0][1] = (predCov[0][1]*S[0][0] - predCov[0][0]*S[0][1])/divisor;
			K[1][0] = (predCov[1][0]*S[1][1] - predCov[1][1]*S[1][0])/divisor;
			K[1][1] = (predCov[1][1]*S[0][0] - predCov[1][0]*S[0][1])/divisor;
		
			angle = pred_angle + K[0][0]*angle_diff + K[0][1]*rate_diff;
			rate = pred_rate + K[1][0]*angle_diff + K[1][1]*rate_diff;

			Cov[0][0] = (1 - K[0][0])*predCov[0][0] + K[0][1]*predCov[1][0];
			Cov[0][1] = (1 - K[0][0])*predCov[0][1] + K[0][1]*predCov[1][1];
			Cov[1][0] = K[1][0]*predCov[0][0] + (1 - K[1][1])*predCov[1][0];
			Cov[1][1] = K[1][0]*predCov[0][1] + (1 - K[1][1])*predCov[1][1];
		}
	
		void init_main(int time_us, double init_angle, double init_rate){
			dt = time_us/1000000.0;
			angle = init_angle;
			rate = init_rate;
			Cov[0][0] = 0.2;
			Cov[0][1] = 0;
			Cov[1][0] = 0;
			Cov[1][1] = 0.002;
		}
	
		void init_processCov(double pred_var_angle, double pred_var_rate){
			//Process Noise Covariance
			Q[0][0] = pred_var_angle;
			Q[0][1] = sqrt(abs(pred_var_angle*pred_var_rate));
			Q[1][0] = Q[0][1];
			Q[1][1] = pred_var_rate;
		}

		void init_measureCov(double var_angle, double var_rate){
			//Measurement Noise Covariance
			R[0][0] = var_angle;
			R[0][1] = 0;
			R[1][0] = 0;
			R[1][1] = var_rate;
		}
		
		
		void update_measureCov(double var_angle, double var_rate){
			//Measurement Noise Covariance
			R[0][0] = var_angle;
			R[0][1] = sqrt(abs(var_angle*var_rate));
			R[1][0] = R[0][1];
			R[1][1] = var_rate;
		}
		
		void take_measurements(double acc_read, double gyro_read){
			//Take measurements
			angle_measure = acc_read;
			rate_measure = gyro_read;
		}

		void kalmanOutput(double acc_read, double gyro_read, double var_angle, double var_rate){
			predict();
			take_measurements(acc_read, gyro_read);
			update_measureCov(var_angle, var_rate);
			update();
		}
		
};

#endif
