#ifndef _Kalman_IMU_h
#define _Kalman_IMU_h

#include <math.h>

class Kalman_IMU{
	public:
		double pred_angle;                          // Predicted State
		double predCov;								// Predicted State Covariance
		double Q;								// Process Noise Covariance
		double angle_measure;						// Measurement z
		double angle_diff;
		double u;
		double R;								// Measurement Noise Covariance
		double S;								// Innovation Covariance
		double dt;									// Time
		double K;								// Kalman Gain K
		double angle;                               // Updated State
		double rate;
		double Cov;						    // Updated State Covariance
	
		void predict(){
			//Predict
			pred_angle = angle + dt*u;
			predCov = Cov + Q;
		}
	
		void update(){
			//Update
			angle_diff = angle_measure - pred_angle;

			S = predCov + R;
		
			K = predCov/S;
		
			angle = pred_angle + K*angle_diff;

			Cov = (1 - K)*predCov;
		}
	
		void init_main(int time_us, double init_angle){
			dt = time_us/1000000.0;
			angle = init_angle;
			Cov = 0.2;
		}
	
		void init_processCov(double pred_var_angle){
			//Process Noise Covariance
			Q = pred_var_angle;
		}

		void init_measureCov(double var_angle){
			//Measurement Noise Covariance
			R = var_angle;
		}
		
		
		void update_measureCov(double var_angle, double var_rate){
			//Measurement Noise Covariance
			R = var_angle + dt*var_rate;
		}
		
		void take_measurements(double acc_read){
			//Take measurements
			angle_measure = acc_read;
		}
		
		void control_input(double rate){
			u = rate;
		}

		void kalmanOutput(double acc_read, double gyro_read, double var_angle, double var_rate){
			control_input(gyro_read);
			predict();
			take_measurements(acc_read);
			update_measureCov(var_angle, var_rate);
			update();
		}
		
};

#endif
