#ifndef _Kalman_h
#define _Kalman_h

class Kalman{
	public:
		double last_X[2];							// Last State
		double pred_X[2];                           // Predicted State
		double predCov[2][2];						// Predicted State Covariance
		double u[2];                                // Control Input u
		double B[2][2];								// Control Input Model
		double Q[2][2];								// Process Noise Covariance
		double z[2];								// Measurement z
		double y[2];								// Innovation/Measurement Residual
		double R[2][2];								// Measurement Noise Covariance
		double S[2][2];								// Innovation Covariance
		double dt;									// Time
		double K[2][2];								// Kalman Gain K
		double new_X[2];                            // New State
		double newCov[2][2];						// New State Covariance
	
		void predict(){
			//Predict
			pred_X[0] = new_X[0] + dt*new_X[1] ;//+ B[0][0]*u[0] + B[0][1]*u[1];
			pred_X[1] = new_X[1]                ;//+ B[1][0]*u[0] + B[1][1]*u[1];

			predCov[0][0] = newCov[0][0] + dt*(dt*newCov[1][1] + newCov[0][1] + newCov[1][0]) + Q[0][0];
			predCov[0][1] = newCov[0][1] + dt*newCov[1][1]                                    + Q[0][1];
			predCov[1][0] = newCov[1][0] + dt*newCov[1][1]                                    + Q[1][0];
			predCov[1][1] = newCov[1][1]                                                      + Q[1][1];
		}
	
		void update(){
			//Update
			y[0] = z[0] - pred_X[0];
			y[1] = z[1] - pred_X[1];

			S[0][0] = predCov[0][0] + R[0][0];
			S[0][1] = predCov[0][1] + R[0][1];
			S[1][0] = predCov[1][0] + R[1][0];
			S[1][1] = predCov[1][1] + R[1][1];

			double deno = S[0][0]*S[1][1] - S[0][1]*S[1][0];
		
			K[0][0] = (predCov[0][0]*S[1][1] - predCov[0][1]*S[1][0])/deno;
			K[0][1] = 0;//(predCov[0][1]*S[0][0] - predCov[0][0]*S[0][1])/deno;
			K[1][0] = 0;//(predCov[1][0]*S[1][1] - predCov[1][1]*S[1][0])/deno;
			K[1][1] = (predCov[1][1]*S[0][0] - predCov[1][0]*S[0][1])/deno;
		
			new_X[0] = pred_X[0] + K[0][0]*y[0] + K[0][1]*y[1];
			new_X[1] = pred_X[1] + K[1][0]*y[0] + K[1][1]*y[1];

			newCov[0][0] = (1 - K[0][0])*predCov[0][0] + K[0][1]*predCov[1][0];
			newCov[0][1] = (1 - K[0][0])*predCov[0][1] + K[0][1]*predCov[1][1];
			newCov[1][0] = K[1][0]*predCov[0][0] + (1 - K[1][1])*predCov[1][0];
			newCov[1][1] = K[1][0]*predCov[0][1] + (1 - K[1][1])*predCov[1][1];
		}
	
		void init_main(int time_us, double init_X_0, double init_X_1){
			dt = time_us/1000000.0;
			new_X[0] = init_X_0;
			new_X[1] = init_X_1;
			newCov[0][0] = 0.2;
			newCov[0][1] = 0;
			newCov[1][0] = 0;
			newCov[1][1] = 0.002;
		}
	
		void init_processCov(double sigma_X_0, double sigma_X_1){
			//Process Noise Covariance
			Q[0][0] = sigma_X_0;
			Q[0][1] = 0;
			Q[1][0] = 0;
			Q[1][1] = sigma_X_1;
		}

		void init_measureCov(double sigma_Z_0, double sigma_Z_1){
			//Measurement Noise Covariance
			R[0][0] = sigma_Z_0;
			R[0][1] = 0;
			R[1][0] = 0;
			R[1][1] = sigma_Z_1;
		}
	
		void kalmanOutput(double z_0, double z_1){
			predict();
			//Take measurements
			z[0] = z_0;
			z[1] = z_1;

			for(int i=0; i<2; i++){
				
				last_X[i] = new_X[i];
			}
			update();
		}
};
#endif
