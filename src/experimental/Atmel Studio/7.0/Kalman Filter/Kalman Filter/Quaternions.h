#ifndef _Quaternions_h
#define _Quaternions_h
#include <math.h>

class Quaternion;
Quaternion q_add(Quaternion Q1, Quaternion Q2);
Quaternion q_multiply(Quaternion Q1, Quaternion Q2);
Quaternion Vector(double x, double y, double z);
Quaternion q_rotate(Quaternion axis, double angle);

class Quaternion{
	
	public:
		double q1 = 0;
		double q2 = 0;	
		double q3 = 0;
		double q4 = 0;
		Quaternion(double a, double b, double c, double d){
			q1 = a;
			q2 = b;
			q3 = c;
			q4 = d;
		}
		
		Quaternion(){
			q1 = 0;
			q2 = 0;
			q3 = 0;
			q4 = 0;
		}
		
		Quaternion(const Quaternion &Q){
			q1 = Q.q1;
			q2 = Q.q2;
			q3 = Q.q3;
			q4 = Q.q4;
		}
		
		Quaternion conjugate(void){
			return Quaternion(q1, -q2, -q3, -q4);
		}
		
		double modulus(void){
			double mod = sqrt(q1*q1 + q2*q2 + q3*q3 + q4*q4);
			return mod;
		}
		
		void normalize(void){
			double mod = this->modulus();
			q1 /= mod;
			q2 /= mod;
			q3 /= mod;
			q4 /= mod;
		}
		
		void rotate(Quaternion axis, double angle){
			Quaternion q = q_rotate(axis, angle);
			Quaternion Q = q_multiply(q, *this);
			Q = q_multiply(Q, q.conjugate());
			*this = Quaternion(Q);
		}
};

Quaternion q_add(Quaternion Q1, Quaternion Q2){
	Quaternion Q3;
	Q3.q1 = Q1.q1 + Q2.q1;
	Q3.q2 = Q1.q2 + Q2.q2;
	Q3.q3 = Q1.q3 + Q2.q3;
	Q3.q4 = Q1.q4 + Q2.q4;
	return Q3;
}

Quaternion q_multiply(Quaternion Q1, Quaternion Q2){
	Quaternion Q3;
	Q3.q1 = Q1.q1*Q2.q1 - Q1.q2*Q2.q2 - Q1.q3*Q2.q3 - Q1.q4*Q2.q4;       // ae - bf - cg - dh
	Q3.q2 = Q1.q2*Q2.q1 + Q1.q1*Q2.q2 - Q1.q4*Q2.q3 + Q1.q3*Q2.q4;       // be + af - dg + ch
	Q3.q3 = Q1.q3*Q2.q1 + Q1.q4*Q2.q2 + Q1.q1*Q2.q3 - Q1.q2*Q2.q4;       // ce + df + ag - bh
	Q3.q4 = Q1.q4*Q2.q1 - Q1.q3*Q2.q2 + Q1.q2*Q2.q3 + Q1.q1*Q2.q4;       // de - cf + bg + ah
	return Q3;
}

Quaternion Vector(double x, double y, double z){
	return Quaternion(0, x, y, z);
}

Quaternion q_rotate(Quaternion axis, double angle){
	axis.normalize();
	angle/= 114.591559026;
	return Quaternion(cos(angle), sin(angle)*axis.q2, sin(angle)*axis.q3, sin(angle)*axis.q4);
}
#endif