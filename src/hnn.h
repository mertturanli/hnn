/*
The MIT License

Copyright (c) 2019 Mert Turanli

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#ifndef HNN_H_
#define HNN_H_

#include <string>
#include <vector>

#include <Eigen/Dense>

using Eigen::Matrix2d;
using Eigen::Array22d;
using Eigen::Matrix;
using Eigen::ArrayXXd;


class HNN {
protected:
	
	static double dt; // time step [s] (0.1)
	static double l; // distance to P (point-offset)

	double pose[3];
	double pose_1[3];
	double velocity[3];
	double velocity_1[3];
	double xt[2];
	double xt_1[2];
	
	//
	double Ki[2];
	double Ki_est[2];

	//
	Matrix<double, 2, 2> AA;
	double x1;
	double x2;
	Matrix<double, 2, 1> yy;
	
	Matrix<double, 2, 2> WW;
	Matrix<double, 2, 1> bb;
	
	Matrix<double, 2, 1> dpitdt;
	Matrix<double, 2, 1> dpitdt_1;
	Matrix<double, 2, 1> pit;
	Matrix<double, 2, 1> pit_1;
	
	Matrix<double, 2, 1> si;
	Matrix<double, 2, 1> theta_est;
	
	Matrix<double, 2, 1> M_dx;

	
	double estErr;
	
	//
	double hf_alpha;
	double hf_beta;

	// 
	bool firstLoop; 

public:
	HNN(double hf_alpha, double hf_beta);
	~HNN();

	void set(const double* pose, const double* velocity, const double* xt);
	void setRealVector(const double* Ki);


	void estimate();

	void getEstimationVector(double* Ki);

	double getEstimationError();
};

#endif /* HNN_H_ */
