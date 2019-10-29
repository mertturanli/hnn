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

#include <string>
#include <cstring>
#include <iostream>
#include <cmath>

#include <cassert>

#include "hnn.h"

double HNN::dt = 0.1;
double HNN::l = 0.1;


#include <Eigen/Dense>

using Eigen::Matrix2d;
using Eigen::Array22d;
using Eigen::Matrix;
using Eigen::ArrayXXd;
using Eigen::MatrixXd;

using namespace std;

HNN::HNN(double hf_alpha, double hf_beta) {

	pose[0] = 0;
	pose[1] = 0;
	pose[2] = 0;
	pose_1[0] = 0;
	pose_1[1] = 0;
	pose_1[2] = 0;

	velocity[0] = 0;
	velocity[1] = 0;
	velocity[2] = 0;
	velocity_1[0] = 0;
	velocity_1[1] = 0;
	velocity_1[2] = 0;

	xt[0] = 0;
	xt[1] = 0;
	xt_1[0] = 0;
	xt_1[1] = 0;

	// estimation vector
	Ki[0] = 0;
	Ki[1] = 0;
	Ki_est[0] = 2.0;
	Ki_est[1] = 2.0;

	//
	this->AA = MatrixXd::Zero(2, 2);
	this->x1 = 0;
	this->x2 = 0;
	this->yy = MatrixXd::Zero(2, 1);
	
	this->WW = MatrixXd::Zero(2, 2);
	this->bb = MatrixXd::Zero(2, 1);
	
	this->dpitdt = MatrixXd::Zero(2, 1);
	this->dpitdt_1 = MatrixXd::Zero(2, 1);
	this->pit = MatrixXd::Zero(2, 1);
	this->pit_1 = MatrixXd::Zero(2, 1);
	this->pit_1(0, 0) = 1e-4; // hf_beta * atanh(1.0 / hf_alpha / Ki_est[0]); // 1e-2
	this->pit_1(1, 0) = 1e-4; // hf_beta * atanh(1.0 / hf_alpha / Ki_est[1]);
	
	this->si = MatrixXd::Zero(2, 1);
	this->theta_est = MatrixXd::Zero(2, 1);

	//
	this->estErr = 0;

	//

	this->hf_alpha = hf_alpha;
	this->hf_beta = hf_beta;

	//
	firstLoop = true;

}

HNN::~HNN() { }



void HNN::set(const double* pose, const double* velocity, const double* xt) {
	this->pose[0] = pose[0];
	this->pose[1] = pose[1];
	this->pose[2] = pose[2];

	this->velocity[0] = velocity[0];
	this->velocity[1] = velocity[1];
	this->velocity[2] = velocity[2];

	this->xt[0] = xt[0];
	this->xt[1] = xt[1];

	if (firstLoop == true) {
		this->pose_1[0] = pose[0];
		this->pose_1[1] = pose[1];
		this->pose_1[2] = pose[2];
		this->pose_2[0] = pose[0];
		this->pose_2[1] = pose[1];
		this->pose_2[2] = pose[2];

		this->velocity_1[0] = velocity[0];
		this->velocity_1[1] = velocity[1];
		this->velocity_1[2] = velocity[2];

		this->xt_1[0] = xt[0];
		this->xt_1[1] = xt[1];
		this->xt_2[0] = xt[0];
		this->xt_2[1] = xt[1];
	}
}

void HNN::setRealVector(const double* Ki) {
	this->Ki[0] = Ki[0];
	this->Ki[1] = Ki[1];
}


void HNN::estimate() {

	// derivatives
	double dx[3] = {0};

	if (firstLoop == false) {
		dx[0] = velocity[0];
		dx[1] = velocity[1];
		dx[2] = velocity[2];
	
		// find linear and angular velocities

		double theta = pose_1[2]; //

		Matrix<double, 2, 2> A = MatrixXd::Zero(2, 2);
		A(0, 0) = cos(theta);
		A(0, 1) = -sin(theta);
		A(1, 0) = sin(theta);
		A(1, 1) = cos(theta);
	
		double angle = atan2(dx[1], dx[0]) - theta;
		angle = fmod(angle, 2.0*M_PI);
		if (angle < 0) {
			angle += 2.0*M_PI;
		}

		if (angle > M_PI) {
			angle -= 2.0*M_PI;
		}
		else if (angle < -M_PI) {
			angle += 2.0*M_PI;
		}
		double angleDegrees = fabs(angle / M_PI * 180.0);
		double direction = 0;
		if (angleDegrees > 90.0) {
			direction = -1;
		}
		else {
			direction = 1;
		}
	
		Matrix<double, 2, 1> M_velocities = MatrixXd::Zero(2, 1);
		M_velocities(0, 0) = direction * sqrt(dx[0]*dx[0] + dx[1]*dx[1]);
		M_velocities(1, 0) = dx[2] * l;

		M_dx = A * M_velocities;

		std::cout << "velocity = (" << velocity[0] << ", " << velocity[1] << ", " << velocity[2] << ")" << std::endl;
		std::cout << "(v_calc, w_calc) = (" << direction * sqrt(dx[0]*dx[0] + dx[1]*dx[1]) << ", " << dx[2] << ")" << std::endl;

	}
	else {
		M_dx(0, 0) = 0;
		M_dx(1, 0) = 0;

	}

	/* AA = [  -dx(1), 0; 
	            0, -dx(2) ];    
	    x1 = Probot_1(1, i);
	    x2 = Probot_1(2, i);
	    yy = [  x1 - xt_1(1, i); x2 - xt_1(2, i)  ];*/

	AA(0, 0) = -M_dx(0, 0);
	AA(1, 1) = -M_dx(1, 0);

	x1 = pose_1[0];
	x2 = pose_1[1];

	yy(0, 0) = x1 - xt_1[0];
	yy(1, 0) = x2 - xt_1[1];

	WW = AA.transpose() * AA;
	bb = -AA.transpose() * yy;

	/* dpitdt(:, i) = -(WW * si(:, i) + bb);
	    pit(:, i) = pit_1(:, i) + (dpitdt(:, i) + dpitdt_1(:, i)) / 2.0 * dt;
	    */
	dpitdt = -(WW * si + bb);
	// integrate
	pit(0, 0) = pit_1(0, 0) + (dpitdt(0, 0) + dpitdt_1(0, 0)) / 2.0 * dt;
	pit(1, 0) = pit_1(1, 0) + (dpitdt(1, 0) + dpitdt_1(1, 0)) / 2.0 * dt;

	// si(:, i) = hf_alpha * tanh(pit(:, i) / hf_beta);
	si(0, 0) = hf_alpha * tanh(pit(0, 0) / hf_beta);
	si(1, 0) = hf_alpha * tanh(pit(1, 0) / hf_beta);

	theta_est = si;

	// assign Ki_est
	Ki_est[0] = 1.0 / theta_est(0, 0);
	Ki_est[1] = 1.0 / theta_est(1, 0);

	// output
	double estErr = sqrt((Ki_est[0] - Ki[0]) * (Ki_est[0] - Ki[0]) +
			     (Ki_est[1] - Ki[1]) * (Ki_est[1] - Ki[1]));

	this->estErr = estErr;


	cout << "Ki_est = [" << Ki_est[0] << ", " << Ki_est[1] << "]" << endl;
	
	pit_1 = pit;
	dpitdt_1 = dpitdt;

	//
    	pose_1[0] = pose[0];
	pose_1[1] = pose[1];
	pose_1[2] = pose[2];

	xt_1[0] = xt[0];
	xt_1[1] = xt[1];
	
	velocity_1[0] = velocity[0];
	velocity_1[1] = velocity[1];
	velocity_1[2] = velocity[2];

	//	
	firstLoop = false;

	return;
}


void HNN::getEstimationVector(double* Ki) {
	Ki[0] = this->Ki_est[0];
	Ki[1] = this->Ki_est[1];

	return;
}

double HNN::getEstimationError() {
	return this->estErr;
}



