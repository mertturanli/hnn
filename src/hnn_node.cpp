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

#include "ros/ros.h"
#include "hnn/AgentPosition.h"
#include "hnn/EstimationVector.h"

#include "hnn.h"

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>

#include <string>
#include <cstdlib>
#include <cstring>

#include <unistd.h>

#include <boost/thread/mutex.hpp>
#include <boost/timer/timer.hpp>

#include <time.h> 


#include <stdlib.h>


boost::mutex mainMutex;


double pose[3] = {0};
double velocity[3] = {0};
double xt[2] = {0};

int poseReceived = 0;
int xtReceived = 0;


using namespace boost::timer;
using namespace std;


void xtCallback(const hnn::AgentPosition& msg) {
	
	ROS_INFO("xt data received\n");

	mainMutex.lock();

	xt[0] = msg.x;
	xt[1] = msg.y;

	xtReceived = 1;

	mainMutex.unlock();

	return;
}

void groundTruthCallback(const nav_msgs::Odometry& msg) {
	//ROS_INFO("position data received\n");
	
	mainMutex.lock();

	tf::Quaternion q( 	msg.pose.pose.orientation.x,
						msg.pose.pose.orientation.y,
						msg.pose.pose.orientation.z,
						msg.pose.pose.orientation.w );

	double yaw, pitch, roll;
	tf::Matrix3x3 mat(q);
	mat.getEulerYPR(yaw, pitch, roll);

	//

	pose[0] = msg.pose.pose.position.x;
	pose[1] = msg.pose.pose.position.y;
	pose[2] = yaw;

	velocity[0] = msg.twist.twist.linear.x;
	velocity[1] = msg.twist.twist.linear.y;
	velocity[2] = msg.twist.twist.angular.z;

	ROS_INFO(" ground truth callback");
	ROS_INFO("  x = %lf\n", pose[0]);
	ROS_INFO("  y = %lf\n", pose[1]);
	ROS_INFO("  theta = %lf\n", pose[2]);

	poseReceived = 1;
	
	mainMutex.unlock();

	return;
}


int main(int argc, char* argv[]) {

	ros::init(argc, argv, "hnn");
	ros::NodeHandle n("~");

	int agentId = atoi(argv[1]);
	double hfAlpha = 0, hfBeta = 0;
	
	n.getParam("hfAlpha", hfAlpha);
	n.getParam("hfBeta", hfBeta);
	
	HNN* hnn = new HNN(hfAlpha, hfBeta);

	std::cout << "Agent id: " << agentId << std::endl;
	std::cout << "hfAlpha: " << hfAlpha << std::endl;
	std::cout << "hfBeta: " << hfBeta << std::endl;
	
	vector<double> v(2);
	n.getParam("realVector", v);
	double vector[2] = {v[0], v[1]};
	hnn->setRealVector(vector);

	
	

	std::ostringstream conv;
	string strRobot("/robot");
	string str1;

	// ground truth
	conv.clear();
	conv.str("");
	conv << agentId + 1;
	str1 = strRobot + conv.str() + "/pose";
	ros::Subscriber subscriberGroundTruth = n.subscribe(str1, 1000, groundTruthCallback);

	// xt
	conv.clear();
	conv.str("");
	conv << agentId + 1;
	str1 = strRobot + conv.str() + "/xt";
	ros::Subscriber subscriberTrajectory = n.subscribe(str1, 1000, xtCallback);

	// est. err.
	str1 = strRobot + conv.str() + "/estErr";
	ros::Publisher estErrPub = n.advertise<std_msgs::Float64>(str1, 10);

	// est. vec.
	str1 = strRobot + conv.str() + "/estVector";
	ros::Publisher estVecPub = n.advertise<hnn::EstimationVector>(str1, 10);


	//

	double poseBuffer[3];
	double velocityBuffer[3];
	double xtBuffer[2];

	ros::Time currentTime;
	ros::Time lastTime = ros::Time::now();
	double dt = 0;

	// time
	ros::Rate r(100.0);


	// loop
	while (ros::ok()) {

		// get dt
		currentTime = ros::Time::now();
		dt = (currentTime - lastTime).toSec();
		

		mainMutex.lock();

		if (poseReceived == 1 && xtReceived == 1) {
			// ground truth
			poseBuffer[0] = pose[0];
			poseBuffer[1] = pose[1];
			poseBuffer[2] = pose[2];

			velocityBuffer[0] = velocity[0];
			velocityBuffer[1] = velocity[1];
			velocityBuffer[2] = velocity[2];	

			xtBuffer[0] = xt[0];
			xtBuffer[1] = xt[1];
		}


		if (dt >= 0.1) {

			cout << "dt = " << dt << endl;
	
			
			// main algorithm
			if (poseReceived == 1 && xtReceived == 1) {
				
				ROS_INFO("Position (%d) = (%f, %f, %f)\n", agentId, poseBuffer[0], poseBuffer[1], poseBuffer[2]);
				ROS_INFO("Velocity (%d) = (%f, %f, %f)\n", agentId, velocityBuffer[0], velocityBuffer[1], velocityBuffer[2]);


				hnn->set(poseBuffer, velocityBuffer, xtBuffer);
				
				hnn->estimate();

				// publish est. err.
				std_msgs::Float64 estErr;
				estErr.data = hnn->getEstimationError();

				estErrPub.publish(estErr);

				// publish est. vector
				hnn::EstimationVector estVec;
				double Ki_est[2];

				hnn->getEstimationVector(Ki_est);

				estVec.data[0] = Ki_est[0];
				estVec.data[1] = Ki_est[1];

				estVecPub.publish(estVec);

			}

			
			
			//
			lastTime = currentTime;

		}

	

		mainMutex.unlock();

		// spin
		ros::spinOnce();

		// sleep
		r.sleep();
	}
	
	delete hnn;
	
	return 0;
}

