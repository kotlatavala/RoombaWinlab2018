#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <aruco_mapping/ArucoMarker.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <tf/transform_datatypes.h>
#include <math.h>
#include <cmath>

//------------------------------------------
// Global variable declaration 
//------------------------------------------

std::string filepath = "/home/ubuntu/catkin_ws/src/roomba_move/data/ctrl_room_map.txt";
const int numTags = 12;
float tagLocs[numTags][3];

int arrIndex = 0;
const int avgSize = 3;
float x_arr[avgSize];
float y_arr[avgSize];
float yaw_arr[avgSize];

float xAvg;
float yAvg;
float yawAvg;

/*
@param: path to map file (string)

Parse map txt file, extract tag numbers and corresponding IRL coords
Store all values in tagLocs array
*/
void fillTagLocs() {
	std::ifstream inFile (filepath.c_str(), std::ios_base::in);

	float tag_num;
	float x_tag;
	float y_tag;
	
	int i = 0;
	if(inFile.is_open()) {
		while(inFile >> tag_num) {
			tagLocs[i][0] = tag_num; 
					
			inFile >> x_tag >> y_tag;
			tagLocs[i][1] = x_tag;
			tagLocs[i][2] = y_tag;
			i++;
		}
		inFile.close();
	}
	else {
		ROS_INFO("Cannot open file");
	}
}

/*
@param: ID number (int)

Search tagLocs for matching tagID
Get IRL x and y coords of tag

@return: float array [x,y]
*/
float * getTagLoc(int id_int) {
	float id = (float) id_int;
	
	int i = 0;
	while (tagLocs[i][0] != id) {
		i++;
	}
	
	float x = tagLocs[i][1];
	float y = tagLocs[i][2];

	static float coords[2] = {x,y};
	return coords;
}

/*
Calcs average of values in pos arrays
*/
void calcAvgs() {
	int i = 0;
	xAvg = 0;
	yAvg = 0;
	yawAvg = 0;

	while (i < avgSize) {
		xAvg += x_arr[i];
		yAvg += y_arr[i];
		yawAvg += yaw_arr[i];
		i++;
	}
	
	xAvg = xAvg/avgSize;
	yAvg = yAvg/avgSize;
	yawAvg = yawAvg/avgSize;

	xAvg = floorf(xAvg*100)/100;
	yAvg = floorf(yAvg*100)/100;
	yawAvg = floorf(yawAvg*100)/100;
}

/*
@param: ArucoMarker msg published to aruco_poses

Callback function for aruco_poses subscriber
Extracts camera pose 
*/
void updatePos(aruco_mapping::ArucoMarker msg) {
	if (msg.marker_visibile) {
		//------------------------------------------
		// Position Extraction
		//------------------------------------------
		geometry_msgs::Point pose_msg = msg.global_camera_pose.position;
		float x_cam = pose_msg.x;
		float y_cam = pose_msg.y;

		int tagID = msg.marker_ids[0];	// Origin marker stored in index 0
		float *origin_coords = getTagLoc(tagID);
		
		float x_real = x_cam + *origin_coords;	// Real pos is cam pos + origin offset
		float y_real = y_cam + *(origin_coords+1);
		
		//------------------------------------------
		// Orienation Extract/Calc
		//------------------------------------------
		geometry_msgs::Quaternion quat_msg = msg.global_camera_pose.orientation;
		tf::Quaternion quat;
		tf::quaternionMsgToTF(quat_msg, quat);
		quat.normalize();	
		
		double yaw_double = tf::getYaw(quat);
		float yaw = (float) yaw_double;
		
		//------------------------------------------
		// Average and Round 
		//------------------------------------------
		x_arr[arrIndex] = x_real;
		y_arr[arrIndex] = y_real;		
		yaw_arr[arrIndex] = yaw;
		
		if (arrIndex == (avgSize-1)) { 	// Rolling array for avg; reset update counter
			arrIndex = 0;
		}
 		else {
			arrIndex++;
		}
		
		calcAvgs();
				
		ROS_INFO("x value: [%f]", xAvg);
		ROS_INFO("y value: [%f]", yAvg);
		ROS_INFO("Yaw value: [%f]", yawAvg);
		ROS_INFO("Origin tag num: [%d]\n", tagID); 
	}
	else {
		ROS_INFO("Cannot see marker");
	}
}

/*
@param: IRL target coords (x,y,theta), cmd_vel topic publisher

Linear PID controller - Moves Roomba in straight line to target
Publish contorls to cmd_vel
Refreshes loc info with spinOnce()
Terminate on reaching destination
*/
void moveTo(double xTarget, double yTarget, ros::Publisher vel_pub, ros::Rate r) {
	double k_p = 0.5;
	double k_d = 0.0;

	geometry_msgs::Twist t;

	double xCurr = (double) xAvg;	// Typecast loc values
	double yCurr = (double) yAvg;
	
	double xToTar = xTarget - xCurr;
	double yToTar = yTarget - yCurr;
	
	// Convert stuff to polar
	double rError = sqrt((xToTar^2) + (yToTar)^2);
	double angleToTar = atan2(yToTar, xToTar);
	
	double thetaCurr = (double) yawAvg;

	double prevError = 0.0;
	double deltaErr = 0.0;
	while (rError > 0.1) {
		// If not facing target, turn to target
		if ((fabs(angleToTar - thetaCurr)) > 0.02) {
			rotateTo (angleToTar, vel_pub);
		}

		// Move to target
		deltaErr = rError - prevError;
		t.linear.x = (float) ((k_p*rError) + (k_d*rError));
		if (t.linear.x > 0.5) {
			t.linear.x = 0.5;
		}
		
		// Refresh pose values
		ros::spinOnce();
		r.sleep();
		double xCurr = (double) xAvg;	// Typecast variables
		double yCurr = (double) yAvg;
		double thetaCurr = (double) yawAvg;

		double xToTar = xTarget - xCurr;	// XY errors
		double yToTar = yTarget - yCurr;

		prevError = rError;
		double rError = sqrt((xToTar^2) + (yToTar)^2);	// Convert to polar
		double angleToTar = atan2(yToTar, xToTar);
	}
}

/*
@param: IRL target angle, cmd_vel topic publisher

Rotational PID controller - Rotates Roomba to target angle
Publish controls to cmd_vel
Refreshes loc info with spinOnce()
Termiante on reaching destination
*/
void rotateTo(double thetaTarget, ros::Publisher vel_pub, ros::Rate r) {
	double k_p = 0.5;
	double k_d = 0.0;

	geometry_msgs::Twist t;
	
	double curr = (double) yawAvg;
	double prevError = 0.0;
	double deltaErr = 0.0;
	double error = 0.0;

	double abs_error = thetaTarget - curr;
	if (abs_error <= 3.1415926535897923) {
		error = abs_error; 
	}
	else {
		error = abs_error - (2*3.1415926535897923);
	} 

	while (fabs(error) > 0.03) {
		deltaErr = error - prevError;
		t.angular.z = (float) ((k_p*error) + (k_d*deltaErr));
		if (t.angular.z < -4.25 || t.angular.z > 4.25) {
			if (t.angular.z > 4.25) {
				t.angular.z = 4.25;
			}
			else {
				t.angular.z = -4.25;
			}
		}
		
		t.angular.z = -t.angular.z;
		vel_pub.publish(t);
		
		ros::spinOnce();
		r.sleep();
		prevError = error;
		curr = (double) yawAvg;
		abs_error = thetaTarget - curr;
		if (abs_error <= 3.1415926535897923) {
			error = abs_error; 
		}
		else {
			error = abs_error - (2*3.1415926535897923);
		}
	}
}

int main(int argc, char** argv) {
	
	//------------------------------------------
	// Ros node initialization
	//------------------------------------------
	ros::init(argc, argv, "move_node");
	ros::NodeHandle nh;

	//------------------------------------------
	// Read in launch parameters
	//------------------------------------------
	//nh.getParam("map_file", filepath);
	fillTagLocs(filepath);
	
	//------------------------------------------
	// Intialize pubs and subs
	//------------------------------------------
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	ros::Subscriber aruco_sub = nh.subscribe("aruco_poses", 1000, updatePos);
	ros::Rate r(10);  // 10 Hz
	for (int i = 0; i <= avgSize+1; i++) { // Completely fill avg arrays
		ros::spinOnce;
		r.sleep();
	}
	
	//------------------------------------------
	// Read in target location/orientation
	//------------------------------------------
	double xTarget, yTarget, thetaTarget;
	std::cout << "Insert target x: ";
	std::cin >> xTarget;
	std::cout << "Insert target y: ";
	std::cin >> yTarget;
	std::cout << "Insert target angle (radians): ";
	std::cin >> thetaTarget;

	std::cout << "Targets(x,y,theta): " << xTarget << "\t" << yTarget << "\t" << thetaTarget << std::endl;
	
	//------------------------------------------
	// Execute move commands
	//------------------------------------------
	double disError;
	while (ros::ok()) {
		//------------------------------------------
		// Error calculations
		//------------------------------------------
		disError = sqrt((xTarget-xAvg)^2 + (yTarget-yAvg)^2);
		if (disError <= 0.1) {
			rotateTo(thetaTarget, vel_pub);
			ROS_INFO("Destination reached. Shutting down...");
			ros::shutdown();
		}
		else {
			moveTo(xTarget, yTarget, vel_pub, r);
		}
		
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
