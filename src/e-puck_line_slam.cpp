// line_following example
// this example reproduce the e-puck_line_demo example
// but uses the ROS controller on the e-puck instead.
// The node connect to an e-puck and then uses values from its sensors
// to follow and line and get around obstacles.
// the duration of the example is given as argument to the node.

#include "ros/ros.h"

#include "nodes/sensor_set.h"
#include "nodes/differential_wheels_enable_control.h"
#include "nodes/robot_set_time_step.h"
#include "nodes/led_set.h"

#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/UInt8MultiArray.h>
#include <cstdlib>
#include <signal.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <sstream>

// Global defines
#define TRUE 1
#define FALSE 0
#define NO_SIDE -1
#define LEFT 0
#define RIGHT 1
#define WHITE 0
#define BLACK 1
#define SIMULATION 0            // for wb_robot_get_mode() function
#define REALITY 2               // for wb_robot_get_mode() function
#define TIME_STEP  32           // [ms]

#define WHEEL_DISTANCE 0.053    // Distance between wheels in meters (axis length); it's the same value as "WHEEL_SEPARATION" but expressed in meters.
#define WHEEL_DIAMETER 4        // cm.
#define WHEEL_CIRCUMFERENCE ((WHEEL_DIAMETER*M_PI)/100.0)    // Wheel circumference (meters).
#define MOT_STEP_DIST (WHEEL_CIRCUMFERENCE/1000.0)      // Distance for each motor step (meters); a complete turn is 1000 steps (0.000125 meters per step (m/steps)).
#define ROBOT_RADIUS 0.035 // meters.

#define DEBUG_ROS_PARAMS 1
#define DEBUG_UPDATE_SENSORS_DATA 0
#define DEBUG_ODOMETRY 0

std::string epuckname;
ros::Time currentTimeMap, lastTimeMap;


// 8 IR proximity sensors
#define NB_DIST_SENS 8
#define PS_RIGHT_00 0
#define PS_RIGHT_45 1
#define PS_RIGHT_90 2
#define PS_RIGHT_REAR 3
#define PS_LEFT_REAR 4
#define PS_LEFT_90 5
#define PS_LEFT_45 6
#define PS_LEFT_00 7

int psValue[NB_DIST_SENS]={0,0,0,0,0,0,0,0};
const int PS_OFFSET_SIMULATION[NB_DIST_SENS] = {300,300,300,300,300,300,300,300};
const int PS_OFFSET_REALITY[NB_DIST_SENS] = {480,170,320,500,600,680,210,640};

int proxData[8];
ros::Publisher proxPublisher[8];
sensor_msgs::Range proxMsg[8];
ros::Publisher laserPublisher;
sensor_msgs::LaserScan laserMsg;

// 3 IR ground color sensors
#define NB_GROUND_SENS 3
#define GS_WHITE 900
#define GS_LEFT 0
#define GS_CENTER 1
#define GS_RIGHT 2
unsigned short gsValue[NB_GROUND_SENS]={0,0,0};

// LEDs
#define NB_LEDS    10

// Encoders
int encValues[2]={0,0};
ros::Publisher odomPublisher;
nav_msgs::Odometry odomMsg;
double xPos, yPos, theta;
double deltaSteps, deltaTheta;
double leftStepsDiff = 0, rightStepsDiff = 0;
double leftStepsPrev = 0, rightStepsPrev = 0;
ros::Time currentTime, lastTime;


//------------------------------------------------------------------------------
//
//    ROS CALLBACKS
//
//------------------------------------------------------------------------------

static char modelList[10][100];
static int count=0;
static int countDist=NB_DIST_SENS;
static int countGnd=NB_GROUND_SENS;
static int countEnc=2;
static int step=TIME_STEP;
double accelerometerValues[3]={0, 0, 0};
std::vector<unsigned char> image;

ros::ServiceClient setTimeStepClient;
nodes::robot_set_time_step setTimeStepSrv;

void updateRosInfo() {
	
	static tf::TransformBroadcaster br;
		
	int i = 0;

	// Proximity handling.

	if(DEBUG_UPDATE_SENSORS_DATA)std::cout << "[" << epuckname << "] " << "prox: " << proxData[0] << "," << proxData[1] << "," << proxData[2] << "," << proxData[3] << "," << proxData[4] << "," << proxData[5] << "," << proxData[6] << "," << proxData[7] << std::endl;

	for(i=0; i<8; i++) {
		if(proxData[i] > 0) {
    		proxMsg[i].range = 0.5/sqrt(proxData[i]);  // Transform the analog value to a distance value in meters (given from field tests).
		} else {
			proxMsg[i].range = proxMsg[i].max_range;
		}
		if(proxMsg[i].range > proxMsg[i].max_range) {
			proxMsg[i].range = proxMsg[i].max_range;
		}
		if(proxMsg[i].range < proxMsg[i].min_range) {
			proxMsg[i].range = proxMsg[i].min_range;
		}
		proxMsg[i].header.stamp = ros::Time::now();
		proxPublisher[i].publish(proxMsg[i]);
	}

	// e-puck proximity positions (cm), x pointing forward, y pointing left
	//           P7(3.5, 1.0)   P0(3.5, -1.0)
	//       P6(2.5, 2.5)           P1(2.5, -2.5)
	//   P5(0.0, 3.0)                   P2(0.0, -3.0)
	//       P4(-3.5, 2.0)          P3(-3.5, -2.0)
	//
	// e-puck proximity orentations (degrees)
	//           P7(10)   P0(350)
	//       P6(40)           P1(320)
	//   P5(90)                   P2(270)
	//       P4(160)          P3(200)
	std::stringstream parent;
	std::stringstream child;
	tf::Transform transform;
	tf::Quaternion q;
        
	transform.setOrigin( tf::Vector3(0.035, -0.010, 0.034) );        
	q.setRPY(0, 0, 6.11);
	transform.setRotation(q);        
	parent << epuckname << "/base_prox0";
	child << epuckname << "/base_link";
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), child.str(), parent.str()));
	
	transform.setOrigin( tf::Vector3(0.025, -0.025, 0.034) );        
	q.setRPY(0, 0, 5.59);
	transform.setRotation(q);
	parent.str("");
	parent << epuckname << "/base_prox1";
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), child.str(), parent.str()));
	
	transform.setOrigin( tf::Vector3(0.000, -0.030, 0.034) );        
	q.setRPY(0, 0, 4.71);
	transform.setRotation(q);
	parent.str("");
	parent << epuckname << "/base_prox2";
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), child.str(), parent.str()));
	
	transform.setOrigin( tf::Vector3(-0.035, -0.020, 0.034) );        
	q.setRPY(0, 0, 3.49);
	transform.setRotation(q);
	parent.str("");
	parent << epuckname << "/base_prox3";
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), child.str(), parent.str()));
	
	transform.setOrigin( tf::Vector3(-0.035, 0.020, 0.034) );        
	q.setRPY(0, 0, 2.8);
	transform.setRotation(q);
	parent.str("");
	parent << epuckname << "/base_prox4";
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), child.str(), parent.str()));
	
	transform.setOrigin( tf::Vector3(0.000, 0.030, 0.034) );        
	q.setRPY(0, 0, 1.57);
	transform.setRotation(q);
	parent.str("");
	parent << epuckname << "/base_prox5";
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), child.str(), parent.str()));
	
	transform.setOrigin( tf::Vector3(0.025, 0.025, 0.034) );        
	q.setRPY(0, 0, 0.70);
	transform.setRotation(q);
	parent.str("");
	parent << epuckname << "/base_prox6";
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), child.str(), parent.str()));
	
	transform.setOrigin( tf::Vector3(0.035, 0.010, 0.034) );        
	q.setRPY(0, 0, 0.17);
	transform.setRotation(q);
	parent.str("");
	parent << epuckname << "/base_prox7";
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), child.str(), parent.str()));
	

	currentTimeMap = ros::Time::now();
	parent.str("");
	parent << epuckname << "/base_laser";
	//populate the LaserScan message
	laserMsg.header.stamp = ros::Time::now();
	laserMsg.header.frame_id = parent.str();
	laserMsg.angle_min = -M_PI/2.0;
	laserMsg.angle_max = M_PI/2.0;
	laserMsg.angle_increment = M_PI/18.0; // 10 degrees.
	//laserMsg.time_increment = (currentTimeMap-lastTimeMap).toSec()/180; //0.003; //(1 / laser_frequency) / (num_readings);
	//laserMsg.scan_time = (currentTimeMap-lastTimeMap).toSec();
	// The laser is placed in the center of the robot, but the proximity sensors are placed around the robot thus add "ROBOT_RADIUS" to get correct values.
	laserMsg.range_min = 0.005+ROBOT_RADIUS; // 0.5 cm + ROBOT_RADIUS.
	laserMsg.range_max = 0.05+ROBOT_RADIUS; // 5 cm + ROBOT_RADIUS. 
	laserMsg.ranges.resize(19);
	laserMsg.intensities.resize(19);
	lastTimeMap = ros::Time::now();
	
	// We use the information from the 6 proximity sensors on the front side of the robot to get 19 laser scan points. The interpolation used is the following:
	// -90 degrees: P2
	// -80 degrees: 4/5*P2 + 1/5*P1
	// -70 degrees: 3/5*P2 + 2/5*P1
	// -60 degrees: 2/5*P2 + 3/5*P1
	// -50 degrees: 1/5*P2 + 4/5*P1
	// -40 degrees: P1
	// -30 degrees: 2/3*P1 + 1/3*P0
	// -20 degrees: 1/3*P1 + 2/3*P0
	// -10 degrees: P0
	// 0 degrees: 1/2*P0 + 1/2*P7
	// 10 degrees: P7
	// 20 degrees: 1/3*P6 + 2/3*P7
	// 30 degrees: 2/3*P6 + 1/3*P7
	// 40 degrees: P6
	// 50 degrees: 1/5*P5 + 4/5*P6
	// 60 degrees: 2/5*P5 + 3/5*P6
	// 70 degrees: 3/5*P5 + 2/5*P6
	// 80 degrees: 4/5*P5 + 1/5*P6
	// 90 degrees: P5
	
	float tempProx;
	tempProx = proxData[2];
	if(tempProx > 0) {   
		laserMsg.ranges[0] = (0.5/sqrt(tempProx))+ROBOT_RADIUS; // Transform the analog value to a distance value in meters (given from field tests).
		laserMsg.intensities[0] = tempProx; 
	} else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
		laserMsg.ranges[0] = laserMsg.range_max;
		laserMsg.intensities[0] = 0;
	}

	tempProx = proxData[2]*4/5 + proxData[1]*1/5;
	if(tempProx > 0) {   
		laserMsg.ranges[1] = (0.5/sqrt(tempProx))+ROBOT_RADIUS; // Transform the analog value to a distance value in meters (given from field tests).
		laserMsg.intensities[1] = tempProx; 
	} else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
		laserMsg.ranges[1] = laserMsg.range_max;
		laserMsg.intensities[1] = 0;
	}
	
	tempProx = proxData[2]*3/5 + proxData[1]*2/5;
	if(tempProx > 0) {   
		laserMsg.ranges[2] = (0.5/sqrt(tempProx))+ROBOT_RADIUS; // Transform the analog value to a distance value in meters (given from field tests).
		laserMsg.intensities[2] = tempProx; 
	} else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
		laserMsg.ranges[2] = laserMsg.range_max;
		laserMsg.intensities[2] = 0;
	}
	
	tempProx = proxData[2]*2/5 + proxData[1]*3/5;
	if(tempProx > 0) {   
		laserMsg.ranges[3] = (0.5/sqrt(tempProx))+ROBOT_RADIUS; // Transform the analog value to a distance value in meters (given from field tests).
		laserMsg.intensities[3] = tempProx; 
	} else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
		laserMsg.ranges[3] = laserMsg.range_max;
		laserMsg.intensities[3] = 0;
	}        
	
	tempProx = proxData[2]*1/5 + proxData[1]*4/5;
	if(tempProx > 0) {   
		laserMsg.ranges[4] = (0.5/sqrt(tempProx))+ROBOT_RADIUS; // Transform the analog value to a distance value in meters (given from field tests).
		laserMsg.intensities[4] = tempProx; 
	} else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
		laserMsg.ranges[4] = laserMsg.range_max;
		laserMsg.intensities[4] = 0;
	}        
	
	tempProx = proxData[1];
	if(tempProx > 0) {   
		laserMsg.ranges[5] = (0.5/sqrt(tempProx))+ROBOT_RADIUS; // Transform the analog value to a distance value in meters (given from field tests).
		laserMsg.intensities[5] = tempProx; 
	} else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
		laserMsg.ranges[5] = laserMsg.range_max;
		laserMsg.intensities[5] = 0;
	}        
	
	tempProx = proxData[1]*2/3 + proxData[0]*1/3;
	if(tempProx > 0) {   
		laserMsg.ranges[6] = (0.5/sqrt(tempProx))+ROBOT_RADIUS; // Transform the analog value to a distance value in meters (given from field tests).
		laserMsg.intensities[6] = tempProx; 
	} else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
		laserMsg.ranges[6] = laserMsg.range_max;
		laserMsg.intensities[6] = 0;
	}        
	
	tempProx = proxData[1]*1/3 + proxData[0]*2/3;
	if(tempProx > 0) {   
		laserMsg.ranges[7] = (0.5/sqrt(tempProx))+ROBOT_RADIUS; // Transform the analog value to a distance value in meters (given from field tests).
		laserMsg.intensities[7] = tempProx; 
	} else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
		laserMsg.ranges[7] = laserMsg.range_max;
		laserMsg.intensities[7] = 0;
	}         
	
	tempProx = proxData[0];
	if(tempProx > 0) {   
		laserMsg.ranges[8] = (0.5/sqrt(tempProx))+ROBOT_RADIUS; // Transform the analog value to a distance value in meters (given from field tests).
		laserMsg.intensities[8] = tempProx; 
	} else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
		laserMsg.ranges[8] = laserMsg.range_max;
		laserMsg.intensities[8] = 0;
	}         
	
	tempProx = (proxData[0]+proxData[7])>>1;
	if(tempProx > 0) {   
		laserMsg.ranges[9] = (0.5/sqrt(tempProx))+ROBOT_RADIUS; // Transform the analog value to a distance value in meters (given from field tests).
		laserMsg.intensities[9] = tempProx; 
	} else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
		laserMsg.ranges[9] = laserMsg.range_max;
		laserMsg.intensities[9] = 0;
	}         
	
	tempProx = proxData[7];
	if(tempProx > 0) {   
		laserMsg.ranges[10] = (0.5/sqrt(tempProx))+ROBOT_RADIUS; // Transform the analog value to a distance value in meters (given from field tests).
		laserMsg.intensities[10] = tempProx; 
	} else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
		laserMsg.ranges[10] = laserMsg.range_max;
		laserMsg.intensities[10] = 0;
	}         
	
	tempProx = proxData[7]*2/3 + proxData[6]*1/3;
	if(tempProx > 0) {   
		laserMsg.ranges[11] = (0.5/sqrt(tempProx))+ROBOT_RADIUS; // Transform the analog value to a distance value in meters (given from field tests).
		laserMsg.intensities[11] = tempProx; 
	} else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
		laserMsg.ranges[11] = laserMsg.range_max;
		laserMsg.intensities[11] = 0;
	}         
	
	tempProx = proxData[7]*1/3 + proxData[6]*2/3;
	if(tempProx > 0) {   
		laserMsg.ranges[12] = (0.5/sqrt(tempProx))+ROBOT_RADIUS; // Transform the analog value to a distance value in meters (given from field tests).
		laserMsg.intensities[12] = tempProx; 
	} else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
		laserMsg.ranges[12] = laserMsg.range_max;
		laserMsg.intensities[12] = 0;
	}         
	
	tempProx = proxData[6];
	if(tempProx > 0) {   
		laserMsg.ranges[13] = (0.5/sqrt(tempProx))+ROBOT_RADIUS; // Transform the analog value to a distance value in meters (given from field tests).
		laserMsg.intensities[13] = tempProx; 
	} else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
		laserMsg.ranges[13] = laserMsg.range_max;
		laserMsg.intensities[13] = 0;
	}         
	
	tempProx = proxData[6]*4/5 + proxData[5]*1/5;
	if(tempProx > 0) {   
		laserMsg.ranges[14] = (0.5/sqrt(tempProx))+ROBOT_RADIUS; // Transform the analog value to a distance value in meters (given from field tests).
		laserMsg.intensities[14] = tempProx; 
	} else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
		laserMsg.ranges[14] = laserMsg.range_max;
		laserMsg.intensities[14] = 0;
	}   
	
	tempProx = proxData[6]*3/5 + proxData[5]*2/5;
	if(tempProx > 0) {   
		laserMsg.ranges[15] = (0.5/sqrt(tempProx))+ROBOT_RADIUS; // Transform the analog value to a distance value in meters (given from field tests).
		laserMsg.intensities[15] = tempProx; 
	} else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
		laserMsg.ranges[15] = laserMsg.range_max;
		laserMsg.intensities[15] = 0;
	}                      
	
	tempProx = proxData[6]*2/5 + proxData[5]*3/5;
	if(tempProx > 0) {   
		laserMsg.ranges[16] = (0.5/sqrt(tempProx))+ROBOT_RADIUS; // Transform the analog value to a distance value in meters (given from field tests).
		laserMsg.intensities[16] = tempProx; 
	} else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
		laserMsg.ranges[16] = laserMsg.range_max;
		laserMsg.intensities[16] = 0;
	}          
	
	tempProx = proxData[6]*1/5 + proxData[5]*4/5;
	if(tempProx > 0) {   
		laserMsg.ranges[17] = (0.5/sqrt(tempProx))+ROBOT_RADIUS; // Transform the analog value to a distance value in meters (given from field tests).
		laserMsg.intensities[17] = tempProx; 
	} else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
		laserMsg.ranges[17] = laserMsg.range_max;
		laserMsg.intensities[17] = 0;
	}          
	
	tempProx = proxData[5];
	if(tempProx > 0) {   
		laserMsg.ranges[18] = (0.5/sqrt(tempProx))+ROBOT_RADIUS; // Transform the analog value to a distance value in meters (given from field tests).
		laserMsg.intensities[18] = tempProx; 
	} else { // Sometimes the values could be negative due to the calibration, it means there is no obstacles.
		laserMsg.ranges[18] = laserMsg.range_max;
		laserMsg.intensities[18] = 0;
	}          
	
	for(i=0; i<19; i++) {
		if(laserMsg.ranges[i] > laserMsg.range_max) {
			laserMsg.ranges[i] = laserMsg.range_max;
		}
		if(laserMsg.ranges[i] < laserMsg.range_min) {
			laserMsg.ranges[i] = laserMsg.range_min;
		}
	}
	
	transform.setOrigin( tf::Vector3(0.0, 0.0, 0.034) );        
	q.setRPY(0, 0, 0);
	transform.setRotation(q);        
	parent.str("");
	child.str("");
	parent << epuckname << "/base_laser";
	child << epuckname << "/base_link";
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), child.str(), parent.str()));
	
	laserPublisher.publish(laserMsg);
	
	
	// Odometry handling.
    if(DEBUG_ODOMETRY) std::cout << "[" << epuckname << "] " << "enc left: " << encValues[0] << ", enc right: " << encValues[1] << std::endl;
    
    // Compute odometry.
	leftStepsDiff = encValues[0]*MOT_STEP_DIST - leftStepsPrev; // Expressed in meters.
	rightStepsDiff = encValues[1]*MOT_STEP_DIST - rightStepsPrev;   // Expressed in meters.
	if(DEBUG_ODOMETRY)std::cout << "[" << epuckname << "] " << "left, right steps diff: " << leftStepsDiff << ", " << rightStepsDiff << std::endl;
	
	deltaTheta = (rightStepsDiff - leftStepsDiff)/WHEEL_DISTANCE;   // Expressed in radiant.
	deltaSteps = (rightStepsDiff + leftStepsDiff)/2;        // Expressed in meters.
	if(DEBUG_ODOMETRY)std::cout << "[" << epuckname << "] " << "delta theta, steps: " << deltaTheta << ", " << deltaSteps << std::endl;

	xPos += deltaSteps*cos(theta + deltaTheta/2);   // Expressed in meters.
	yPos += deltaSteps*sin(theta + deltaTheta/2);   // Expressed in meters.
	theta += deltaTheta;    // Expressed in radiant.
	if(DEBUG_ODOMETRY)std::cout << "[" << epuckname << "] " << "x, y, theta: " << xPos << ", " << yPos << ", " << theta << std::endl;
	
	leftStepsPrev = encValues[0]*MOT_STEP_DIST;     // Expressed in meters.
	rightStepsPrev = encValues[1]*MOT_STEP_DIST;    // Expressed in meters.

	// Publish the odometry message over ROS.
	odomMsg.header.stamp = ros::Time::now();
	odomMsg.header.frame_id = "odom";
	std::stringstream ss;
	ss << epuckname << "/base_link";
	odomMsg.child_frame_id = ss.str();
	odomMsg.pose.pose.position.x = xPos;       
	odomMsg.pose.pose.position.y = yPos;
	odomMsg.pose.pose.position.z = 0;
	// Since all odometry is 6DOF we'll need a quaternion created from yaw.
	geometry_msgs::Quaternion odomQuat = tf::createQuaternionMsgFromYaw(theta);
	odomMsg.pose.pose.orientation = odomQuat;
	currentTime = ros::Time::now();
	odomMsg.twist.twist.linear.x = deltaSteps / ((currentTime-lastTime).toSec());   // "deltaSteps" is the linear distance covered in meters from the last update (delta distance);
																					// the time from the last update is measured in seconds thus to get m/s we multiply them.
	odomMsg.twist.twist.angular.z = deltaTheta / ((currentTime-lastTime).toSec());  // "deltaTheta" is the angular distance covered in radiant from the last update (delta angle);
																					// the time from the last update is measured in seconds thus to get rad/s we multiply them.
	if(DEBUG_ODOMETRY)std::cout << "[" << epuckname << "] " << "time elapsed = " << (currentTime-lastTime).toSec() << " seconds" << std::endl;
	lastTime = ros::Time::now();

	odomPublisher.publish(odomMsg);
	
	// Publish the transform over tf.
	geometry_msgs::TransformStamped odomTrans;
	odomTrans.header.stamp = odomMsg.header.stamp;
	odomTrans.header.frame_id = odomMsg.header.frame_id;
	odomTrans.child_frame_id = odomMsg.child_frame_id;
	odomTrans.transform.translation.x = xPos;
	odomTrans.transform.translation.y = yPos;
	odomTrans.transform.translation.z = 0.0;
	odomTrans.transform.rotation = odomQuat;
	br.sendTransform(odomTrans);	
	
}

void encodersCallback(const std_msgs::Float64MultiArray::ConstPtr& value) {
	encValues[0] = value->data[0];
	encValues[1] = value->data[1];
	countEnc=2;
}

void psCallback(const std_msgs::Float64::ConstPtr& value)
{
  if (countDist < NB_DIST_SENS) {
		proxData[countDist] = value->data;
    psValue[countDist]=((value->data - PS_OFFSET_SIMULATION[countDist]) < 0) ? 0 : (value->data - PS_OFFSET_SIMULATION[countDist]);
	}
  countDist++;
}

void gsCallback(const std_msgs::Float64::ConstPtr& value)
{
  if (countGnd < NB_GROUND_SENS)
    gsValue[countGnd]=value->data;
  countGnd++;
}

void modelNameCallback(const std_msgs::String::ConstPtr& name)
{
  count++;
  strcpy(modelList[count],name->data.c_str());
  ROS_INFO("model #%d : %s",count,name->data.c_str());
}

void quit(int sig)
{
  setTimeStepSrv.request.step = 0;
  setTimeStepClient.call(setTimeStepSrv);
  ROS_INFO("user stopped the node e-puck_line.");
  ros::shutdown();
  exit(0);
}


//------------------------------------------------------------------------------
//
//    BEHAVIORAL MODULES
//
//------------------------------------------------------------------------------

////////////////////////////////////////////
// LFM - Line Following Module
//
// This module implements a very simple, Braitenberg-like behavior in order
// to follow a black line on the ground. Output speeds are stored in
// lfm_speed[LEFT] and lfm_speed[RIGHT].

int lfm_speed[2];

#define LFM_FORWARD_SPEED 200
#define LFM_K_GS_SPEED 0.4

void LineFollowingModule(void)
{
  int DeltaS=0;
  
  DeltaS = gsValue[GS_RIGHT]-gsValue[GS_LEFT];
  
  lfm_speed[LEFT]  = LFM_FORWARD_SPEED - LFM_K_GS_SPEED * DeltaS;
  lfm_speed[RIGHT] = LFM_FORWARD_SPEED + LFM_K_GS_SPEED * DeltaS;
}


////////////////////////////////////////////
// OAM - Obstacle Avoidance Module
//
// The OAM routine first detects obstacles in front of the robot, then records
// their side in "oam_side" and avoid the detected obstacle by 
// turning away according to very simple weighted connections between
// proximity sensors and motors. "oam_active" becomes active when as soon as 
// an object is detected and "oam_reset" inactivates the module and set 
// "oam_side" to NO_SIDE. Output speeds are in oam_speed[LEFT] and oam_speed[RIGHT].

int oam_active, oam_reset;
int oam_speed[2];
int oam_side = NO_SIDE;

#define OAM_OBST_THRESHOLD 100
#define OAM_FORWARD_SPEED 150
#define OAM_K_PS_90 0.2
#define OAM_K_PS_45 0.9
#define OAM_K_PS_00 1.2
#define OAM_K_MAX_DELTAS 600

void ObstacleAvoidanceModule(void)
{
  int max_ds_value, DeltaS=0, i;
  int Activation[]={0,0};

  // Module RESET
  if (oam_reset)
  {
    oam_active = FALSE;
    oam_side = NO_SIDE;
  }
  oam_reset = 0;

  // Determine the presence and the side of an obstacle
  max_ds_value = 0;
  for (i = PS_RIGHT_00; i <= PS_RIGHT_45; i++) {
    if (max_ds_value < psValue[i]) max_ds_value = psValue[i];
    Activation[RIGHT] += psValue[i];
  }
  for (i = PS_LEFT_45; i <= PS_LEFT_00; i++) {
    if (max_ds_value < psValue[i]) max_ds_value = psValue[i];
    Activation[LEFT] += psValue[i];
  }
  if (max_ds_value > OAM_OBST_THRESHOLD) oam_active = TRUE;
  
  if (oam_active && oam_side == NO_SIDE) // check for side of obstacle only when not already detected
  {
    if (Activation[RIGHT] > Activation[LEFT]) oam_side = RIGHT;
    else oam_side = LEFT;
  }

  // Forward speed
  oam_speed[LEFT]  = OAM_FORWARD_SPEED;
  oam_speed[RIGHT] = OAM_FORWARD_SPEED;

  // Go away from obstacle
  if (oam_active)
  {
    // The rotation of the robot is determined by the location and the side of the obstacle
    if (oam_side == LEFT)
    {
      DeltaS -= (int) (OAM_K_PS_90 * psValue[PS_LEFT_90]); //(((psValue[PS_LEFT_90]-PS_OFFSET)<0)?0:(psValue[PS_LEFT_90]-PS_OFFSET)));
      DeltaS -= (int) (OAM_K_PS_45 * psValue[PS_LEFT_45]); //(((psValue[PS_LEFT_45]-PS_OFFSET)<0)?0:(psValue[PS_LEFT_45]-PS_OFFSET)));
      DeltaS -= (int) (OAM_K_PS_00 * psValue[PS_LEFT_00]); //(((psValue[PS_LEFT_00]-PS_OFFSET)<0)?0:(psValue[PS_LEFT_00]-PS_OFFSET)));
    }
    else // oam_side == RIGHT
    {
      DeltaS += (int) (OAM_K_PS_90 * psValue[PS_RIGHT_90]);  //(((psValue[PS_RIGHT_90]-PS_OFFSET)<0)?0:(psValue[PS_RIGHT_90]-PS_OFFSET)));
      DeltaS += (int) (OAM_K_PS_45 * psValue[PS_RIGHT_45]);  //(((psValue[PS_RIGHT_45]-PS_OFFSET)<0)?0:(psValue[PS_RIGHT_45]-PS_OFFSET)));
      DeltaS += (int) (OAM_K_PS_00 * psValue[PS_RIGHT_00]);  //(((psValue[PS_RIGHT_00]-PS_OFFSET)<0)?0:(psValue[PS_RIGHT_00]-PS_OFFSET)));
    }
    if (DeltaS > OAM_K_MAX_DELTAS) DeltaS = OAM_K_MAX_DELTAS;
    if (DeltaS < -OAM_K_MAX_DELTAS) DeltaS = -OAM_K_MAX_DELTAS;

    // Set speeds
    oam_speed[LEFT] -= DeltaS;
    oam_speed[RIGHT] += DeltaS;
  }
}


////////////////////////////////////////////
// LLM - Line Leaving Module
//
// Since it has no output, this routine is not completely finished. It has
// been designed to monitor the moment while the robot is leaving the
// track and signal to other modules some related events. It becomes active
// whenever the "side" variable displays a rising edge (changing from -1 to 0 or 1).

int llm_active=FALSE, llm_inibit_ofm_speed, llm_past_side=NO_SIDE;
int lem_reset;

#define LLM_THRESHOLD 800

void LineLeavingModule(int side)
{
  // Starting the module on a rising edge of "side"
  if (!llm_active && side!=NO_SIDE && llm_past_side==NO_SIDE)
    llm_active = TRUE;

  // Updating the memory of the "side" state at the previous call
  llm_past_side = side;

  // Main loop
  if (llm_active) // Simply waiting until the line is not detected anymore
  {
    if (side == LEFT)
    {
      if ((gsValue[GS_CENTER]+gsValue[GS_LEFT])/2 > LLM_THRESHOLD)  // out of line
      {
        llm_active = FALSE;
        // *** PUT YOUR CODE HERE ***
        llm_inibit_ofm_speed = FALSE;
        lem_reset = TRUE; 
      }
      else  // still leaving the line
      {
        // *** PUT YOUR CODE HERE ***
        llm_inibit_ofm_speed = TRUE;
      }
    }
    else // side == RIGHT
    {
      if ((gsValue[GS_CENTER]+gsValue[GS_RIGHT])/2 > LLM_THRESHOLD)  // out of line
      {
        llm_active = FALSE;
        // *** PUT YOUR CODE HERE ***
        llm_inibit_ofm_speed = FALSE;
        lem_reset = TRUE;
      }
      else // still leaving the line
      {
        // *** PUT YOUR CODE HERE ***
        llm_inibit_ofm_speed = TRUE;
      }
    }
  }
}


////////////////////////////////////////////
// OFM - Obstacle Following Module
//
// This function just gives the robot a tendency to steer toward the side
// indicated by its argument "side". When used in competition with OAM it
// gives rise to an object following behavior. The output speeds are
// stored in ofm_speed[LEFT] and ofm_speed[RIGHT].

int ofm_active;
int ofm_speed[2];

#define OFM_DELTA_SPEED 150

void ObstacleFollowingModule(int side)
{
  if (side != NO_SIDE)
  {
    ofm_active = TRUE;
    if (side == LEFT)
    {
      ofm_speed[LEFT]  = -OFM_DELTA_SPEED;
      ofm_speed[RIGHT] = OFM_DELTA_SPEED;
    }
    else
    {
      ofm_speed[LEFT]  = OFM_DELTA_SPEED;
      ofm_speed[RIGHT] = -OFM_DELTA_SPEED;
    }
  }
  else // side = NO_SIDE
  {
    ofm_active = FALSE;
    ofm_speed[LEFT]  = 0;
    ofm_speed[RIGHT] = 0;
  }
}


////////////////////////////////////////////
// LEM - Line Entering Module
//
// This is the most complex module (and you might find easier to re-program it
// by yourself instead of trying to understand it ;-). Its purpose is to handle
// the moment when the robot must re-enter the track (after having by-passed
// an obstacle, e.g.). It is organized like a state machine, which state is
// stored in "lem_state" (see LEM_STATE_STANDBY and following #defines).
// The inputs are (i) the two lateral ground sensors, (ii) the argument "side"
// which determines the direction that the robot has to follow when detecting
// a black line, and (iii) the variable "lem_reset" that resets the state to
// standby. The output speeds are stored in lem_speed[LEFT] and
// lem_speed[RIGHT].

#define LEM_FORWARD_SPEED 100
#define LEM_K_GS_SPEED 0.8
#define LEM_THRESHOLD 500

#define LEM_STATE_STANDBY 0
#define LEM_STATE_LOOKING_FOR_LINE 1
#define LEM_STATE_LINE_DETECTED 2
#define LEM_STATE_ON_LINE 3

int lem_active;
int lem_speed[2];
int lem_state, lem_black_counter;
int curOpGsValue, prevOpGsValue;

void LineEnteringModule(int side)
{
  int Side, OpSide, GS_Side, GS_OpSide;

  // Module reset
  if (lem_reset)
  {
    lem_state = LEM_STATE_LOOKING_FOR_LINE;
  }
  lem_reset = FALSE;

  // Initialization
  lem_speed[LEFT]  = LEM_FORWARD_SPEED;
  lem_speed[RIGHT] = LEM_FORWARD_SPEED;
  if (side==LEFT)  // if obstacle on left side -> enter line rightward
  {
    Side = RIGHT;  // line entering direction
    OpSide = LEFT;
    GS_Side = GS_RIGHT;
    GS_OpSide = GS_LEFT;
  }
  else            // if obstacle on left side -> enter line leftward
  {
    Side = LEFT;  // line entering direction
    OpSide = RIGHT;
    GS_Side = GS_LEFT;
    GS_OpSide = GS_RIGHT;
  }

  // Main loop (state machine)
  switch (lem_state)
  {
    case LEM_STATE_STANDBY:
      lem_active = FALSE;
      break;
    case LEM_STATE_LOOKING_FOR_LINE:
      if (gsValue[GS_Side]<LEM_THRESHOLD)
      {
        lem_active = TRUE;
        // set speeds for entering line
        lem_speed[OpSide] = LEM_FORWARD_SPEED;
        lem_speed[Side] = LEM_FORWARD_SPEED; // - LEM_K_GS_SPEED * gsValue[GS_Side];
        lem_state = LEM_STATE_LINE_DETECTED;
        // save ground sensor value
        if (gsValue[GS_OpSide]<LEM_THRESHOLD)
        {
          curOpGsValue = BLACK;
          lem_black_counter = 1;
        }
        else
        {
          curOpGsValue = WHITE;
          lem_black_counter = 0;
        }
        prevOpGsValue = curOpGsValue;
      }
      break;
    case LEM_STATE_LINE_DETECTED:
      // save the oposite ground sensor value
      if (gsValue[GS_OpSide]<LEM_THRESHOLD)
      {
        curOpGsValue = BLACK;
        lem_black_counter ++;
      }
      else curOpGsValue = WHITE;
      // detect the falling edge BLACK->WHITE
      if (prevOpGsValue==BLACK && curOpGsValue==WHITE)
      {
        lem_state = LEM_STATE_ON_LINE;
        lem_speed[OpSide] = 0;
        lem_speed[Side]   = 0;
      }
      else
      {
        prevOpGsValue = curOpGsValue;
        // set speeds for entering line
        lem_speed[OpSide] = LEM_FORWARD_SPEED + LEM_K_GS_SPEED * (GS_WHITE-gsValue[GS_Side]);
        lem_speed[Side] = LEM_FORWARD_SPEED - LEM_K_GS_SPEED * (GS_WHITE-gsValue[GS_Side]);
      }
      break;
    case LEM_STATE_ON_LINE:
      oam_reset = TRUE;
      lem_active = FALSE;
      lem_state = LEM_STATE_STANDBY;
      break;
  }
}



//------------------------------------------------------------------------------
//
//    CONTROLLER
//
//------------------------------------------------------------------------------

////////////////////////////////////////////
// Main
int main(int argc, char **argv)
{
  int i, speed[2], psOffset[NB_DIST_SENS]={0,0,0,0,0,0,0,0}, Mode = 1;
  int oamOfmSpeed[2];

  // ROS-specific code
  
  int speedLin = 100;
  int speedAng = 0;
  int wantedModel = 0;
  int nStep = 0;	

	double init_xpos, init_ypos, init_theta; 

	ros::init(argc, argv, "e_puck_line_slam",ros::init_options::AnonymousName);  
	ros::NodeHandle np("~"); // Private.
	ros::NodeHandle n; // Public.

  signal(SIGINT,quit);  

	np.param<std::string>("epuck_name", epuckname, "epuck");
	np.param("xpos", init_xpos, 0.0);
    np.param("ypos", init_ypos, 0.0);
    np.param("theta", init_theta, 0.0);
	
	if(DEBUG_ROS_PARAMS) {
        std::cout << "[" << epuckname << "] " << "epuck name: " << epuckname << std::endl;
        std::cout << "[" << epuckname << "] " << "init pose: " << init_xpos << ", " << init_ypos << ", " << init_theta << std::endl;
	}
	
  
  // declaration of variable names used to define services and topics name dynamically
	std::string modelName;
  geometry_msgs::Twist command;
  
  // get the name of the robot
  ros::Subscriber nameSub = n.subscribe("model_name", 100, modelNameCallback);
  while (count == 0 || count < nameSub.getNumPublishers())  {
    ros::spinOnce();
    ros::spinOnce();
    ros::spinOnce();
  }
  ros::spinOnce();
  if (count == 1)
    modelName = modelList[1];
  else {
    std::cout << "choose the # of the model you want to use :\n";
    std::cin >> wantedModel;
    if (1 <= wantedModel && wantedModel <= count)
      modelName = modelList[wantedModel];
    else  {
      ROS_ERROR("invalid choice");
      return 1;
    }
  }
  nameSub.shutdown();
  count = 0;

  
  // send robot time step to webots
  setTimeStepClient = n.serviceClient<nodes::robot_set_time_step>(modelName+"/Robot/time_step");
  setTimeStepSrv.request.step = step;

  std::vector<ros::ServiceClient> setDistSensorClient;
  nodes::sensor_set setDistSensorSrv;
  
  if (setTimeStepClient.call(setTimeStepSrv) && setTimeStepSrv.response.success == 1)
    nStep++;
  else
    ROS_ERROR("Failed to call service time_step to update robot's time step");

	// enable encoders
	ros::Subscriber SubEncoder;
	ros::ServiceClient setEncoderClient = n.serviceClient<nodes::sensor_set>(modelName+"/e_puck/set_sensor");
	nodes::sensor_set setEncoderSrv;
	setEncoderSrv.request.period = step;
	if (setEncoderClient.call(setEncoderSrv) && setEncoderSrv.response.success == 1)  {
		ROS_INFO("Encoders enabled");
		std::ostringstream s;
		s << step;
		SubEncoder = n.subscribe(modelName+"/e_puck/"+s.str(), 1, encodersCallback);
		while (SubEncoder.getNumPublishers() == 0);
	} else {
		if (setEncoderSrv.response.success == 2)
			ROS_ERROR("Sampling period is not valid");
		ROS_ERROR("Failed to encoders");
		return 1;
	}

  // enable ir proximity sensors
  char deviceName[20];
  ros::Subscriber SubDistIr[NB_DIST_SENS];
  for (i = 0; i < NB_DIST_SENS; i++)  {
    sprintf(deviceName, "ps%d", i);
    setDistSensorClient.push_back(n.serviceClient<nodes::sensor_set>(modelName+'/'+deviceName+"/set_sensor"));
    setDistSensorSrv.request.period = step;
    if (setDistSensorClient[i].call(setDistSensorSrv) && setDistSensorSrv.response.success == 1)  {
      ROS_INFO("device %s enabled", deviceName);
      std::ostringstream s;
      s << step;
      SubDistIr[i] = n.subscribe(modelName+'/'+deviceName+'/'+s.str(), 1, psCallback);
      while (SubDistIr[i].getNumPublishers() == 0);
    }
    else  {
      if (setDistSensorSrv.response.success == 2)
        ROS_ERROR("Sampling period is not valid");
      ROS_ERROR("Failed to enable %s", deviceName);
      return 1;
    }
  }
  
	for(i=0; i<8; i++) {
		std::stringstream ss;
		ss.str("");
		ss << "proximity" << i;
		proxPublisher[i] = n.advertise<sensor_msgs::Range>(ss.str(), 10);
		//proxMsg[i] = new sensor_msgs::Range();
		proxMsg[i].radiation_type = sensor_msgs::Range::INFRARED;
		ss.str("");
		ss << epuckname << "/base_prox" << i;
		proxMsg[i].header.frame_id =  ss.str();
		proxMsg[i].field_of_view = 0.26;    // About 15 degrees...to be checked!
		proxMsg[i].min_range = 0.005;       // 0.5 cm.
		proxMsg[i].max_range = 0.05;        // 5 cm.                    
	} 

	laserPublisher = n.advertise<sensor_msgs::LaserScan>("scan", 10);

  // enable ir ground sensors
  ros::Subscriber SubGndIr[NB_GROUND_SENS];
  for (i = 0; i < NB_GROUND_SENS; i++)  {
    sprintf(deviceName, "gs%d", i);
    setDistSensorClient.push_back(n.serviceClient<nodes::sensor_set>(modelName+'/'+deviceName+"/set_sensor"));
    setDistSensorSrv.request.period = step;
    if (setDistSensorClient[i + NB_DIST_SENS].call(setDistSensorSrv) && setDistSensorSrv.response.success == 1) {
      ROS_INFO("device %s enabled", deviceName);
      std::ostringstream s;
      s << step;
      SubGndIr[i] = n.subscribe(modelName+'/'+deviceName+'/'+s.str(), 1, gsCallback);
      while (SubGndIr[i].getNumPublishers() == 0);
    }
    else  {
      if (setDistSensorSrv.response.success == 2)
        ROS_ERROR("Sampling period is not valid");
      ROS_ERROR("Failed to enable %s", deviceName);
      return 1;
    }
  }

  // enable the wheels command
  ros::Publisher pub;
  ros::ServiceClient enableControlClient = n.serviceClient<nodes::differential_wheels_enable_control>(modelName+"/differential_wheels/enable_commands");
  nodes::differential_wheels_enable_control enableControlSrv;

  enableControlSrv.request.enable = 1;
  
  if (enableControlClient.call(enableControlSrv) && enableControlSrv.response.success == 1) {
    ROS_INFO("wheels enabled !");
    pub = n.advertise<geometry_msgs::Twist>(modelName+"/differential_wheels/commands", 100);
  }
  else  {
    ROS_ERROR("Failed to call service enable_control to enable them");
    return 1;
  }
  
  // turn the leds on
  std::vector<ros::ServiceClient> setLedClient;
  nodes::led_set setLedSrv;
  sprintf(deviceName, "led0");
  setLedClient.push_back(n.serviceClient<nodes::led_set>(modelName+'/'+deviceName+"/set_led"));
  setLedSrv.request.value = 1;
  if (setLedClient[0].call(setLedSrv) && setLedSrv.response.success == 1)
    ROS_INFO("%s turned on !", deviceName);
  else  {
    ROS_ERROR("Failed to call service set_led to enable %s", deviceName);
    return 1;
  }
  sprintf(deviceName, "led8");
  setLedClient.push_back(n.serviceClient<nodes::led_set>(modelName+'/'+deviceName+"/set_led"));
  setLedSrv.request.value = 1;
  if (setLedClient[1].call(setLedSrv) && setLedSrv.response.success == 1)
    ROS_INFO("%s turned on !",deviceName);
  else  {
    ROS_ERROR("Failed to call service set_led to enable %s",deviceName);
    return 1;
  }
  
	// Init odometry publisher.
	odomPublisher = n.advertise<nav_msgs::Odometry>("odom", 10);
	currentTime = ros::Time::now();
	lastTime = ros::Time::now();  
	theta = init_theta;
    xPos = init_xpos;
    yPos = init_ypos;
    
    
  oam_reset = TRUE;
  llm_active = FALSE;
  llm_past_side = NO_SIDE;
  ofm_active = FALSE;
  lem_active = FALSE;
  lem_state = LEM_STATE_STANDBY;
  for (i = 0; i < NB_DIST_SENS; i++) psOffset[i] = PS_OFFSET_SIMULATION[i];
  oam_reset = TRUE;
  oam_active = FALSE;
  
  if (setTimeStepClient.call(setTimeStepSrv) && setTimeStepSrv.response.success == 1)
    nStep++;
  else
    ROS_ERROR("Failed to call service time_step to update robot's time step");
 
  
  // Main loop
  while (1)  {
    if (!(setTimeStepClient.call(setTimeStepSrv) && setTimeStepSrv.response.success == 1))
      ROS_ERROR("Failed to call service time_step to update robot's time step");
    
    //get sensors value
    if (nStep % 1 == 0) {
      countDist = 0;
      countGnd = 0;
      countEnc = 0;
      while (countDist < NB_DIST_SENS || countGnd < NB_GROUND_SENS || countEnc < 2)
        ros::spinOnce();
    }
    
    updateRosInfo();

    // Speed initialization
    speed[LEFT] = 0;
    speed[RIGHT] = 0;

  ///////////////////////////////////////////////
  // *** START OF SUBSUMPTION ARCHITECTURE *** //
  ///////////////////////////////////////////////

    // LFM - Line Following Module
    LineFollowingModule();

    speed[LEFT]  = lfm_speed[LEFT];
    speed[RIGHT] = lfm_speed[RIGHT];

    // OAM - Obstacle Avoidance Module
    ObstacleAvoidanceModule();

    // LLM - Line Leaving Module
    LineLeavingModule(oam_side);

    // OFM - Obstacle Following Module
    ObstacleFollowingModule(oam_side);

    // Inibit A
    if (llm_inibit_ofm_speed) {
      ofm_speed[LEFT] = 0;
      ofm_speed[RIGHT] = 0;
    }

    // Sum A
    oamOfmSpeed[LEFT] = oam_speed[LEFT] + ofm_speed[LEFT];
    oamOfmSpeed[RIGHT] = oam_speed[RIGHT] + ofm_speed[RIGHT];

    // Suppression A
    if (oam_active || ofm_active) {
      speed[LEFT]  = oamOfmSpeed[LEFT];
      speed[RIGHT] = oamOfmSpeed[RIGHT];
    }
    

    // LEM - Line Entering Module
    LineEnteringModule(oam_side);

    // Suppression B
    if (lem_active) {
      speed[LEFT]  = lem_speed[LEFT];
      speed[RIGHT] = lem_speed[RIGHT];
    }


  /////////////////////////////////////////////
  // *** END OF SUBSUMPTION ARCHITECTURE *** //
  /////////////////////////////////////////////

    // Debug display
 //   if (oam_side == -1)
 //     ROS_INFO("OAM %d side NO_SIDE step %d LLM %d inibitA %d   OFM %d   LEM %d state %d oam_reset %d", oam_active, nStep, llm_active, llm_inibit_ofm_speed, ofm_active, lem_active, lem_state, oam_reset);
 //   else if (oam_side == 0)
 //     ROS_INFO("OAM %d side LEFT step %d LLM %d inibitA %d   OFM %d   LEM %d state %d oam_reset %d", oam_active, nStep, llm_active, llm_inibit_ofm_speed, ofm_active, lem_active, lem_state, oam_reset);
 //   else if (oam_side == 1)
 //     ROS_INFO("OAM %d side RIGHT step %d LLM %d inibitA %d   OFM %d   LEM %d state %d oam_reset %d", oam_active, nStep, llm_active, llm_inibit_ofm_speed, ofm_active, lem_active, lem_state, oam_reset);

	//speed[LEFT] = 500;
	//speed[RIGHT] = -500;

    speedLin = (speed[LEFT] + speed[RIGHT]) / 2;
    speedAng = (speed[RIGHT] - speed[LEFT]) / 2;
    command.linear.x = speedLin;
    command.angular.z = speedAng;
    pub.publish(command);
  }
  
  command.linear.x = 0;
  command.angular.z = 0;
  pub.publish(command);
  
  // disable the wheels command
  enableControlSrv.request.enable = 0;
  if (enableControlClient.call(enableControlSrv) && enableControlSrv.response.success == 2) {
    pub.shutdown();
    ROS_INFO("wheels disabled");
  }
  else  {
    ROS_ERROR("Failed to call service enable_control to disable them (success value return is %d)", enableControlSrv.response.success);
    return 1;
  }

  //turn off leds
  sprintf(deviceName, "led8");
  setLedSrv.request.value = 0;
  if (setLedClient[1].call(setLedSrv) && setLedSrv.response.success == 1)
    ROS_INFO("%s turned off", deviceName);
  else  {
    ROS_ERROR("Failed to call service set_led to enable %s", deviceName);
    return 1;
  }
  sprintf(deviceName, "led0");
  setLedSrv.request.value = 0;
  if (setLedClient[0].call(setLedSrv) && setLedSrv.response.success == 1)
    ROS_INFO("%s turned off", deviceName);
  else  {
    ROS_ERROR("Failed to call service set_led to enable %s", deviceName);
    return 1;
  }
  
  // disable ir proximity sensors
  for (i = 0; i < NB_DIST_SENS; i++)  {
    SubDistIr[i].shutdown();
    ROS_INFO("ds%d disabled", i);
  }
  
  // disable ir ground sensors
  for (i = 0; i < NB_GROUND_SENS; i++)  {
    SubGndIr[i].shutdown();
    ROS_INFO("gs%d disabled", i);
  }
  
  // tells Webots this node will stop using time_step service
  setTimeStepSrv.request.step = 0;
  if (setTimeStepClient.call(setTimeStepSrv) && setTimeStepSrv.response.success == 2)
    ROS_INFO("robot's time step updated to end simulation");
  else
    ROS_ERROR("Failed to call service time_step to end simulation");
  ros::shutdown();
  return 0;
}

