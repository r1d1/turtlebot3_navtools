#ifndef ACTIONTOMOVE_H
#define ACTIONTOMOVE_H

#include <iostream>
#include <ctime>
#include <sys/time.h>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <rosgraph_msgs/Clock.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <kobuki_msgs/BumperEvent.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "BP_experiment/Actions.h"
#include "BP_experiment/ValidActions.h"

#include "tf/transform_listener.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

//#include "obstacle2.cpp"
// are we using gazebo?
#define SIM false  // is this a simulation?

// define the default number of directions :
#define DEFAULT_NBDIRECTIONS 36
#define DEFAULT_MAXSPEED 0.3

//namespace act2mov {
class ActToMove2
{
	private :
		// The node handle we'll be using
		ros::NodeHandle nh;

		ros::Publisher cmd_vel_pub_;
		ros::Publisher actionFinished_pub;
		ros::Publisher actiondir_pub;
		ros::Publisher validactions_pub;
		
		// This is actually used to time behavior re-evaluation, not has a real cb:
		ros::Subscriber baseScan_sub;
		ros::Subscriber bumpers_sub;
		ros::Subscriber kinectpcl_sub;
		// Subscriptions :
		ros::Subscriber actionToDo_sub;
		ros::Subscriber pose_sub;
		ros::Subscriber control_sub;
		ros::Subscriber clock_sub_;
	
		ros::Timer control_timer;

		laser_geometry::LaserProjection projector;
		sensor_msgs::PointCloud laserscanToPCL;
		sensor_msgs::PointCloud leftCluster;
		sensor_msgs::PointCloud centCluster;
		sensor_msgs::PointCloud rightCluster;
	  
		float leftObsPercent;
		float centObsPercent;
		float rightObsPercent; 

		tf::TransformListener listener;
		tf::StampedTransform transform_base;
		
		std_msgs::Float32 behav_cmd;
		BP_experiment::Actions receivedAction;
		geometry_msgs::PoseWithCovarianceStamped receivedPose;
		geometry_msgs::Twist base_cmd;
	
		sensor_msgs::LaserScan baseLaserData;
		float scanDataRate; // how trustable are laser data ?
		bool bumperData[3];
		int bumperEdgeData[3];
		sensor_msgs::PointCloud pclData;

		// To parametrize ACTIONDISTANCE
		float actionDistance;

		float max_lin_vel;
		float max_ang_vel;
		float prev_ang_vel;
		int direction;

		// PID controller :
		float angular_error, prev_angular_error, delta_angular_error, sum_angular_error, Kp, Kd, Ki;
		float pos_error, pos_goal_x, pos_goal_y;
		// To drive a certiain distance : 
		float pos_dist, pos_start_x, pos_start_y;
		float pos_dist_b, pos_start_b_x, pos_start_b_y;
		float previous_tilt_angle, previous_tilt_diff;
		float targetAngle;
		ros::Time commandStart;
		// random explo : 
		//ros::Time begin;
		int accuRandom;
		int deltaAccu;

		// Controller param:
		int nbOfDirections; // Define the number of angles the robot can go to, aka How many actions are available
		// Command token :
		bool commandTokenAvailable;
		bool navActionToken;
		bool enable;
		bool explo;

		// This is le bordel :
		bool backwards;
		//bool backStep;
		int backStep;
		bool backRight;
		bool backLeft;
		bool done;
		bool bumped;
		
		bool noLaserData;
		int rotateTo;
		
		float backwardDist;
		int navStep;
		bool initialisePose;
		bool smooth;

		// Old code by Scarlett, probably not useful here :
		double clck; 
		float filtre_diff_close;
		float old_tourn;
	  
		bool ftg;
		ros::Time begin;
		ros::Time begin2;
		ros::Time debut_periode_tilt;
		int close_obstacle_tilt;
		int ralenti_obstacle_tilt;
		bool first;
		int old_k;
		float old_y; 
		bool croissant;
		ros::Time t2;
	
	public :
		//ActToMove2(ros::NodeHandle &nh, Obstacle *obst);
		//ActToMove2(ros::NodeHandle &nh_, Obstacle* obst, int a, float s, float p, float i, float d);
		ActToMove2(ros::NodeHandle &nh_, int a, float p, float i, float d, bool e, float ad, bool iR);
		//void orientTowards(float goalAngle);
		float orientTowards(float goalAngle);
	  	void behavior();
	  	void behavior2();
	  	//previous name of drive() : void drive_explo();
	  	void drive();
		// useless as we use drive :
	  	void drive_action();
		void computeObstruction();
	  	// useless on turtlebot :
		//void behavior(Obstacle* obst);
	  	bool run();
//	  	bool drive(ros::NodeHandle &node);
	  	void bLaserCallback(const sensor_msgs::LaserScan & msg) ;
	  	void bumperCallback(const kobuki_msgs::BumperEvent & msg) ;
//	  	void bumperCallback(const sensor_msgs::PointCloud2 & msg) ;
	  	int getBumpCode();
	  	void processBumpCode(int c) ;
	  	void pcl_received(const sensor_msgs::PointCloud2 & msg) ;
		void actionCallback(BP_experiment::Actions msg);
		void poseCallback(geometry_msgs::PoseWithCovarianceStamped msg);
		void controlCallback(std_msgs::Bool msg);
		void timerCallback(const ros::TimerEvent & msg);
	  	
	  	//float behav_3(bool first);
	  	
	  	void getSimTime(const rosgraph_msgs::Clock& msg){ clck=msg.clock.now().toSec(); }
	
		void setMaxSpeedValues(float, float);		
};

//}

#endif
