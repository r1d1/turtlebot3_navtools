#ifndef STATESLAM3_H
#define STATESLAM3_H

#include <iostream>
#include <cstdlib>
#include <string>
#include <fstream>
#include <sys/time.h>
#include <ctime>
#include <vector>
#include <math.h>
#include <algorithm>
#include <getopt.h>

#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include "tf/transform_listener.h"

#include <actionlib/client/simple_action_client.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#include <rosgraph_msgs/Clock.h>

#include <laser_geometry/laser_geometry.h>

#include <sensor_msgs/LaserScan.h>

#include <visualization_msgs/MarkerArray.h>

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Twist.h>

#include "habelar_msgs/StateReward.h"
#include "habelar_msgs/State.h"
#include "habelar_msgs/Reward.h"
#include "habelar_msgs/CommandSignal.h"
#include "habelar_msgs/ValidActions.h"

using std::vector;
using std::string;

class state_slam3
{
	private:
	float min1;
	int indice_min1;
	float min2;
	int indice_min2;
	float min3;
	int indice_min3;
	int it;
	int current_poly;
	bool first;
	string old_mess;
	ros::Time clock;
	ros::Time clock_pub;
	bool mobile;
	
	int iterateur;
	
	geometry_msgs::Point32 old_position;
	vector < vector<geometry_msgs::Point32> >vector_cell_poly;
	vector<geometry_msgs::Point32> vector_cell;
	vector<geometry_msgs::Point32> vector_centre_poly;
	
	vector<geometry_msgs::Point32> voronoiCenters;
	vector<float> voronoiDist;
	int activeCenter;
	int xMinCenter; 
	int xMaxCenter;
	int yMinCenter;
	int yMaxCenter;
	float cellRadius;

	float current_angle;
	geometry_msgs::PolygonStamped poly_stamp;
	geometry_msgs::Point32 current_position;

	ros::NodeHandle nh;
	ros::Subscriber Scan_filter_sub;
	ros::Subscriber reward_sub;
	ros::Subscriber actionControl_sub;
	ros::Subscriber command_sub;
	ros::Subscriber validActions_sub;

	int nbr_place_cell;
	bool first_cell;
	float dist_cell_mini_IsIn;
	tf::TransformListener Listener;
  	tf::StampedTransform transform_base;
  	sensor_msgs::LaserScan  base_laser;
  	float Reward;

	// Erwan addition for reward waiting :
	bool flagRewardReceived;
	bool flagActionFinished;
	std::ofstream poseOrientCellslog;
	ros::Time previousTime_log;
	ros::Time previousTime_map;
	std::vector<float> validActions;
	//end of addition

	//ros::ServiceClient clientNav;
	ros::Publisher Marker_pub;
	ros::Publisher Marker2_pub;
	ros::Publisher poly_pub;
	ros::Publisher statereward_pub;
	ros::Publisher state_pub;
	visualization_msgs::MarkerArray marker;
	//promethe_app::NavService srvExp;
	habelar_msgs::StateReward place_cell;
	habelar_msgs::State place_cell_no_rwd;

public:
  //! Action client initialization 
	state_slam3(ros::NodeHandle &nh_, float cellrad);
	~state_slam3();

	// Callbacks
	//void reward_received( const habelar_msgs::Reward & msg) ;
	void reward_received(const std_msgs::Float32 & msg) ;
	void actionControlCallback(const std_msgs::Bool & msg) ;
	void base_received( const sensor_msgs::LaserScan & msg);
	void controlCallback(const habelar_msgs::CommandSignal & msg) ;
	void validActionCallback( const habelar_msgs::ValidActions & msg);

	// "Voronoi"
	void computeDistToCenters();
	float computeMinDistToCenters();
	float distToActiveCenter();
	void updateActiveCenter();

	// ---------------------------------------------------------
	void create_cell();
	void gestionCell();
	int IsIn(geometry_msgs::Point32 position,vector<geometry_msgs::Point32>);
	void WhoIsCloser(int num);
	void WhoIsCloserOut(int out);
	void gestionCellPoly();
	
	// ---------------------------------------------------------
	void display();	// affiche dans rviz les liex d'apprentissage des place cell 
	void displayPoly();

	void displayVoronoiCenters();
	void displayVoronoiFrontiers();
};

#endif
