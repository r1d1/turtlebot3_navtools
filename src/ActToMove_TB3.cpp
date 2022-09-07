// Scarlett code adaptation to take action into account

#include "ActToMove_TB3.h"

#define _USE_MATH_DEFINES
#define PI 3.14159265359
// In degrees :
#define ANGULAR_ERROR 3.00
//#define ANGULAR_ERROR 3.0
#define COMMANDTIMEOUT 7.0
#define ACTIONDISTANCE 0.35   // defined as 0.5 for dist_states = 0.35
//#define ACTIONDISTANCE 1.0
// the following distance is half the PR2 width + 0.5 m (danger zone) 
#define DISTANCETHRESHOLD 1.2
#define TURTLE_HEIGHT 0.5 // the robot is 420 mm highh
#define TURTLE_RADIUS 0.15 // the robot is 354 mm width : 0.2 // TB3 is 0.2 diameter at most
#define DIST_OBSTACLE 1.0 // max distance where measured obstacle are considered

// Thresholds for random rotation when going straight
#define THRESHOLD_HIGH 300
#define THRESHOLD_LOW 0

//*******variables**********
float max_distance_stop=0.4; // 0.3 dead kinect view + 0.66 * robot diameter ; empirical value
float max_distance_slower=1.0; // 	cf "longueur couloir" de obstacle2.cpp 
int min_lin_vel=0;

float filtering_coef= 0.99999;		// avoid right-left behavior of the robot

//*******/variables**********

int sign(float value){ return ((value < 0.0) ? -1 : ((value > 0) ? 1 : 0) ); }

int angleToAction(float angle, int actionnumber)
{
	float step = 360.0 / actionnumber;
	int remaining = int(fabs(angle)) % int(step);
	int action = int(fabs(angle)) / int(step) + ((remaining > step / 2) ? 1 : 0);
	return (((angle < 0) ? actionnumber - action : action) % actionnumber);
}

float actionToAngle(float action, float actionnumber)
{
	return ((action <= (actionnumber / 2)) ? (action * 360.0 / actionnumber) : (action * 360.0 / actionnumber - 360.0) );
}

void ActToMove_TB3::setMaxSpeedValues(float a, float b){ max_lin_vel = a; max_ang_vel = b; }

//ActToMove_TB3::ActToMove_TB3(ros::NodeHandle &nh_, Obstacle* obst, int a, float s, float p, float i, float d)
ActToMove_TB3::ActToMove_TB3(ros::NodeHandle &nh_, int a, float p, float i, float d, bool e, float ad=ACTIONDISTANCE)
{
	std::cout << "param: " << a << ", " << ", " << p << ", " << i << ", " << d << std::endl;
	/* Node handle that manage the node and provide publishing and subscribing function */
	nh = nh_;
	
	//Default max values 
	max_lin_vel=0.3;
	max_ang_vel=0.3;
	prev_ang_vel=0.0;

	actionDistance = ad;

	direction = 0;	
	// Update command at 50 Hz -- temporary set to 2Hz for testing
	control_timer = nh_.createTimer(ros::Duration(1.0 / 50.0), &ActToMove_TB3::timerCallback, this);

	actionToDo_sub = nh.subscribe("actionToDo", 1, &ActToMove_TB3::actionCallback, this);
	control_sub = nh.subscribe("controlEnable", 1, &ActToMove_TB3::controlCallback, this);
	
	baseScan_sub = nh.subscribe("scan", 1, &ActToMove_TB3::bLaserCallback, this);
	 

	pose_sub = nh.subscribe("odom", 1, &ActToMove_TB3::poseCallback, this);

	ftg=false;
	first=true;
	old_tourn=0;

	std::cout << "PID / p: " <<  p << " i: " << i << " d: " << d << std::endl;
	Kp = p;
	Kd = d;
	Ki = i;
	// Param controller :
	nbOfDirections = a;
	navStep = -1;

	pos_start_x = 0.0;
	pos_start_y =0.0; 

	pos_start_b_x = 0.0;
	pos_start_b_y = 0.0;

	backwards = false;
	backwardDist = 0.2;
	done = true;
	accuRandom = 0;
	deltaAccu = 1;


	enable = true;
	explo = e;
	noLaserData = true; 

	//cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1);
	cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	actionFinished_pub = nh.advertise<std_msgs::Bool>("turtlebot3/act2move/action_finished", 1);
	actiondir_pub = nh.advertise<visualization_msgs::MarkerArray>("actionDir", 1);
	// Not used on turtlebot, valid actions should be computed in timerCallback
	validactions_pub = nh.advertise<habelar_msgs::ValidActions>("turtlebot3/act2move/valid_actions", 1);
    debug_leftCluster_pub = nh.advertise<sensor_msgs::PointCloud>("turtlebot3/act2move/debug/leftCluster", 1);;
    debug_centCluster_pub = nh.advertise<sensor_msgs::PointCloud>("turtlebot3/act2move/debug/centCluster", 1);;
    debug_rightCluster_pub = nh.advertise<sensor_msgs::PointCloud>("turtlebot3/act2move/debug/rightCluster", 1);;
}

void ActToMove_TB3::bLaserCallback( const sensor_msgs::LaserScan & msg)
{
	baseLaserData = msg;
//	std::cout << "scan frame : " << msg.header.frame_id << std::endl;
	int count = 0;
	for(int i = 0 ; i < msg.ranges.size() ; i++){ if( !std::isnan(msg.ranges[i]) ){ count += 1; } }
	scanDataRate = float(count) / msg.ranges.size();
	// Transform data into Pcl for easier reasoning :
	projector.projectLaser(msg, laserscanToPCL);
	noLaserData = !(laserscanToPCL.points.size() > 100); //# MAGIC NUMBER ON MINIMAL PC SIZE
//	std::cout << "Data from laser ? " << (noLaserData ? "NOPE" : "YES") << ", " << scanDataRate << ", " << () << std::endl;
	//std::cout << "scan rate : " << scanDataRate << std::endl;
}

void ActToMove_TB3::timerCallback(const ros::TimerEvent & ) 
{
	base_cmd.angular.z = 0.0;
	base_cmd.linear.x = 0.0;
	// ---------------------------
	// Update obstruction data :
	// (processing laser)
	computeObstruction();
	// ---------------------------
	if( enable )
	{	if( explo )
		{
            //std::cout << "timer/Explo" << std::endl; 
			// Code for exploring
			drive();
		}
		else
		{
            //std::cout << "timer/Action" << std::endl; 
			// Code for exploring
			// Code for driving when action received :
			float err = 0.0;
			switch(navStep)
			{
				case 0:
					// Orient to direction
					err = orientTowards(actionToAngle(receivedAction.actionID, nbOfDirections));
					// If orientation OK, next step (go forward) :
					if(fabs(err) < ANGULAR_ERROR)
					{
						navStep = 1;
						commandStart = ros::Time::now();
						std::cout << "Starting to go forward : " <<  commandStart << std::endl;
					}
				break;
				case 1:
					// Go forward
					drive();
					
					// IF GOAL REACHED, RELEASE TOKENS :
					if( (fabs(pos_dist) > actionDistance) )
					{
						std::cout << "Action ended (dist reached or timeout) : " << pos_dist << "/" << actionDistance << " - " << (ros::Time::now() - commandStart) << std::endl;
						pos_dist = 0.0;
						navActionToken = false;
						navStep = -1;
						commandTokenAvailable = true;
						std_msgs::Bool actionended;
						actionended.data = true;
						actionFinished_pub.publish(actionended);
					}
				break;
				default:
					// Wadafu happened:
				//	std::cout << "Inconsistent nav step, stoppping the robot !" << std::endl;
					navStep = -1;
					base_cmd.linear.x = 0.0;
					base_cmd.linear.y = 0.0;
					base_cmd.angular.z = 0.0;
				break;
			}
		}
		// Publish here to avoid conflicting commands when not enabled
		cmd_vel_pub_.publish(base_cmd);
	}
	else
	{
		// Don't do that to let another source control the robot :
//		base_cmd.linear.x = 0.0;
//		base_cmd.linear.y = 0.0;
//		base_cmd.angular.z = 0.0;
	}
    //std::cout << base_cmd << std::endl;
	
}

void ActToMove_TB3::controlCallback(std_msgs::Bool msg)
{
	std::cout << "Control callback: " << (msg.data ? "Yes": "No") << std::endl;
	enable = msg.data;
	if( enable ){ actionFinished_pub.publish(msg); }
    else // stop the robot
    {
		base_cmd.linear.x = 0.0;
		base_cmd.linear.y = 0.0;
		base_cmd.angular.z = 0.0;
		cmd_vel_pub_.publish(base_cmd);
	}
}

void ActToMove_TB3::actionCallback(habelar_msgs::Actions msg)
{
    std::cout << "action cb" << std::endl;
	ROS_ERROR("Action callback : %d", msg.actionID);
	// Save action received into internal memory
	receivedAction.actionID = msg.actionID;
	pos_start_x = receivedPose.pose.pose.position.x;
	pos_start_y = receivedPose.pose.pose.position.y;
	navStep = 0;

	if(commandTokenAvailable == true)
	{
		commandTokenAvailable = false;
		navActionToken = true;
		sum_angular_error = 0.0;
	}
	
}

//void ActToMove_TB3::poseCallback(geometry_msgs::PoseWithCovarianceStamped msg)
void ActToMove_TB3::poseCallback(nav_msgs::Odometry msg)
{
	//ROS_WARN("Got pose");
	receivedPose = msg;
	
	// Now, we should transform the pose from odom frame to map frame:
	std::string frameToTransform;
	try
	{
		listener.lookupTransform("/map", "/base_link", ros::Time(0), transform_base);
		receivedPose.pose.pose.position.x = transform_base.getOrigin().x();
		receivedPose.pose.pose.position.y = transform_base.getOrigin().y();
		receivedPose.pose.pose.position.z = transform_base.getOrigin().z();
		tf::Quaternion orient = transform_base.getRotation().normalize();
		receivedPose.pose.pose.orientation.x = orient.x();
		receivedPose.pose.pose.orientation.y = orient.y();
		receivedPose.pose.pose.orientation.z = orient.z();
		receivedPose.pose.pose.orientation.w = orient.w();
	}
	catch (tf::TransformException ex) 
	{
		ROS_ERROR("%s",ex.what());
	}
}

//void ActToMove_TB3::orientTowards(float goalAngle)
float ActToMove_TB3::orientTowards(float goalAngle)
{
	base_cmd.linear.x = 0.0;
	base_cmd.linear.y = 0.0;
	base_cmd.angular.z = 0.0;
	float robotAngle = 180.0 * tf::getYaw(receivedPose.pose.pose.orientation) / PI;

	// P(I)D controller
	angular_error = ( (fabs(goalAngle - robotAngle) > 180.0 ) ? -sign(goalAngle - robotAngle) * (360.0 - fabs(goalAngle) - fabs(robotAngle)) : goalAngle - robotAngle );
	// dirty hack to avoid spending time on very small modifications ?
	//if (!explo && (angular_error < 2*ANGULAR_ERROR) ) { return 0.0; } 

	//NOT USED :
	sum_angular_error += angular_error;
	delta_angular_error = angular_error - prev_angular_error;
	base_cmd.angular.z = Kp * angular_error + Kd * delta_angular_error; 
	
	prev_angular_error = angular_error;

	return angular_error;
}

// Smooth command law
float fcmd(float x, float dist, float alpha)
{
	//if( (x > dist) or (x < 0.0)){ ROS_ERROR("VAlue Error"); exit(EXIT_FAILURE); }
	if( (x > dist) or (x < 0.0)){ ROS_ERROR("OUT of range, returning 0.0"); return 0.0; }
	float y = x / dist;
	float z = 0.0;
	if ( y < alpha ){ z = y; }
	else
	{
		if( y > (1.0-alpha) ){ z = 1-y; }
		else{ z = alpha; }
	}
	return 0.5 * (1.0 + sin( PI / (2.0*alpha) * z));
}

// Obstacle aware speed command :
float fobs(float x, float slow, float stop)
{
	float y = (x - stop) / (slow - stop);
	if(x < stop){ y = 0.0; }
	else if(x > slow){ y = 1.0; }
	return sin( PI / 2.0 * y);
}

bool cmpXpos(geometry_msgs::Point32 i, geometry_msgs::Point32 j){ return i.x < j.x; }

void ActToMove_TB3::computeObstruction()
{
	// ---------------------------------------------
	// Process obstacle data :
	leftCluster = centCluster = rightCluster = laserscanToPCL;

	leftCluster.points.clear();
	centCluster.points.clear();
	rightCluster.points.clear();
	
	for(int i = 0 ; i < laserscanToPCL.points.size() ; i++)
	{
        // TB3 has a 360deg scan whereas TB2 only had 180 so we need 
        // to check for x to exclude points in the back of the robot
        if( laserscanToPCL.points[i].x > 0 )
        {
		    float y = laserscanToPCL.points[i].y;
		    // central corridor
		    if( fabs(y) < TURTLE_RADIUS )
		    {
		    	centCluster.points.push_back(laserscanToPCL.points[i]);
		    }
		    else
		    {
		    	if( y < 0.0){ rightCluster.points.push_back(laserscanToPCL.points[i]); }
		    	else{ leftCluster.points.push_back(laserscanToPCL.points[i]); }
		    }
        }
	}

	leftObsPercent = 0.0;
	centObsPercent = 0.0;
	rightObsPercent = 0.0; 

	for(int i = 0 ; i < leftCluster.points.size() ; i++){ if(leftCluster.points[i].x < DIST_OBSTACLE){ leftObsPercent += 1.0 / leftCluster.points.size(); } }
	for(int i = 0 ; i < rightCluster.points.size() ; i++){ if(rightCluster.points[i].x < DIST_OBSTACLE){rightObsPercent += 1.0 / rightCluster.points.size(); } }
	for(int i = 0 ; i < centCluster.points.size() ; i++){ if(centCluster.points[i].x < DIST_OBSTACLE){centObsPercent += 1.0 / centCluster.points.size(); } }

    // for visualisation of each cluster in rviz
    debug_leftCluster_pub.publish(leftCluster); 
    debug_centCluster_pub.publish(centCluster); 
    debug_rightCluster_pub.publish(rightCluster);
}


// VERSION TURTLEBOT COMPATIBLE :
//void ActToLove2::drive() lol typo
// This fonction is the behavior when in exploratio mode :
void ActToMove_TB3::drive()
{
	//std::cout << "Frame ID: " << pclData.header.frame_id << std::endl;
	
	// Set cmd to zero if in any case, our computations fail and we are in a non-expected situation :
	base_cmd.linear.x = 0.0;
	base_cmd.linear.y = 0.0;
	base_cmd.angular.z = 0.0;

	// Current robot pose (do we need it ?)
	float robotposeX = receivedPose.pose.pose.position.x;
	float robotposeY = receivedPose.pose.pose.position.y;
	float robotAngle = 180.0 * tf::getYaw(receivedPose.pose.pose.orientation) / PI;
	
	// Update distance done :
	pos_dist = sqrt(pow(robotposeX - pos_start_x, 2) + pow(robotposeY - pos_start_y,2) );		
	pos_dist_b = sqrt(pow(robotposeX - pos_start_b_x, 2) + pow(robotposeY - pos_start_b_y,2) );		

	//std::cout << "poses:  " << pos_dist << ", " << pos_dist_b << std::endl;
	// From Scarlett code :
	float linear_vel = 0.0;
	float angular_vel = 0.0;
	//pour l'equation sin(ax+b) atteigne le min et le max aux valeurs souahitees 
	float a= 2*90/(max_distance_slower - max_distance_stop);
	float b= 90 * (max_distance_stop + max_distance_slower)/(max_distance_stop - max_distance_slower);

	// ---------------------------------------------
	//std::cout << "Poses & Percents & Flags : " << pos_dist << ", " << pos_dist_b << ", " << leftObsPercent << ", " << centObsPercent << ", " << rightObsPercent << " | " << (backwards ? "back": "front") << ", " << (backLeft ? "BL": "/BL") << ", " <<(backRight ? "BR": "/BR") << ", " << (backStep ? "Step 1":"Step 2") << std::endl;

	// Process cases :
	if( backwards ) // Bumpers fired, going backward procedure :
	{
		// two steps procedure :
		float err = 0.0;
		switch(backStep)
		{
			case 0: // backwards
				linear_vel = -max_lin_vel;
				angular_vel = 0.0;
//				std::cout << "backdist : " << pos_dist_b << ", " << backwardDist << std::endl;
				//if( pos_dist_b > backwardDist ){ backStep = 1; }
				if(explo){ if( pos_dist_b > backwardDist ){ backStep = 1; } }
				else{ if( pos_dist_b > backwardDist ){ backwards = false; pos_dist = actionDistance+0.1; } } // If we don't explore, we consider bump to finish action (driven distance forced to max action distance)
			break;
			case 1: // reorient
				err = orientTowards(targetAngle);
				std::cout << "backstep 1 : " << err << ", " << targetAngle << ", " << robotAngle << ", " << base_cmd.angular.z << ", " << angular_vel << std::endl;
				// This is a hack because orientTowards directly modifies the message base_cmd but we use an intermediate variable angular vel
				angular_vel=base_cmd.angular.z; 
				if( fabs(err) < ANGULAR_ERROR ){ std::cout << "true" << std::endl; backStep = 0; backwards = false; }	
			break;
			default:
				ROS_ERROR("Wrong nav step !!!");
			break;
		}
	}
	else // Not bumpers (yet), relying on lasers :
	{
		// forwards :
		if( !noLaserData )
		{
			std::vector<geometry_msgs::Point32>::iterator lMinXPt = std::min_element(leftCluster.points.begin(), leftCluster.points.end(), cmpXpos);
			std::vector<geometry_msgs::Point32>::iterator cMinXPt = std::min_element(centCluster.points.begin(), centCluster.points.end(), cmpXpos);
			std::vector<geometry_msgs::Point32>::iterator rMinXPt = std::min_element(rightCluster.points.begin(), rightCluster.points.end(), cmpXpos);

			// Then compute speed command depending on obstacles
			float diff_close = rightObsPercent - leftObsPercent;
			filtre_diff_close = filtering_coef * filtre_diff_close + (1-filtering_coef) * diff_close;
			direction = (filtre_diff_close < 0) ? -1 : 1;
		
			//~~~~ case 1: very close ~~~~
			if( cMinXPt->x < max_distance_stop ) 	
			{
				// Stop and turn to less obstructed side
				linear_vel = 0.0; //STOP
				angular_vel = direction * max_ang_vel;
			}
			//~~~~ case 2 : Obstacle at midrange  ~~~~
			else if( cMinXPt->x <= max_distance_slower) 	
			{
				//ROS_WARN("B 2");
				// Progressively slowing and turning to most open direction
				linear_vel = max_lin_vel/2*(sin((a*cMinXPt->x+b)*M_PI/180) + 1) ;
				angular_vel = max_ang_vel/2*(-sin((a*cMinXPt->x+b)*M_PI/180) +1)* direction ;
			}
			//~~~~ case 3 : Go straight  ~~~~
			else
			{
				//ROS_WARN("B 4");
				// Explo, so we don't need any fonction :
				linear_vel = max_lin_vel; // * fcmd(pos_dist, actionDistance, 0.2);
				accuRandom += deltaAccu;
				if(explo)
				{
					if( accuRandom > THRESHOLD_HIGH )
					{
						deltaAccu = -1; 
						prev_ang_vel = angular_vel = (1 - 2 * (rand() % 2)) * max_ang_vel / 2.0;
					}
					else
					{
						if( accuRandom < THRESHOLD_LOW ){ deltaAccu = 1; prev_ang_vel = angular_vel = 0.0; }
						else{ angular_vel = prev_ang_vel; }
					}
//					std::cout << "Accu :! " << accuRandom << std::endl;
				}
				else{ angular_vel = 0.0; }
				filtre_diff_close = 0;	// raz coef filtré sinon il tourne toujours ds le meme sens 
			}
		}
		else
		{
			ROS_WARN("NO data from distance sensor yet. Turning but not going straight.");
			if(explo)
			{
				base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = linear_vel = 0.0;
				angular_vel = direction * max_ang_vel;
			}
			else
			{
				pos_dist = actionDistance+0.1;
			}
			// If we don't explore, we consider no laser data to finish action (driven distance forced to max action distance)
		//	std::cout << " dire: " << direction << std::endl;
		}
	}

	// Making speed saturate :
	if (linear_vel > max_lin_vel ){ linear_vel = max_lin_vel; }
	if (linear_vel < -max_lin_vel ){ linear_vel = -max_lin_vel; }
	if (angular_vel > max_ang_vel ){ angular_vel = max_ang_vel; }
	if (angular_vel < -max_ang_vel ){ angular_vel = -max_ang_vel; }
	
	base_cmd.linear.x =  linear_vel ;
	base_cmd.angular.z = angular_vel;
}

// ====================================

bool ActToMove_TB3::run()
{	
	while(nh.ok()){ ros::spinOnce(); }	

	return true;
}

// ====================================

int main(int argc, char** argv)
{
	//init the ROS node
	ros::init(argc, argv, "act2move_tb3");
	ros::NodeHandle nh;
	ros::NodeHandle node;
	// to get a random number in the future
	srand(time(NULL));

	int opt;
	// temp variables to store parameters, named according to options :
	float s=DEFAULT_MAXSPEED, w=DEFAULT_MAXSPEED, p=0.0, i=0.0 , d=0.0; // s parameter in constructor is useless ; it is used with setter 
	int a=DEFAULT_NBDIRECTIONS;
	bool e = false;
	float ad=ACTIONDISTANCE;

	while( (opt = getopt(argc, argv, "ewsapidhz") ) != -1)
	{
		switch(opt)
		{
			case 'z' : // set exploration behavior
				ad = atof(argv[optind]);
			break;
			case 'e' : // set exploration behavior
				e = true;
			break;
			case 'a' : // set action number
				a = atoi(argv[optind]);
			break;
			case 'w' : // rotative speed
				w = atof(argv[optind]);
			break;
			case 's' : // P coeff
				s = atof(argv[optind]);
			break;
			case 'p' : // P coeff
				p = atof(argv[optind]);
			break;
			case 'i' : // I coeff
				i = atof(argv[optind]);
			case 'd' : // D coeff
				d = atof(argv[optind]);
			break;
			case 'h' : // show Help option
				std::cout   << " Help : " << std::endl
					//		<< "Usage : rosrun habelar_msgs qlneural <-h (help) | options>" << std::endl
							<< "-s\t\t : Max linear speed of robot "  << std::endl
							<< "-d\t\t : Number of directions available "  << std::endl
							<< std::endl
							<< "-p\t\t : PID : Proportional coefficient "  << std::endl
							<< "-i\t\t : PID : Integral coefficient "  << std::endl
							<< "-d\t\t : PID : Derivative coefficient "  << std::endl
							<< std::endl
							<< "-h \t\t : print this text and exit" << std::endl;
							exit(EXIT_SUCCESS);
			break;
			default :
				std::cout   << "unknown option" << std::endl
							<< "This node expects some arguments, use " << std::endl
							<< "\t rosrun <package> <node> -h" << std::endl
							<< "to get help." << std::endl;
							exit(EXIT_FAILURE);
			break;
		}
	}

    std::cout << " Action dist ? : " << ad << std::endl;
    std::cout << " Exploration ? : " << e << std::endl;
    std::cout << " Nb of available directions : " << a << std::endl;
    std::cout << " Max A speed : " << w << std::endl;
    std::cout << " Max L speed : " << s << std::endl;
    std::cout << " K_p : " << p << std::endl;
    std::cout << " K_i : " << i << std::endl;
    std::cout << " K_d : " << d << std::endl;
	
//	Obstacle obst(node);
	//std::cout<<"111"<< std::endl;
	//ActToMove_TB3 tCtrl(nh,&obst, a, s, p, i, d);// while
	ActToMove_TB3 tCtrl(nh, a, p, i, d, e, ad);// while
	tCtrl.setMaxSpeedValues(s, w);
	tCtrl.run();
	
    return 0;
}
