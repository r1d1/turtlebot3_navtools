#include "state_slam3.hpp"
#define PI 3.14159265359
using namespace std;

int APP=0;
int RECO=1;
bool LOAD=0;
bool LEARN=1;
//float RAYON_CELL=1.0;
float RAYON_CELL=0.75;
//float RAYON_CELL=2; previous value
//loat APPROX=0.4;
int NBRE_POINTS_POLY=50;	// attention si modif ici, il faut modif geometry_msgs::Point32 tab_point[50] de la classe
//float X_TILT_BASE=0.2;
//std::ofstream recon("reco_state_slam3");
std::ofstream crea; //("crea_state_slam3");
int IN=0;
int OUT=1;
int NEAR=2;
int time_inactivity=6;
int time_pub=3;

//int MESSAGE=1;	// 3 cell plus fortes 
int MESSAGE=0;	// 1 cell la plus forte

//============================================================================================

string transformation(int max)
{
	string chaine;
	stringstream ss1;
	ss1 << max;
	if (max<10){ chaine="0"+ss1.str(); }
	else{ chaine=ss1.str();	}
	return chaine;
}

//============================================================================================

state_slam3::state_slam3(ros::NodeHandle &nh_, float cell_rad=RAYON_CELL)
{
	nh=nh_;
	Marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_state", 100);

	first=0;
	poly_pub = nh.advertise<geometry_msgs::PolygonStamped>("polygon_publish", 1);
	clock=ros::Time::now();
	statereward_pub = nh.advertise<habelar_msgs::StateReward>("statereward", 1);
	state_pub = nh.advertise<habelar_msgs::State>("statealone", 1);
	reward_sub = nh.subscribe("rewardalone", 1, &state_slam3::reward_received, this);
	validActions_sub = nh.subscribe("valid_actions", 1, &state_slam3::validActionCallback, this);
	
	Scan_filter_sub = nh.subscribe("scan", 1, &state_slam3::base_received, this);
	actionControl_sub = nh.subscribe("action_finished", 1, &state_slam3::actionControlCallback, this);
	command_sub = nh.subscribe("commandStateSlam", 1, &state_slam3::controlCallback, this);

	poly_stamp.polygon.points.reserve(NBRE_POINTS_POLY);
	iterateur=0;
	//tab_point=[0];

	// Erwan addition :
	flagRewardReceived = false;
	flagActionFinished = false;
	activeCenter=0;
	Reward = 0.0;
	cellRadius=cell_rad;
	time_t t = time(0); 
	struct tm * now = localtime( &t );
	std::ostringstream datess;
	datess  << (now->tm_year+1900)
		<< (now->tm_mon+1) / 10 << (now->tm_mon+1) % 10
		<< now->tm_mday / 10  << now->tm_mday % 10
		<< "-"
		<< now->tm_hour / 10 << now->tm_hour % 10
		<< now->tm_min / 10 << now->tm_min % 10
		<< now->tm_sec / 10 << now->tm_sec % 10;

	poseOrientCellslog.open("poseCell_log"+datess.str()+".dat" , std::ios::out|std::ios::trunc);
	// end of addition
	previousTime_log = ros::Time::now();
	// ne pas charger le fichier :
	if (LOAD==1){ first_cell=1; }
	else{ first_cell=0; }
}
  
//--------------------------------------------------------------------------------------------  

state_slam3:: ~state_slam3()
{
	poseOrientCellslog.close();
	std::cout << "Closing pose log" << std::endl;
}

//============================================================================================

void  state_slam3::base_received( const sensor_msgs::LaserScan & msg) 
{
	base_laser=msg;
 	try
	{
		Listener.lookupTransform("/map","/base_link", ros::Time(0), transform_base);	
		current_position.x=transform_base.getOrigin().x();
		current_position.y=transform_base.getOrigin().y();
		current_angle=tf::getYaw(transform_base.getRotation()) ;
		
		computeDistToCenters();
		//for(int cell=0 ; cell < voronoiCenters.size() ; cell++){ std::cout << voronoiDist[cell] << ", " << std::flush; }
		//std::cout << std::endl;
		updateActiveCenter();
		float dist=distToActiveCenter();
//		float dist = computeMinDistToCenters();
		//std::cout << "Active center : " << activeCenter << " " << dist << " " << voronoiCenters.size() << " " << voronoiDist.size()<< std::endl;

		// Checking only for active center :
		 if( (dist > 2*cellRadius) || (voronoiCenters.size() == 0) )
		// Checking the min distance over all nodes :
//		if( (dist > 2*cellRadius) || (voronoiCenters.size() == 0) )
		{
			/* create new center */
			std::cout << "\033[034m" << "New center ! \033[0m" << std::endl;
			voronoiCenters.push_back(current_position);
			computeDistToCenters();
		}
		
		displayVoronoiCenters();

		if (abs(current_position.x-old_position.x)>0.05 || abs(current_position.y-old_position.y)>0.05)
		{
			mobile=true;
			old_position=current_position;
			clock=ros::Time::now();
			// en ne MAJ pas pas clock lorsque old_position=current_position , ainsi on commence a compter depuis cb1 de temps le robot n'a pas bougé
		}
		
		// checking flagRewardReceived : Erwan Addition to wait for reward before sending message (Science Fair hack)
		//if ( (ros::Time::now() > (clock+ros::Duration(time_inactivity))) && flagRewardReceived )
		// checking flagActionFinished : Erwan Addition to wait for action to finish before sending new state message (Nav experiment hack)
		if ( (ros::Time::now() > (clock+ros::Duration(time_inactivity))) && flagActionFinished )
		{
			clock=ros::Time::now();
			place_cell.reward=Reward;
			place_cell.stateID=std::to_string(activeCenter);
			place_cell.stateType="Nav2";
			//Reward=0;
			place_cell.validActions=validActions;
			
			place_cell_no_rwd.stateID=std::to_string(activeCenter);
			place_cell_no_rwd.stateType="Nav2";
			place_cell_no_rwd.validActions=validActions;

			std::cout << "\033[33m Inactivity - Publish StateReward info:\033[0m " << place_cell.stateID << ", " << place_cell.reward << std::endl;
			statereward_pub.publish(place_cell);
			state_pub.publish(place_cell_no_rwd);
			// Erwan addition to wait for reward to be received -- Science Fair hack
//			flagRewardReceived = false;
			flagActionFinished = false;
			// end of addition
			Reward=0;
			puts("PR2 - No motion detected");
			//puts("NE BOUGE PLUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUS");
		}
		
	}
	catch (tf::TransformException ex) 
	{
      		ROS_ERROR("%s",ex.what());
	}
}

//============================================================================================

void  state_slam3::reward_received( const std_msgs::Float32 & msg) 
{
	Reward=msg.data;
	std::cout<< "Got Reward !  reward="<<Reward<<", msg.reward="<< msg.data << std::endl;
	flagRewardReceived = true;
}

//validActions_sub = nh.subscribe("validactions", 1, &state_slam3::validActionCallback, this);
void state_slam3::validActionCallback( const habelar_msgs::ValidActions & msg)
{
//	std::cout << "Valid Actions : " << msg << std::endl;
	validActions = msg.actionStatus;
}

//--------------------------------------------------------------------------------------------
void state_slam3::actionControlCallback(const std_msgs::Bool & msg)
{
	std::cout << "Exec layer said action finished ! " << std::endl;
	flagActionFinished = msg.data;
}

void state_slam3::controlCallback(const habelar_msgs::CommandSignal & msg)
{
	if(msg.save)
	{
		std::cout << "\033[32m" << "Saving centers ...\033[0m" << std::endl;
		std::ofstream voronoiCenters_log("voronoiCenters_exp");
		for(int center=0 ; center < voronoiCenters.size() ; center++)
		{
			voronoiCenters_log << center << " " << voronoiCenters[center].x << " " << voronoiCenters[center].y << " " << voronoiCenters[center].z << std::endl;
		}
		std::cout << "\033[32m" << "done ...\033[0m" << std::endl;
	}
	
	if(msg.load && !msg.loadfile.empty())
	{
		std::cout << "\033[32m" << "Loading centers ...\033[0m" << std::endl;
		std::ifstream voronoiCenters_log(msg.loadfile);
		voronoiCenters.clear();
		int cid;
		float xv,yv,zv;
		while(voronoiCenters_log >> cid >> xv >> yv >> zv)
		{
			geometry_msgs::Point32 centerpoint;
			centerpoint.x = xv;
			centerpoint.y = yv;
			centerpoint.z = zv;
			voronoiCenters.push_back(centerpoint);
		}
	}
}

//============================================================================================
// Cell handling, Erwan style (2015)
//============================================================================================
void state_slam3::computeDistToCenters()
{
	//std::cout << "Voronoi distances : " << voronoiDist.size() << " " << voronoiCenters.size() << std::endl;
	if(voronoiDist.size() != voronoiCenters.size()){ voronoiDist.resize(voronoiCenters.size(), 0.0); }

	for(int center=0 ; center < voronoiCenters.size() ; center++)
	{
		voronoiDist[center] = std::sqrt(std::pow(voronoiCenters[center].x-current_position.x, 2)+std::pow(voronoiCenters[center].y-current_position.y, 2));
	}
}

//--------------------------------------------------------------------------------------------
float state_slam3::computeMinDistToCenters()
{
	// update distance info if needed :
	computeDistToCenters();
	std::vector<float>::iterator minusElem = std::min_element(voronoiDist.begin(), voronoiDist.end());
	return (*minusElem);
}
//--------------------------------------------------------------------------------------------
float state_slam3::distToActiveCenter()
{
	float dist = -1;
	if( voronoiDist.size() > 0)
	{
		voronoiDist[activeCenter] = std::sqrt(std::pow(voronoiCenters[activeCenter].x-current_position.x, 2)+std::pow(voronoiCenters[activeCenter].y-current_position.y, 2));
		dist=voronoiDist[activeCenter];
	}
	//std::cout << "dist : " << dist << std::endl;
	return dist;
}

//--------------------------------------------------------------------------------------------
// Cant use min_element with this version of cpp ...
//int argminVector(std::vector<float> vec){ for(std::vector<float>::iterator vecEl = vec.begin() ; vecEl != vec.end ; vecEl++){  } }

void state_slam3::updateActiveCenter()
{
	int previousCenter = activeCenter;
	activeCenter = std::distance(voronoiDist.begin(), std::min_element(voronoiDist.begin(), voronoiDist.end()));
	if (activeCenter != previousCenter)
	{
		std::cout << "Active center : " << activeCenter << " " << voronoiCenters.size() << " " << voronoiDist.size()<< std::endl;
		// Store extremum centers :
		std::vector<float> xValues, yValues;
		for(int center=0 ; center < voronoiCenters.size() ; center++){ xValues.push_back(voronoiCenters[center].x); yValues.push_back(voronoiCenters[center].y); }
		xMinCenter = std::distance(xValues.begin(), std::min_element(xValues.begin(), xValues.end()));
		xMaxCenter = std::distance(xValues.begin(), std::max_element(xValues.begin(), xValues.end()));
		yMinCenter = std::distance(yValues.begin(), std::min_element(yValues.begin(), yValues.end()));
		yMaxCenter = std::distance(yValues.begin(), std::max_element(yValues.begin(), yValues.end()));
		
		place_cell_no_rwd.stateID=std::to_string(activeCenter);
		place_cell_no_rwd.stateType="Nav2";
		place_cell_no_rwd.validActions=validActions;
		state_pub.publish(place_cell_no_rwd);
	}
	ros::Time nowTime = ros::Time::now();
	// Only log every 0.5s
	if( nowTime - previousTime_log > ros::Duration(0.5) )
	{
		poseOrientCellslog << ros::Time::now() << " " << current_position.x << " " << current_position.y << " " << current_angle << " " << activeCenter << " " << voronoiCenters.size()<< std::endl;
		//std::cout << nowTime << " " << current_position.x << " " << current_position.y << " " << 180.0*current_angle/PI << " " << activeCenter << " " << voronoiCenters.size()<< std::endl;
		previousTime_log = nowTime;
	}
	if( flagActionFinished )
	{
		clock=ros::Time::now();
		place_cell.reward=Reward;
		place_cell.stateID=std::to_string(activeCenter);
		place_cell.stateType="Nav2";
		place_cell.validActions=validActions;
		
		
		Reward=0;
		std::cout << "\033[33m Update Active Center - Publish StateReward info:\033[0m " << place_cell.stateID << ", " << place_cell.reward << std::endl;
		statereward_pub.publish(place_cell);
		flagActionFinished = false;
	}
}


//============================================================================================
// Cell handling, Scarlett style (2014)
//============================================================================================

void state_slam3::create_cell()
{
	geometry_msgs::Point32 point_poly;
	float angle_base_increment= base_laser.angle_increment;
	float Size_base_laser = base_laser.ranges.size();
	float theta_base;
	float x_base;			
	float y_base; 
	float distance;
	vector<geometry_msgs::Point32> tab_point(NBRE_POINTS_POLY);
	geometry_msgs::Point32 centre;
	int pas=Size_base_laser/NBRE_POINTS_POLY;
	centre=current_position;	//!!!
	for(int k=0; k < NBRE_POINTS_POLY ; k++)	// tout les n indice du laser on recupere sa distance vis a vis du centre pour en faire un polygone
	{	
		theta_base=(Size_base_laser/2-k*pas)*angle_base_increment-current_angle;
		x_base = (base_laser.ranges[k*pas])*sin(theta_base) ;			
		y_base = (base_laser.ranges[k*pas])*cos(theta_base); 	
		point_poly.y=-(x_base)+centre.y;
		point_poly.x=(y_base)+centre.x;
		distance=sqrt(std::pow(centre.x-point_poly.x,2)+std::pow(centre.y-point_poly.y,2));
		
		if (distance > cellRadius)
		{
			x_base = cellRadius*sin(theta_base) ;
			y_base = cellRadius*cos(theta_base) ;
			point_poly.y=-(x_base)+centre.y;
			point_poly.x=(y_base)+centre.x;	
		}
		geometry_msgs::Polygon polygone;
		tab_point[k]=point_poly;
	}
	vector_cell_poly.push_back(tab_point);
	vector_centre_poly.push_back(current_position);
}

//--------------------------------------------------------------------------------------------

int state_slam3::IsIn(geometry_msgs::Point32 position, vector<geometry_msgs::Point32> tableau)
{
	float max_x=0;
	float max_y=0;
	float min_x=500;
	float min_y=500;
	int c=0;
	int infk;
	int supk;
	float distance_mini=500;
	
	for ( int k=0; k<NBRE_POINTS_POLY ; k++)	// pour chaque arête 
	{
		supk=k+1;
		infk=k;
		if (k==NBRE_POINTS_POLY-1)
		{	
			supk=0;
			infk=k;
		}
	
		if (position.y<=tableau[infk].y && position.y>=tableau[supk].y || position.y>=tableau[infk].y && position.y<=tableau[supk].y)
		{
		
			float a = (tableau[infk].y -tableau[supk].y )/ (tableau[infk].x -tableau[supk].x );
			float b= tableau[infk].y-a*tableau[infk].x;
			float ydroite_xpos= a*position.x+b;
			float xdroite_ypose= (position.y-b)/a;
			if (abs(position.x-xdroite_ypose)< distance_mini) distance_mini=abs(position.x-xdroite_ypose);
			if( position.x>xdroite_ypose){ c++; }	// si pt au niveau de l'ordonnée de l'arête fixee et à gauche de l'arête 
		}
	}
	dist_cell_mini_IsIn=distance_mini;
	
	if( c%2 == 0 ) 
	{
		//WhoIsCloser(distance_mini);
		if( distance_mini > float(cellRadius/500.0) ){ return OUT; }
		else { return NEAR; }
	}
	else { return IN; }
}

//--------------------------------------------------------------------------------------------

void state_slam3::WhoIsCloser(int num)
{
	float dist_centre=sqrt(std::pow(vector_centre_poly[num].x-current_position.x,2)+std::pow(vector_centre_poly[num].y-current_position.y,2));
	if( dist_centre<min1 ) 
	{
		//puts("1111");
		min3=min2;
		indice_min3=indice_min2;
		min2=min1;
		indice_min2=indice_min1;
		min1=dist_centre;
		indice_min1=num;
	}
	if( dist_centre<min2 && num!=indice_min1 ) 
	{
		//puts("222");
		min3=min2;
		indice_min3=indice_min2;
		min2=dist_centre;
		indice_min2=num;
	}
	if( dist_centre<min3 && num!=indice_min1 && num!=indice_min2) 
	{
	//	puts("333");
		min3=dist_centre;
		indice_min3=num;
	}
}

//--------------------------------------------------------------------------------------------
// le plus proche parmis les poly auquels le point n'appartient pas 
void state_slam3::WhoIsCloserOut(int num)	
{
	float dist_centre=sqrt(std::pow(vector_centre_poly[num].x-current_position.x,2)+std::pow(vector_centre_poly[num].y-current_position.y,2));
	if (dist_centre<min1 && it < 1 ) 
	{
		puts("1111");
		min3=min2;
		indice_min3=indice_min2;
		min2=min1;
		indice_min2=indice_min1;
		min1=dist_centre;
		indice_min1=num;
	}
	if(dist_centre<min2 && it < 2 && num!=indice_min1 ) 
	{
		puts("222");
		min3=min2;
		indice_min3=indice_min2;
		min2=dist_centre;
		indice_min2=num;
	}
	if(dist_centre<min3 && it < 3 && num!=indice_min1 && num!=indice_min2) 
	{
		puts("333");
		min3=dist_centre;
		indice_min3=num;
	}
}

//============================================================================================
// Displaying data :
//============================================================================================
void state_slam3::displayVoronoiCenters()
{
	int32_t shape = visualization_msgs::Marker::CYLINDER;	
	//int32_t shape = visualization_msgs::Marker::LINE_STRIP;	
	marker.markers.resize(voronoiCenters.size());
	for (int i = 0; i < voronoiCenters.size(); i++)	//for each center, add a marker 
	{
		marker.markers[i].header.frame_id = "/map";
		marker.markers[i].header.stamp = ros::Time();
		marker.markers[i].ns = "basic_shapes";
		marker.markers[i].id = i;
		marker.markers[i].type = shape;
		marker.markers[i].action = visualization_msgs::Marker::ADD;

		marker.markers[i].pose.position.x = voronoiCenters[i].x;
		marker.markers[i].pose.position.y= voronoiCenters[i].y;
		marker.markers[i].pose.position.z = voronoiCenters[i].z;

		marker.markers[i].scale.x = 0.1;
		marker.markers[i].scale.y = 0.1;
		marker.markers[i].scale.z = 0.01 ;

		//if( (i == xMinCenter) || (i == xMaxCenter) || (i == yMinCenter) || (i == yMaxCenter) ) 
		marker.markers[i].color.r =0;
		marker.markers[i].color.g =0;
		marker.markers[i].color.b =255;
		if( (i == activeCenter) ) 
		{
			marker.markers[i].color.r =255;
		}
		marker.markers[i].color.a = 1.0;
	}
	Marker_pub.publish(marker);
}

//--------------------------------------------------------------------------------------
void state_slam3::displayPoly()
{
	int32_t shape = visualization_msgs::Marker::CYLINDER;	
	//int32_t shape = visualization_msgs::Marker::LINE_STRIP;	
	marker.markers.resize(NBRE_POINTS_POLY*vector_cell_poly.size());
	for (int j = 0; j<  vector_cell_poly.size(); j++)	// pour chaque polygone enregistré
	{
		for (int i = 0; i<NBRE_POINTS_POLY; i++)
		{
			marker.markers[i].header.frame_id = "/map";
			marker.markers[i].header.stamp = ros::Time();
			marker.markers[i].ns = "basic_shapes";
			marker.markers[i].id = (j+1)*i;
			marker.markers[i].type = shape;
			marker.markers[i].action = visualization_msgs::Marker::ADD;

			marker.markers[i].pose.position.x = vector_cell_poly[j][i].x;

			//std::cout<<"cos phi"<<cos(phi)<<std::endl;
			marker.markers[i].pose.position.y= vector_cell_poly[j][i].y;
			marker.markers[i].pose.position.z = 0;

			// Set the scale of the marker -- 1x1x1 here means 1m on a side
			marker.markers[i].scale.x = 0.05;
			marker.markers[i].scale.y = 0.05;
			marker.markers[i].scale.z = 0.01 ;
		
			// Set the color -- be sure to set alpha to something non-zero!
			float colorA[3] = {0, 0, 255} ;
			float colorB[3] = {255, 0, 0} ;
			float color[3];
		
			for (int k=0; k<3; k++){ color[k] = colorA[k] + i/24 * (colorB[k] - colorA[k]); }
		
			marker.markers[i].color.r =0;
			marker.markers[i].color.g =66;
			marker.markers[i].color.b =11;
			marker.markers[i].color.a = 1.0;
		}
	}
	// This should go outside loops as we need to publish it only when it's full
	Marker_pub.publish(marker);
}

//============================================================================================
 
int main(int argc, char** argv)
{
	ros::init(argc, argv, "state_slam3");
	ros::NodeHandle nh;
	int opt;
	float rad=RAYON_CELL;
	
	const struct option availableOptions[] = {
		{"cellradius", no_argument, 0, 'c'},
		{0,0,0,0}
	};
	
	while( (opt = getopt_long(argc, argv, "c:", availableOptions, NULL) ) != -1)
	{
		switch(opt)
		{
			case 'c' : // set Tau (Temperature of softmax)
				rad = atof(argv[optind]);
				std::cout << " Cell Radius : " << rad << std::endl;
			break;
			default:
				std::cout   << "unknown option" << std::endl;
			break;
		}
	}

	state_slam3 state_slam3(nh, rad);
        while (nh.ok()){ ros::spin(); }
	return 0;
}
