

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <world_state/WorldState.hpp>

using namespace world;



int main( int argc, char* argv[] ){
	
	ros::init(argc, argv, "test_node");
	ros::NodeHandle node;
	ros::AsyncSpinner spinner(4);
	spinner.start();
	
	
	WorldState ws;
	
	ros::waitForShutdown();
	return 0;
}


