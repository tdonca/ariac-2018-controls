#include <ros/ros.h>
#include <controller_server/ControllerServer.hpp>



int main( int argc, char* argv[] ){

	ros::init( argc, argv, "controller_server_node" );
	ros::NodeHandle node;
	ros::AsyncSpinner spinner{2};
	
	control::ControllerServer cs{};

	
	spinner.start();
	ros::waitForShutdown();
	return 0;
}
