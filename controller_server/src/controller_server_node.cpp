#include <ros/ros.h>
#include <controller_server/ControllerServer.hpp>



int main( int argc, char* argv[] ){

	ros::init( argc, argv, "controller_server_node" );
	
	control::ControllerServer cs{};

	ros::waitForShutdown();
	return 0;
}
