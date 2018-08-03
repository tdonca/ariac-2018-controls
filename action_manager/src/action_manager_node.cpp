#include <ros/ros.h>
#include <action_manager/ActionManager.hpp>



int main( int argc, char* argv[] ){

	ros::init( argc, argv, "action_manager_node" );
	
	control::ActionManager am{};

	ros::waitForShutdown();
	return 0;
}
