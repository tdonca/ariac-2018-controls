#include <ros/ros.h>
#include <iiwa14_controller/IIWA14Controller.hpp>



int main( int argc, char* argv[] ){

	ros::init( argc, argv, "iiwa14_controller_node" );
	ros::NodeHandle node;
	ros::AsyncSpinner spinner{2};
	
	control::iiwa14::IIWA14Controller ic{};

	
	spinner.start();
	ros::waitForShutdown();
	return 0;
}
