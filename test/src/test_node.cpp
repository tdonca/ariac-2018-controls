#include <ros/ros.h>
#include <iiwa14_controller/ExecuteCartesian.h>



int main( int argc, char* argv[] ){

	ros::init( argc, argv, "test_node" );
	ros::NodeHandle node;
	ros::AsyncSpinner spinner{2};
	spinner.start();

	ROS_INFO("Test Node is ready.");



	//Test
	ros::ServiceClient cartesian_srv = node.serviceClient<iiwa14_controller::ExecuteCartesian>( "iiwa14_controller/execute_cartesian" );
	iiwa14_controller::ExecuteCartesian cart_srv;



	if( cartesian_srv.call(cart_srv) ){
		if( cart_srv.response.success ){
			ROS_INFO("Cartesian path executed successfully.");
		}
		else{
			ROS_ERROR("Fail: %s", cart_srv.response.message.c_str());
		}
	}
	else{
		ROS_ERROR("Could not call ExecuteCartesian.");
	}
	//END



	ros::waitForShutdown();
	return 0;
}
