#include <ros/ros.h>

#include <action_manager/DoAction.h>
#include <osrf_gear/Order.h>


void cb_fillOrder( const osrf_gear::Order::ConstPtr & msg ){

	for(int i = 0; i < msg->shipments[0].products.size(); ++i ){
		
	}
}




int main( int argc, char* argv[] ){

	ros::init( argc, argv, "test_action_node" );
	ros::NodeHandle node;
	ros::AsyncSpinner spinner{2};
	spinner.start();

	ROS_INFO("Test Action Node is ready.");



	//Test
	ros::ServiceClient action_srv = node.serviceClient<action_manager::DoAction>( "action_manager/do_action" );
	ros::Subscriber new_orders = node.subscribe( "ariac/orders", 1, &cb_fillOrder );

	



	// while(getchar()) {
		
	// 	ROS_INFO("Tasks \n1: approach_part \n2: move_to_bin \n3: grab_part \n4: move_to_box \n5: place_part");
	// 	int i;
	// 	std::string task_name;
	// 	std::cin>>i;


	// 	if( i == 1){
	// 		action_manager::DoAction a_srv;
	// 		a_srv.request.action = "approach_part";
	// 		a_srv.request.part_name = "gear_part_8";

	// 		if( action_srv.call(a_srv) ){
	// 			if(a_srv.response.success){
	// 				ROS_INFO("Approach part succeeded");
	// 			}
	// 			else{
	// 				ROS_ERROR("Fail: %s", a_srv.response.message.c_str());
	// 			}
	// 		}
	// 		else{
	// 			ROS_ERROR("Could not call DoAction.");
	// 		}
	// 	}

	// 	else if (i == 2){
	// 		action_manager::DoAction a_srv;
	// 		a_srv.request.action = "move_to_bin";
	// 		a_srv.request.bin_name = "BIN1";
	// 		if( action_srv.call(a_srv) ){
	// 			if(a_srv.response.success){
	// 				ROS_INFO("Move to bin succeeded");
	// 			}
	// 			else{
	// 				ROS_ERROR("Fail: %s", a_srv.response.message.c_str());
	// 			}
	// 		}
	// 		else{
	// 			ROS_ERROR("Could not call DoAction.");
	// 		}
	// 	}

	// 	else if (i == 3){
	// 		action_manager::DoAction a_srv;
	// 		a_srv.request.action = "grab_part";
	// 		a_srv.request.part_name = "gear_part_8";
	// 		if( action_srv.call(a_srv) ){
	// 			if(a_srv.response.success){
	// 				ROS_INFO("Grab part succeeded");
	// 			}
	// 			else{
	// 				ROS_ERROR("Fail: %s", a_srv.response.message.c_str());
	// 			}
	// 		}
	// 		else{
	// 			ROS_ERROR("Could not call DoAction.");
	// 		}

	// 	}

	// 	else if (i == 4){
	// 		action_manager::DoAction a_srv;
	// 		a_srv.request.action = "move_to_box";
	// 		a_srv.request.box_name = "BOX";
	// 		if( action_srv.call(a_srv) ){
	// 			if(a_srv.response.success){
	// 				ROS_INFO("Move to box succeeded");
	// 			}
	// 			else{
	// 				ROS_ERROR("Fail: %s", a_srv.response.message.c_str());
	// 			}
	// 		}
	// 		else{
	// 			ROS_ERROR("Could not call DoAction.");
	// 		}

	// 	}

	// 	else if (i == 5){
	// 		action_manager::DoAction a_srv;
	// 		a_srv.request.action = "place_part";
	// 		a_srv.request.part_name = "gear_part_8";
	// 		// goal pose????
	// 		if( action_srv.call(a_srv) ){
	// 			if(a_srv.response.success){
	// 				ROS_INFO("Place part succeeded");
	// 			}
	// 			else{
	// 				ROS_ERROR("Fail: %s", a_srv.response.message.c_str());
	// 			}
	// 		}
	// 		else{
	// 			ROS_ERROR("Could not call DoAction.");
	// 		}

	// 	}

	// }



	ros::waitForShutdown();
	return 0;
}
