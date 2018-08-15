#include <ros/ros.h>

#include <action_manager/DoAction.h>
#include <osrf_gear/Order.h>
#include <world_state/WorldStateClient.hpp>




class TestOrder {

	private:

		ros::NodeHandle node_;
		ros::AsyncSpinner spinner_;
		client::WorldStateClient world_client_;
		ros::ServiceClient action_srv_;
		ros::Subscriber order_sub_;


	public:

		TestOrder()
		:	node_(),
			spinner_(2),
			world_client_(),
			action_srv_(),
			order_sub_()
		{

			action_srv_ = node_.serviceClient<action_manager::DoAction>( "action_manager/do_action" );
			order_sub_ = node_.subscribe( "ariac/orders", 1, &TestOrder::cb_fillOrder, this );

			spinner_.start();
		}


		void cb_fillOrder( const osrf_gear::Order::ConstPtr & msg );

};



void TestOrder::cb_fillOrder( const osrf_gear::Order::ConstPtr & msg ){

	for(int i = 0; i < msg->shipments[0].products.size(); ++i ){
		
		std::string type = msg->shipments[0].products[i].type;
		std::string name;
		std::string container;
		action_manager::DoAction clear;
		action_manager::DoAction a_srv;
		

		//find part 
		if( world_client_.findPartOfType(type, name) ){
			ROS_INFO("Found part: %s", name.c_str());
			world_client_.markPartUsed(name);
		}
		else{
			ROS_ERROR("Could not find part of type: %s", type.c_str());
			return;
		}

		//find container
		if( world_client_.getPartContainer(name, container) ){
			ROS_INFO("Found part container: %s", container.c_str());
		}
		else{
			ROS_ERROR("Could not find container of part: %s", name.c_str());
			--i;
			continue;
		}



		//move to facebin
		a_srv = clear;
		a_srv.request.action = "move_to_bin";
		a_srv.request.bin_name = "FACE" + container; // CLIENT
		if( action_srv_.call(a_srv) ){
			if(a_srv.response.success){
				ROS_INFO("Move to bin succeeded");
			}
			else{
				ROS_ERROR("Fail: %s", a_srv.response.message.c_str());
				--i;
				continue;
			}
		}
		else{
			ROS_ERROR("Could not call DoAction.");
			return;
		}


		//move to bin
		a_srv = clear;
		a_srv.request.action = "move_to_bin";
		a_srv.request.bin_name = container; // CLIENT
		if( action_srv_.call(a_srv) ){
			if(a_srv.response.success){
				ROS_INFO("Move to bin succeeded");
			}
			else{
				ROS_ERROR("Fail: %s", a_srv.response.message.c_str());
				--i;
				continue;
			}
		}
		else{
			ROS_ERROR("Could not call DoAction.");
			return;
		}


		// approach part
		a_srv = clear;
		a_srv.request.action = "approach_part";
		a_srv.request.part_name = name;

		if( action_srv_.call(a_srv) ){
			if(a_srv.response.success){
				ROS_INFO("Approach part succeeded");
			}
			else{
				ROS_ERROR("Fail: %s", a_srv.response.message.c_str());
				--i;
				continue;
			}
		}
		else{
			ROS_ERROR("Could not call DoAction.");
			return;
		}


		// ROS_INFO("Press for next step...");
		// std::cin >> i;


		// grab part
		a_srv = clear;
		a_srv.request.action = "grab_part";
		a_srv.request.part_name = name;
		if( action_srv_.call(a_srv) ){
			if(a_srv.response.success){
				ROS_INFO("Grab part succeeded");
			}
			else{
				ROS_ERROR("Fail: %s", a_srv.response.message.c_str());
				--i;
				continue;
			}
		}
		else{
			ROS_ERROR("Could not call DoAction.");
			return;
		}

		// ROS_INFO("Press for next step...");
		// std::cin >> i;

		// move to bin
		a_srv = clear;
		a_srv.request.action = "move_to_bin";
		a_srv.request.bin_name = container; // CLIENT
		if( action_srv_.call(a_srv) ){
			if(a_srv.response.success){
				ROS_INFO("Move to bin succeeded");
			}
			else{
				ROS_ERROR("Fail: %s", a_srv.response.message.c_str());
				--i;
				continue;
			}
		}
		else{
			ROS_ERROR("Could not call DoAction.");
			return;
		}


		
		//move to facebin
		a_srv = clear;
		a_srv.request.action = "move_to_bin";
		a_srv.request.bin_name = "FACE" + container; // CLIENT
		if( action_srv_.call(a_srv) ){
			if(a_srv.response.success){
				ROS_INFO("Move to bin succeeded");
			}
			else{
				ROS_ERROR("Fail: %s", a_srv.response.message.c_str());
				--i;
				continue;
			}
		}
		else{
			ROS_ERROR("Could not call DoAction.");
			return;
		}


		//move to facebin by box
		a_srv = clear;
		a_srv.request.action = "face_box";
		a_srv.request.bin_name = container; 
		if( action_srv_.call(a_srv) ){
			if(a_srv.response.success){
				ROS_INFO("Move to face box succeeded");
			}
			else{
				ROS_ERROR("Fail: %s", a_srv.response.message.c_str());
				--i;
				continue;	
			}
		}
		else{
			ROS_ERROR("Could not call DoAction.");
			return;
		}



		// move to box
		a_srv = clear;
		a_srv.request.action = "move_to_box";
		a_srv.request.box_name = "BOX";
		if( action_srv_.call(a_srv) ){
			if(a_srv.response.success){
				ROS_INFO("Move to box succeeded");
			}
			else{
				ROS_ERROR("Fail: %s", a_srv.response.message.c_str());
				--i;
				continue;
			}
		}
		else{
			ROS_ERROR("Could not call DoAction.");
			return;
		}



		// ROS_INFO("Press for next step...");
		// std::cin >> i;


		// place part in goal pose
		geometry_msgs::Pose goal_pose;
		if( world_client_.computeGoalPose( msg->shipments[0].products[i].pose, "BOX0", goal_pose ) ){
			ROS_INFO("Found goal pose: %.4f, %.4f, %.4f", goal_pose.position.x, goal_pose.position.y, goal_pose.position.z);
		} 
		else{
			ROS_ERROR("Could not compute the goal pose");
			--i;
			continue;
		}
		a_srv = clear;
		a_srv.request.action = "place_part";
		a_srv.request.part_name = name;
		a_srv.request.goal_pose = goal_pose;
		if( action_srv_.call(a_srv) ){
			if(a_srv.response.success){
				ROS_INFO("Place part succeeded");
		
			}
			else{
				ROS_ERROR("Fail: %s", a_srv.response.message.c_str());
				--i;
				continue;
			}
		}
		else{
			ROS_ERROR("Could not call DoAction.");
			return;
		}



	}




	

}




int main( int argc, char* argv[] ){

	ros::init( argc, argv, "test_action_node" );
	TestOrder to{};

	
	ROS_INFO("Test Action Node is ready.");


	ros::waitForShutdown();
	return 0;


	



	// while(getchar()) {
		
	// 	ROS_INFO("Tasks \n1: approach_part \n2: move_to_bin \n3: grab_part \n4: move_to_box \n5: place_part");
	// 	int i;
	// 	std::string task_name;
	// 	std::cin>>i;


	// 	if( i == 1){
	// 		action_manager::DoAction a_srv;
	// 		a_srv.request.action = "approach_part";
	// 		a_srv.request.part_name = "gear_part_8";

	// 		if( action_srv_.call(a_srv) ){
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
	// 		if( action_srv_.call(a_srv) ){
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
	// 		if( action_srv_.call(a_srv) ){
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
	// 		if( action_srv_.call(a_srv) ){
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
	// 		if( action_srv_.call(a_srv) ){
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



}
