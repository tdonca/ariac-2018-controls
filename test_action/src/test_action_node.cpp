#include <ros/ros.h>

#include <action_manager/DoAction.h>


int main( int argc, char* argv[] ){

	ros::init( argc, argv, "test_action_node" );
	ros::NodeHandle node;
	ros::AsyncSpinner spinner{2};
	spinner.start();

	ROS_INFO("Test Action Node is ready.");



	//Test
	ros::ServiceClient action_srv = node.serviceClient<action_manager::DoAction>( "action_manager/do_action" );


	

	while(getchar()) {
		
		ROS_INFO("Tasks \n1: approach_part \n2: move_to_bin \n3: grab_part \n4: move_to_box \n5: place_part");
		int i;
		std::string task_name;
		std::cin>>i;


		if( i == 1){
			action_manager::DoAction a_srv;
			a_srv.request.action = "approach_part";
			a_srv.request.part_name = "gear_part_8";

			if( action_srv.call(a_srv) ){
				if(a_srv.response.success){
					ROS_INFO("Approach part succeeded");
				}
				else{
					ROS_ERROR("Fail: %s", a_srv.response.message.c_str());
				}
			}
			else{
				ROS_ERROR("Could not call DoAction.");
			}
		}

		else if (i == 2){
			action_manager::DoAction a_srv;
			a_srv.request.action = "move_to_bin";
			a_srv.request.bin_name = "BIN1";
			if( action_srv.call(a_srv) ){
				if(a_srv.response.success){
					ROS_INFO("Move to bin succeeded");
				}
				else{
					ROS_ERROR("Fail: %s", a_srv.response.message.c_str());
				}
			}
			else{
				ROS_ERROR("Could not call DoAction.");
			}
		}

		else if (i == 3){
			action_manager::DoAction a_srv;
			a_srv.request.action = "grab_part";
			a_srv.request.part_name = "gear_part_8";
			if( action_srv.call(a_srv) ){
				if(a_srv.response.success){
					ROS_INFO("Grab part succeeded");
				}
				else{
					ROS_ERROR("Fail: %s", a_srv.response.message.c_str());
				}
			}
			else{
				ROS_ERROR("Could not call DoAction.");
			}

		}

		else if (i == 4){
			action_manager::DoAction a_srv;
			a_srv.request.action = "move_to_box";
			a_srv.request.box_name = "BOX";
			if( action_srv.call(a_srv) ){
				if(a_srv.response.success){
					ROS_INFO("Move to box succeeded");
				}
				else{
					ROS_ERROR("Fail: %s", a_srv.response.message.c_str());
				}
			}
			else{
				ROS_ERROR("Could not call DoAction.");
			}

		}

		else if (i == 5){
			action_manager::DoAction a_srv;
			a_srv.request.action = "place_part";
			a_srv.request.part_name = "gear_part_8";
			// goal pose????
			if( action_srv.call(a_srv) ){
				if(a_srv.response.success){
					ROS_INFO("Place part succeeded");
				}
				else{
					ROS_ERROR("Fail: %s", a_srv.response.message.c_str());
				}
			}
			else{
				ROS_ERROR("Could not call DoAction.");
			}

		}

	}

	// 	if( i == 1){
	// 		j_srv.request.joint_names = jn;
	// 		j_srv.request.joint_values = box;
	// 		if( joints_srv.call(j_srv) ){
	// 			if( j_srv.response.success ){
	// 				ROS_INFO("Joints path executed successfully.");
	// 			}
	// 			else{
	// 				ROS_ERROR("Fail: %s", j_srv.response.message.c_str());
	// 			}
	// 		}
	// 		else{
	// 			ROS_ERROR("Could not call MoveJoints.");
	// 		}
	// 	}



	// 	else if( i == 2){
	// 		p_srv.request.goal_pose = box_p;
	// 		if( pose_srv.call(p_srv) ){
	// 			if( p_srv.response.success ){
	// 				ROS_INFO("Motion path executed successfully.");
	// 			}
	// 			else{
	// 				ROS_ERROR("Fail: %s", p_srv.response.message.c_str());
	// 			}
	// 		}
	// 		else{
	// 			ROS_ERROR("Could not call MovePose.");
	// 		}

	// 	}



	// 	else if( i == 3){
	// 		geometry_msgs::Pose cp = box_p;
	// 		cp.position.z += 0.2;
	// 		cp.position.x -= 0.1;
	// 		c_srv.request.waypoints = std::vector<geometry_msgs::Pose>{cp};
	// 		if( cartesian_srv.call(c_srv) ){
	// 			if( c_srv.response.success ){
	// 				ROS_INFO("Cartesian path executed successfully.");
	// 			}
	// 			else{
	// 				ROS_ERROR("Fail: %s", c_srv.response.message.c_str());
	// 			}
	// 		}
	// 		else{
	// 			ROS_ERROR("Could not call MoveCartesian.");
	// 		}
	// 	}



	// 	else if( i == 4){
	// 		g_srv.request.enable = true;
	// 		if( gripper_srv.call(g_srv) ){
	// 			if( g_srv.response.success ){
	// 				ROS_INFO("Gripper action executed successfully.");
	// 			}
	// 			else{
	// 				ROS_ERROR("Fail: %s", g_srv.response.message.c_str());
	// 			}
	// 		}
	// 		else{
	// 			ROS_ERROR("Could not call ActivateGripper.");
	// 		}
	// 	}

	// }

	





	// if( cartesian_srv.call(cart_srv) ){
	// 	if( cart_srv.response.success ){
	// 		ROS_INFO("Cartesian path executed successfully.");
	// 	}
	// 	else{
	// 		ROS_ERROR("Fail: %s", cart_srv.response.message.c_str());
	// 	}
	// }
	// else{
	// 	ROS_ERROR("Could not call ExecuteCartesian.");
	// }
	//END



	ros::waitForShutdown();
	return 0;
}
