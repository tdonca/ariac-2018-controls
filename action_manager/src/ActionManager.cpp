#include <action_manager/ActionManager.hpp>


namespace control {



	bool ActionManager::sv_processAction( action_manager::DoAction::Request & req, action_manager::DoAction::Response & rsp ){

		if( req.action == "approach_part" ){

			rsp.success = approachPart( req.part_name, rsp.message );

		}
		else if( req.action == "grab_part" ) {

			rsp.success = grabPart( req.part_name, rsp.message );

		}
		else if( req.action == "place_part" ) {

			rsp.success = placePart( req.goal_pose, rsp.message );

		}
		else if( req.action == "move_to_bin" ) {

			rsp.success = moveToBin( req.bin_name, rsp.message );

		}
		else if( req.action == "move_to_box" ) {

			rsp.success = moveToBox( req.box_name, rsp.message );

		}
		else {

			rsp.message = "Invalid action name provided.";
			ROS_ERROR("%s", rsp.message.c_str());
			rsp.success = false;
		}


		return true;
	}




	bool ActionManager::approachPart( std::string part_name, std::string & error_message ){

		// Get current part's above-pose
		geometry_msgs::Pose pose;
		if( world_state_.getPartAbovePose( part_name, pose ) ){
			ROS_INFO("%s found at pose: %.3f %.3f %.3f", part_name.c_str(), pose.position.x, pose.position.y, pose.position.z);
		}	
		else{
			error_message = "Could not get pose of: " + part_name;
			return false;
		}


		// Move robot to above the part
		controller_server::MovePose p_srv;
		p_srv.request.goal_pose = pose;
		if( pose_srv_.call(p_srv) ){
			if( p_srv.response.success ){
				ROS_INFO("Robot successfully moved to part pose.");
			}
			else{
				ROS_ERROR("Fail: %s", p_srv.response.message.c_str());
				return false;
			}
		}
		else{
			ROS_ERROR("Could not call MovePose srv");
			return false;
		}

		return true;
	}


	bool ActionManager::grabPart( std::string part_name, std::string & error_message ){

		// Get current part's grab-pose
		geometry_msgs::Pose pose;
		if( world_state_.getPartGrabPose( part_name, pose ) ){
			ROS_INFO("%s found at pose: %.3f %.3f %.3f", part_name.c_str(), pose.position.x, pose.position.y, pose.position.z);
		}	
		else{
			error_message = "Could not get pose of: " + part_name;
			return false;
		}


		// Make sure the gripper is off
		controller_server::ActivateGripper g_srv;
		g_srv.request.enable = false;
		if( gripper_srv_.call(g_srv) ){
			if( g_srv.response.success ){
				ROS_INFO("Gripper deactivated.");
			}
			else{
				ROS_ERROR("Fail: %s", g_srv.response.message.c_str());
				return false;
			}
		}
		else{
			ROS_ERROR("Could not call ActivateGripper srv");
			return false;
		}



		// Move robot to the part
		controller_server::MoveCartesian c_srv;
		c_srv.request.waypoints.push_back(pose);
		if( cartesian_srv_.call(c_srv) ){
			if( c_srv.response.success ){
				ROS_INFO("Robot successfully moved to part pose.");
			}
			else{
				ROS_ERROR("Fail: %s", c_srv.response.message.c_str());
				return false;
			}
		}
		else{
			ROS_ERROR("Could not call MoveCartesian srv");
			return false;
		}


		// Grab the part with the gripper
		g_srv.request.enable = true;
		if( gripper_srv_.call(g_srv) ){
			if( g_srv.response.success ){
				ROS_INFO("Robot successfully grabbed the part.");
			}
			else{
				ROS_ERROR("Fail: %s", g_srv.response.message.c_str());
				return false;
			}
		}
		else{
			ROS_ERROR("Could not call ActivateGripper srv");
			return false;
		}


		// Lift part up
		pose.position.z += 0.05;
		c_srv.request.waypoints[0] = pose;
		if( cartesian_srv_.call(c_srv) ){
			if( c_srv.response.success ){
				ROS_INFO("Robot successfully moved to above pose.");
			}
			else{
				ROS_ERROR("Fail: %s", c_srv.response.message.c_str());
				return false;
			}
		}
		else{
			ROS_ERROR("Could not call MoveCartesian srv");
			return false;
		}

		return true;

	}


	bool ActionManager::placePart( geometry_msgs::Pose goal_pose, std::string & error_message ){

		// // Move part to the goal pose
		// controller_server::MoveCartesian c_srv;
		// c_srv.request.waypoints.push_back(pose);
		// if( cartesian_srv_.call(c_srv) ){
		// 	if( c_srv.response.success ){
		// 		ROS_INFO("Robot successfully moved to goal pose.");
		// 	}
		// 	else{
		// 		ROS_ERROR("Fail: %s", c_srv.response.message.c_str());
		// 		return false;
		// 	}
		// }
		// else{
		// 	ROS_ERROR("Could not call MoveCartesian srv");
		// 	return false;
		// }


		// Deactivate gripper
		controller_server::ActivateGripper g_srv;
		g_srv.request.enable = false;
		if( gripper_srv_.call(g_srv) ){
			if( g_srv.response.success ){
				ROS_INFO("Robot successfully placed the part.");
			}
			else{
				ROS_ERROR("Fail: %s", g_srv.response.message.c_str());
				return false;
			}
		}
		else{
			ROS_ERROR("Could not call ActivateGripper srv");
			return false;
		}

		return true;
	}


	bool ActionManager::moveToBin( std::string bin_name, std::string & error_message ){

		// Get joint values for the bin
		std::vector<std::string> jn;
		std::vector<double> jv;
		if( world_state_.getBinLocation( bin_name, jn, jv ) ){
			ROS_INFO("%s location found.", bin_name.c_str());
		}
		else{
			ROS_ERROR("Could not find the location of bin: %s", bin_name.c_str());
			return false;
		}

		// Move robot to the bin
		controller_server::MoveJoints j_srv;
		j_srv.request.joint_names = jn;
		j_srv.request.joint_values = jv;
		if( joints_srv_.call(j_srv) ){
			if( j_srv.response.success ){
				ROS_INFO("Robot successfully moved to bin: %s", bin_name.c_str());
			}
			else{
				ROS_ERROR("Fail: %s", j_srv.response.message.c_str());
				return false;
			}
		}
		else{
			ROS_ERROR("Could not call MoveJoints srv");
			return false;
		}

		return true;
	}


	bool ActionManager::moveToBox( std::string box_name, std::string & error_message ){

		// Get joint values for the box
		std::vector<std::string> jn;
		std::vector<double> jv;
		if( world_state_.getBoxLocation( box_name, jn, jv ) ){
			ROS_INFO("%s location found.", box_name.c_str());
		}
		else{
			ROS_ERROR("Could not find the location of box: %s", box_name.c_str());
			return false;
		}

		// Move robot to the box
		controller_server::MoveJoints j_srv;
		j_srv.request.joint_names = jn;
		j_srv.request.joint_values = jv;
		if( joints_srv_.call(j_srv) ){
			if( j_srv.response.success ){
				ROS_INFO("Robot successfully moved to box: %s", box_name.c_str());
			}
			else{
				ROS_ERROR("Fail: %s", j_srv.response.message.c_str());
				return false;
			}
		}
		else{
			ROS_ERROR("Could not call MoveJoints srv");
			return false;
		}


		return true;
	}

}