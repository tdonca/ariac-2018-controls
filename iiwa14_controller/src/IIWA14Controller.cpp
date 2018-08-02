#include <iiwa14_controller/IIWA14Controller.hpp>



namespace control {
namespace iiwa14 {


	bool IIWA14Controller::sv_executeCartesian( iiwa14_controller::ExecuteCartesian::Request & req, iiwa14_controller::ExecuteCartesian::Response & rsp ){


		rsp.success = executeCartesian( req.waypoints, rsp.message );
		return true;
	}


	bool IIWA14Controller::sv_executeJoints( iiwa14_controller::ExecuteJoints::Request & req, iiwa14_controller::ExecuteJoints::Response & rsp ){

		if( req.joint_names.size() != req.joint_values.size() ){
			ROS_ERROR("Joint names and values do not match!");
			return false;
		} 

		rsp.success = executeJoints( req.joint_names, req.joint_values, rsp.message );

		ros::Duration(1.0).sleep();
		geometry_msgs::Pose box = move_group_.getCurrentPose().pose;
		ROS_INFO("%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", box.position.x, box.position.y, box.position.z, 
															box.orientation.x, box.orientation.y, box.orientation.z, box.orientation.w);
		return true;
	}


	bool IIWA14Controller::sv_executeMotion( iiwa14_controller::ExecuteMotion::Request & req, iiwa14_controller::ExecuteMotion::Response & rsp ){

		rsp.success = executeMotion( req.goal_pose, rsp.message );
		return true;
	}


	bool IIWA14Controller::sv_executeGripper( iiwa14_controller::ExecuteGripper::Request & req, iiwa14_controller::ExecuteGripper::Response & rsp ){

		rsp.success = executeGripper( req.enable, rsp.message );
		return true;
	}






	bool IIWA14Controller::executeCartesian( std::vector<geometry_msgs::Pose> waypoints, std::string & error_message ){

		// Compute the cartesian path
		int attempts = 0;
		bool found = false;
		moveit::planning_interface::MoveGroupInterface::Plan plan;
		while(attempts < 5) {
			double fraction = move_group_.computeCartesianPath( waypoints, 0.001, 0.0, plan.trajectory_ );
			if(fraction == 1) {
				ROS_INFO("Successfully computed the cartesian path.");
				found = true;
				break;
			}
			else{
				ROS_INFO("Only partial path computed: %.3f", fraction * 100);
			}
			attempts += 1;
		}

		// Abort if no path found
		if( !found ){
			error_message = "Could not compute cartesian path. Aborting execution...";
			ROS_ERROR("%s", error_message.c_str());
			return false;
		}

		// Execute the Cartesian Path
		if( move_group_.execute(plan) ){
			ROS_INFO("Successfully executed the cartesian path.");
			return true;
		}
		else{
			error_message = "Could not execute the full cartesian path. Aborting execution...";
			ROS_ERROR("%s", error_message.c_str());
			return false;
		}
	}


	bool IIWA14Controller::executeJoints( std::vector<std::string> jn, std::vector<double> jv, std::string & error_message ){

		// Define the joint motion goal
		std::map<std::string, double> jv_target;
		for( int i = 0; i < jn.size(); ++i ){
			jv_target[jn[i]] = jv[i];
		}
		move_group_.setJointValueTarget(jv_target);

		// Compute the joint value plan
		int attempts = 0;
		bool found = false;
		moveit::planning_interface::MoveGroupInterface::Plan plan;
		while( attempts < 5 ) {
			found = ( move_group_.plan(plan) == moveit_msgs::MoveItErrorCodes::SUCCESS);
			if(found) {
				ROS_INFO("Successfully planned the joint value motion.");
				found = true;
				break;
			}
			else{
				ROS_INFO("Could not plan the joint value motion.");
			}
			attempts += 1;
		}

		// Abort if not plan found
		if( !found ){
			error_message = "Could not compute joint value motion. Aborting execution...";
			ROS_ERROR("%s", error_message.c_str());
			return false;
		}

		// Execute the Joint Value Motion
		if( move_group_.execute(plan) ){
			ROS_INFO("Successfully executed the joint value motion.");
			return true;
		}
		else{
			error_message = "Could not execute the joint value motion. Aborting execution...";
			ROS_ERROR("%s", error_message.c_str());
			return false;
		}
	}


	bool IIWA14Controller::executeMotion( geometry_msgs::Pose goal, std::string & error_message ){

		// Define the pose goal
		move_group_.setPoseTarget(goal);

		// Compute the pose motion plan
		int attempts = 0;
		bool found = false;
		moveit::planning_interface::MoveGroupInterface::Plan plan;
		while( attempts < 5 ) {
			found = ( move_group_.plan(plan) == moveit_msgs::MoveItErrorCodes::SUCCESS);
			if(found) {
				ROS_INFO("Successfully planned the pose goal motion.");
				found = true;
				break;
			}
			else{
				ROS_INFO("Could not plan the pose goal motion.");
			}
			attempts += 1;
		}

		// Abort if not plan found
		if( !found ){
			error_message = "Could not compute pose goal motion. Aborting execution...";
			ROS_ERROR("%s", error_message.c_str());
			return false;
		}

		// Execute the pose goal motion
		if( move_group_.execute(plan) ){
			ROS_INFO("Successfully executed the pose goal motion.");
			return true;
		}
		else{
			error_message = "Could not execute the pose goal motion. Aborting execution...";
			ROS_ERROR("%s", error_message.c_str());
			return false;
		}

	}


	bool IIWA14Controller::executeGripper( bool enable, std::string & error_message ){

		osrf_gear::VacuumGripperControl g_srv;
		g_srv.request.enable = enable;
		if( vacuum_srv_.call(g_srv) ){
			if( g_srv.response.success ){
				ROS_INFO("Successfully toggled the gripper.");
				return true;
			}	
			else{
				ROS_ERROR("Gripper could not be toggled");
				return false;
			}
		}
		else{
			ROS_ERROR("Could not call VacummGripperControl service");
			return false;
		}	
	}



}
}