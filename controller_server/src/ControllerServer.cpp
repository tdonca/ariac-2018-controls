#include <controller_server/ControllerServer.hpp>


namespace control {


	bool ControllerServer::sv_moveJoints( controller_server::MoveJoints::Request & req, controller_server::MoveJoints::Response & rsp ){

		rsp.success = moveJoints( req.joint_names, req.joint_values, rsp.message );
		return true;
	}


	bool ControllerServer::sv_movePose( controller_server::MovePose::Request & req, controller_server::MovePose::Response & rsp ){

		rsp.success = movePose( req.goal_pose, rsp.message );
		return true;
	}


	bool ControllerServer::sv_moveCartesian( controller_server::MoveCartesian::Request & req, controller_server::MoveCartesian::Response & rsp ){

		rsp.success = moveCartesian( req.waypoints, rsp.message );
		return true;
	}


	bool ControllerServer::sv_activateGripper( controller_server::ActivateGripper::Request & req, controller_server::ActivateGripper::Response & rsp ){

		rsp.success = activateGripper( req.enable, rsp.message );
		return true;

	}





	bool ControllerServer::moveJoints( std::vector<std::string> joint_names, std::vector<double> joint_values, std::string & error_message ){

		iiwa14_controller::ExecuteJoints j_srv;
		j_srv.request.joint_names = joint_names;
		j_srv.request.joint_values = joint_values;
		if( joints_srv_.call(j_srv) ){
			if( j_srv.response.success ){
				ROS_INFO("Joints path executed successfully.");
				return true;
			}
			else{
				ROS_ERROR("Fail: %s", j_srv.response.message.c_str());
				error_message = j_srv.response.message.c_str();
				return false;
			}
		}
		else{
			error_message = "Could not call ExecuteJoints.";
			ROS_ERROR("%s", error_message.c_str());
			return false;
		}
	}


	bool ControllerServer::movePose( geometry_msgs::Pose goal, std::string & error_message ){

		iiwa14_controller::ExecutePose p_srv;
		p_srv.request.goal_pose = goal;
		if( pose_srv_.call(p_srv) ){
			if( p_srv.response.success ){
				ROS_INFO("Pose goal executed successfully.");
				return true;
			}
			else{
				ROS_ERROR("Fail: %s", p_srv.response.message.c_str());
				error_message = p_srv.response.message.c_str();
				return false;
			}
		}
		else{
			error_message = "Could not call ExecutePose.";
			ROS_ERROR("%s", error_message.c_str());
			return false;
		}
	}


	bool ControllerServer::moveCartesian( std::vector<geometry_msgs::Pose> waypoints, std::string & error_message ){

		iiwa14_controller::ExecuteCartesian c_srv;
		c_srv.request.waypoints = waypoints;
		if( cartesian_srv_.call(c_srv) ){
			if( c_srv.response.success ){
				ROS_INFO("Cartesian goal executed successfully.");
				return true;
			}
			else{
				ROS_ERROR("Fail: %s", c_srv.response.message.c_str());
				error_message = c_srv.response.message.c_str();
				return false;
			}
		}
		else{
			error_message = "Could not call ExecuteCartesian.";
			ROS_ERROR("%s", error_message.c_str());
			return false;
		}
	}


	bool ControllerServer::activateGripper( bool enable, std::string & error_message ){

		iiwa14_controller::ExecuteGripper g_srv;
		g_srv.request.enable = enable;
		if( gripper_srv_.call(g_srv) ){
			if( g_srv.response.success ){
				ROS_INFO("Gripper action executed successfully.");
				return true;
			}
			else{
				ROS_ERROR("Fail: %s", g_srv.response.message.c_str());
				error_message = g_srv.response.message.c_str();
				return false;
			}
		}
		else{
			error_message = "Could not call ExecuteGripper.";
			ROS_ERROR("%s", error_message.c_str());
			return false;
		}
	}	


}