#include <iiwa14_controller/IIWA14Controller.hpp>



namespace control {
namespace iiwa14 {


	bool IIWA14Controller::sv_executeCartesian( iiwa14_controller::ExecuteCartesian::Request & req, iiwa14_controller::ExecuteCartesian::Response & rsp ){


		//rsp.success = executeCartesian( req.waypoints, rsp.message );
		return true;
	}


	bool IIWA14Controller::sv_executeJoints( iiwa14_controller::ExecuteJoints::Request & req, iiwa14_controller::ExecuteJoints::Response & rsp ){


		return true;
	}


	bool IIWA14Controller::sv_executeMotion( iiwa14_controller::ExecuteMotion::Request & req, iiwa14_controller::ExecuteMotion::Response & rsp ){


		return true;
	}


	bool IIWA14Controller::sv_executeGripper( iiwa14_controller::ExecuteGripper::Request & req, iiwa14_controller::ExecuteGripper::Response & rsp ){


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


	}


	bool IIWA14Controller::executeMotion( geometry_msgs::Pose goal, std::string & error_message ){


	}


	bool IIWA14Controller::executeGripper( bool enable, std::string & error_message ){


		
	}



}
}