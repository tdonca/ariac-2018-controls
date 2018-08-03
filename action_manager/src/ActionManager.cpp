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



		return true;
	}


	bool ActionManager::grabPart( std::string part_name, std::string & error_message ){




		return true;
	}


	bool ActionManager::placePart( geometry_msgs::Pose goal_pose, std::string & error_message ){



		return true;
	}


	bool ActionManager::moveToBin( std::string bin_name, std::string & error_message ){




		return true;
	}


	bool ActionManager::moveToBox( std::string box_name, std::string & error_message ){




		return true;
	}

}