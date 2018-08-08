
#include <world_state/WorldStateClient.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace client {
	

	void getRPY( const geometry_msgs::Pose  pose, double & r, double & p, double & y ){
	
	tf2::Quaternion arm_q;
	tf2::fromMsg( pose.orientation, arm_q );
	tf2::Matrix3x3(arm_q).getRPY( r, p, y );
	

	}

	void setRPY( const double r, const double p, const double y, geometry_msgs::Pose & pose ){
		
		tf2::Quaternion q;
		q.setRPY( r, p, y );
		pose.orientation = tf2::toMsg( q );
	}


	void WorldStateClient::test(){
		
		movePartToBox("gear_part_5");
		ros::Duration(5.0).sleep();
		
		std::vector<PlannerPart> parts;
		getBoxParts(parts);
		
		ROS_INFO("Parts in box:");
		for(int i = 0; i < parts.size(); ++i ){
			ROS_INFO("%s", parts[i].name.c_str());
		}
		
		
		
		ROS_INFO("Remove part");
		removePart("gear_part_5");
		getBoxParts(parts);
		
		
		ROS_INFO("Parts in box:");
		for(int i = 0; i < parts.size(); ++i ){
			ROS_INFO("%s", parts[i].name.c_str());
		}
	}
	
	bool WorldStateClient::getPartType( std::string type, PlannerPart & part_found ){
		
		// call find part type service  
		world_state::FindPartType fpt_srv;
		fpt_srv.request.type = type;
		
		if( m_find_part_type_srv.call(fpt_srv) ){
			if( fpt_srv.response.success ){
				//ROS_INFO("Message: %s", fpt_srv.response.message.c_str());
				part_found.name = fpt_srv.response.part.name;
				part_found.type = fpt_srv.response.part.type;
				part_found.id = fpt_srv.response.part.id;
				part_found.current_pose = fpt_srv.response.part.current_pose;
				return true;
			}
			else{
				ROS_ERROR("Message: %s", fpt_srv.response.message.c_str());
				return false;
			}
		}
		else{
			ROS_ERROR("Error calling FindPartType service.");
			return false;
		}
	}
	
	
	
	// ******* FREE ALL PARTS BEFORE ABORTING OR PREEMPTING A SHIPMENT *********
	bool WorldStateClient::markPartUsed( PlannerPart const & part ){
		
		world_state::MarkPartUsed mp_srv;
		mp_srv.request.name = part.name;
		
		if( m_mark_part_used_srv.call(mp_srv) ){
			if( mp_srv.response.success ){
				ROS_INFO("Successfully marked used %s", part.name.c_str());
				return true;
			}
			else{
				ROS_ERROR("Message: %s", mp_srv.response.message.c_str());
				return false;
			}
		}
		else{
			ROS_ERROR("Error calling MarkPartUsed service.");
			return false;
		}	
	}
	
	
	
	// ******* FREE ALL PARTS BEFORE ABORTING OR PREEMPTING A SHIPMENT *********
	bool WorldStateClient::releasePart( PlannerPart const & part ){
		
		world_state::ReleasePart rp_srv;
		rp_srv.request.name = part.name;
		
		if( m_release_part_srv.call(rp_srv) ){
			if( rp_srv.response.success ){
				ROS_INFO("Successfully released %s", part.name.c_str());
				return true;
			}
			else{
				ROS_ERROR("Message: %s", rp_srv.response.message.c_str());
				return false;
			}
		}
		else{
			ROS_ERROR("Error calling ReleasePart service.");
			return false;
		}		
	}
	
	
	bool WorldStateClient::getGripperPart( PlannerPart & part_found ){
		
		world_state::GetGripperPart gp_srv;
		
		if( m_gripper_part_srv.call(gp_srv) ){
			if( gp_srv.response.success ){
				ROS_INFO("Success: %s", gp_srv.response.message.c_str());
				part_found.name = gp_srv.response.part.name;
				part_found.type = gp_srv.response.part.type;
				part_found.id = gp_srv.response.part.id;
				part_found.current_pose = gp_srv.response.part.current_pose;
				return true;
			}
			else{
				ROS_ERROR("Message: %s", gp_srv.response.message.c_str());
				return false;
			}
		}
		else{
			ROS_ERROR("Error calling GetGripperPart service.");
			return false;
		}		
	}
	
	
			
	bool WorldStateClient::getBoxParts( std::vector<PlannerPart> & parts_found ){
		
		world_state::GetBoxParts bp_srv;
		
		if( m_box_parts_srv.call(bp_srv) ){
			if( bp_srv.response.success ){
				ROS_INFO("Success: %s", bp_srv.response.message.c_str());
				parts_found.clear();
				parts_found.resize( bp_srv.response.parts.size() );
				for( int i = 0; i < parts_found.size(); ++i ){
					parts_found[i].name = bp_srv.response.parts[i].name;
					parts_found[i].type = bp_srv.response.parts[i].type;
					parts_found[i].id = bp_srv.response.parts[i].id;
					parts_found[i].current_pose = bp_srv.response.parts[i].current_pose;
				}
				return true;
			}
			else{
				ROS_ERROR("Message: %s", bp_srv.response.message.c_str());
				return false;
			}
		}
		else{
			ROS_ERROR("Error calling GetBoxParts service.");
			return false;
		}	
	}
	
	
	bool WorldStateClient::movePartToBox( std::string name ){
		
		world_state::MovePartToBox ptb_srv;
		ptb_srv.request.name = name;
		
		if( m_move_part_to_box_srv.call(ptb_srv) ){
			if( ptb_srv.response.success ){
				ROS_INFO("Success: %s", ptb_srv.response.message.c_str());
				return true;
			}
			else{
				ROS_ERROR("Message: %s", ptb_srv.response.message.c_str());
				return false;
			}
		}
		else{
			ROS_ERROR("Error calling MovePartToBox service.");
			return false;
		}
	}
			
			
			
			
	bool WorldStateClient::removePart( std::string name ){
		
		world_state::RemovePart rp_srv;
		rp_srv.request.name = name;
		
		if( m_remove_part_srv.call(rp_srv) ){
			if( rp_srv.response.success ){
				ROS_INFO("Success: %s", rp_srv.response.message.c_str());
				return true;
			}
			else{
				ROS_ERROR("Message: %s", rp_srv.response.message.c_str());
				return false;
			}
		}
		else{
			ROS_ERROR("Error calling RemovePart service.");
			return false;
		}
	}		
			
			
	
	bool WorldStateClient::getPartPose( std::string name, geometry_msgs::Pose & part_pose ){

		world_state::GetPartPose pp_srv;
		pp_srv.request.part_name = name;
		
		if( m_part_pose_srv.call(pp_srv) ){
			if( pp_srv.response.success ){

				part_pose = pp_srv.response.part_pose;
				return true;
			}
			else{
				ROS_ERROR("Fail: %s", pp_srv.response.message.c_str());
				return false;
			}
		}
		else{
			ROS_ERROR("Error calling GetPartPose service.");
			return false;
		}
	}
			

			
	bool WorldStateClient::getPartGrabPose( std::string name, geometry_msgs::Pose & part_pose ){

		world_state::GetPartPose pp_srv;
		pp_srv.request.part_name = name;
		
		if( m_part_pose_srv.call(pp_srv) ){
			if( pp_srv.response.success ){

				part_pose = pp_srv.response.part_pose;

				// Flip Z-axis orientation to match robot EE
				double part_r, part_p, part_y;
				getRPY( part_pose, part_r, part_p, part_y );
				setRPY( part_r+3.14159, part_p, part_y, part_pose );
				// Offset to stop right above the part
				part_pose.position.z+= 0.05;
				return true;
			}
			else{
				ROS_ERROR("Fail: %s", pp_srv.response.message.c_str());
				return false;
			}
		}
		else{
			ROS_ERROR("Error calling GetPartPose service.");
			return false;
		}
	}


	bool WorldStateClient::getBinLocation( std::string name, std::vector<std::string> & jn, std::vector<double> & jv ){

		world_state::GetBinLocation bl_srv;
		bl_srv.request.bin_name = name;
		
		if( m_bin_location_srv.call(bl_srv) ){
			if( bl_srv.response.success ){

				jn = bl_srv.response.joint_names;
				jv = bl_srv.response.joint_values;
				return true;
			}
			else{
				ROS_ERROR("Fail: %s", bl_srv.response.message.c_str());
				return false;
			}
		}
		else{
			ROS_ERROR("Error calling GetBinLocation service.");
			return false;
		}
	}
			
	


	bool WorldStateClient::getBoxLocation( std::string name, std::vector<std::string> & jn, std::vector<double> & jv ){

		world_state::GetBoxLocation bl_srv;
		bl_srv.request.box_name = name;
		
		if( m_bin_location_srv.call(bl_srv) ){
			if( bl_srv.response.success ){

				jn = bl_srv.response.joint_names;
				jv = bl_srv.response.joint_values;
				return true;
			}
			else{
				ROS_ERROR("Fail: %s", bl_srv.response.message.c_str());
				return false;
			}
		}
		else{
			ROS_ERROR("Error calling GetBoxLocation service.");
			return false;
		}

	}



}
