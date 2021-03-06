#ifndef CLIENT_WORLDSTATE
#define CLIENT_WORLDSTATE

#include <ros/ros.h>
#include <world_state/FindPartType.h>
#include <world_state/MarkPartUsed.h>
#include <world_state/ReleasePart.h>
#include <world_state/GetGripperPart.h>
#include <world_state/GetBoxParts.h>
#include <world_state/MovePartToBox.h>
#include <world_state/RemovePart.h>
#include <world_state/GetPartPose.h>
#include <world_state/GetBinLocation.h>
#include <world_state/GetBoxLocation.h>
#include <world_state/ComputeGoalPose.h>
#include <world_state/GetPartContainer.h>
#include <osrf_gear/VacuumGripperState.h>



namespace client {
	
	struct PlannerPart;
	void getRPY( const geometry_msgs::Pose  pose, double & r, double & p, double & y );
	

	class WorldStateClient {
		
		private:

			ros::NodeHandle m_node;
			ros::ServiceClient m_find_part_type_srv;
			ros::ServiceClient m_mark_part_used_srv;
			ros::ServiceClient m_release_part_srv;
			ros::ServiceClient m_gripper_part_srv;
			ros::ServiceClient m_box_parts_srv;
			ros::ServiceClient m_move_part_to_box_srv;
			ros::ServiceClient m_remove_part_srv;
			ros::ServiceClient m_part_pose_srv;
			ros::ServiceClient m_bin_location_srv;
			ros::ServiceClient m_box_location_srv;
			ros::ServiceClient m_goal_pose_srv;
			ros::ServiceClient m_part_container_srv;
			ros::Subscriber m_gripper_state_sub;
			bool m_gripper_has_part;


		public:
			
			WorldStateClient()
			:	m_node(),
				m_find_part_type_srv(),
				m_release_part_srv(),
				m_gripper_part_srv(),
				m_box_parts_srv(),
				m_move_part_to_box_srv(),
				m_remove_part_srv(),
				m_part_pose_srv(),
				m_bin_location_srv(),
				m_box_location_srv(),
				m_goal_pose_srv(),
				m_part_container_srv(),
				m_gripper_state_sub(),
				m_gripper_has_part(false)
			
			{	
				ROS_INFO("WorldStateClient is waiting for services to appear...");
				m_find_part_type_srv = m_node.serviceClient<world_state::FindPartType>("find_part_type");
				m_mark_part_used_srv = m_node.serviceClient<world_state::MarkPartUsed>("mark_part_used");
				m_release_part_srv = m_node.serviceClient<world_state::ReleasePart>("release_part");
				m_gripper_part_srv = m_node.serviceClient<world_state::GetGripperPart>("gripper_part");
				m_box_parts_srv = m_node.serviceClient<world_state::GetBoxParts>("box_parts");
				m_move_part_to_box_srv = m_node.serviceClient<world_state::MovePartToBox>("move_part_to_box");
				m_remove_part_srv = m_node.serviceClient<world_state::RemovePart>("remove_part");
				m_part_pose_srv = m_node.serviceClient<world_state::GetPartPose>("get_part_pose");
				m_bin_location_srv = m_node.serviceClient<world_state::GetBinLocation>("get_bin_location");
				m_box_location_srv = m_node.serviceClient<world_state::GetBoxLocation>("get_box_location");
				m_goal_pose_srv = m_node.serviceClient<world_state::ComputeGoalPose>("compute_goal_pose");
				m_part_container_srv = m_node.serviceClient<world_state::GetPartContainer>("get_part_container");

				m_find_part_type_srv.waitForExistence();
				m_mark_part_used_srv.waitForExistence();
				m_release_part_srv.waitForExistence();
				m_gripper_part_srv.waitForExistence();
				m_box_parts_srv.waitForExistence();
				m_move_part_to_box_srv.waitForExistence();
				m_remove_part_srv.waitForExistence();
				m_part_pose_srv.waitForExistence();
				m_bin_location_srv.waitForExistence();
				m_box_location_srv.waitForExistence();
				m_goal_pose_srv.waitForExistence();
				m_part_container_srv.waitForExistence();
				ROS_INFO("WorldStateClient services have appeared.");


				m_gripper_state_sub = m_node.subscribe( "ariac/gripper/state", 1, &WorldStateClient::cb_gripperState, this);
				
			}
			
			
			
			bool markPartUsed( PlannerPart const & part );
			
			bool releasePart( PlannerPart const & part );
			
			bool getGripperPart( PlannerPart & part_found );
			
			bool getBoxParts( std::vector<PlannerPart> & parts_found );
			
			bool movePartToBox( std::string name );
			
			bool removePart( std::string name );

			bool getPartPose( std::string name, geometry_msgs::Pose & part_pose );
			
			bool getPartAbovePose( std::string name, geometry_msgs::Pose & part_pose );
			
			bool getPartGrabPose( std::string name, geometry_msgs::Pose & part_pose );
			
			bool getBinLocation( std::string name, std::vector<std::string> & jn, std::vector<double> & jv );
			
			bool getBoxLocation( std::string name, std::vector<std::string> & jn, std::vector<double> & jv );

			bool findPartOfType( std::string type, std::string & part_name );

			bool markPartUsed( std::string part_name );

			bool computeGoalPose( geometry_msgs::Pose relative_pose, std::string box_name, geometry_msgs::Pose & goal_pose );

			bool getPartContainer( std::string part_name, std::string & container_name );

			bool gripperHasPart();

		
		private:
		
			void test();
			
			void cb_gripperState( const osrf_gear::VacuumGripperState::ConstPtr & msg );
			
			
		
	};
	
	
	
	struct PlannerPart{
		
		std::string name;
		std::string type;
		std::string id;
		geometry_msgs::Pose current_pose;
		geometry_msgs::Pose goal_pose;
		
		void printPart(){
			double r, p, y;
			getRPY(current_pose, r, p, y );
			ROS_INFO("Name: %s", name.c_str());
			ROS_INFO("Type: %s", type.c_str());
			ROS_INFO("Position: %.3f, %.3f, %.3f",current_pose.position.x,current_pose.position.y,current_pose.position.z);
			ROS_INFO("Orientation: %.3f, %.3f, %.3f, %.3f",current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z,current_pose.orientation.w);
			ROS_INFO("RollPitchYaw: %.3f, %.3f, %.3f", r, p, y);
			ROS_INFO(" ");
		}
	};
}



#endif
