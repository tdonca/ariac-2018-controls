#ifndef WORLD_WORLDSTATE
#define WORLD_WORLDSTATE

#include <memory>
#include <deque>
#include <ros/ros.h>
#include <world_state/Container.hpp>
#include <world_state/Part.hpp>
#include <world_state/Gripper.hpp>
#include <world_state/Bin.hpp>
#include <world_state/Box.hpp>
#include <world_state/NoneContainer.hpp>
#include <world_state/Sensor.hpp>
#include <world_state/LogicalCameraSensor.hpp>
#include <world_state/QualityControlSensor.hpp>
#include <world_state/Robot.hpp>
#include <world_state/IIWA14Robot.hpp>
#include <world_state/StateGraph.hpp>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/move_group_interface/move_group_interface.h>
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



namespace world {
						//type					//name
	typedef std::unordered_map< std::string, std::unordered_map< std::string, std::weak_ptr<Part> > > WorldPartsMap;
						//name				
	typedef std::unordered_map< std::string, std::unique_ptr<Sensor> > SensorMap;
	typedef std::map< std::string, std::pair<std::string, ros::Time> > TFMap;
	
	class WorldState {
		
		private:

			std::vector<Bin> m_bins;
			std::deque< std::unique_ptr<Box> > m_boxes;
			NoneContainer m_removed;
			SensorMap m_sensors;
			Gripper m_gripper;
			WorldPartsMap m_parts;
			std::vector<std::string> m_faulty_parts; 
			std::unique_ptr<Robot> m_robot;
			StateGraph m_graph;
			TFMap m_tf_list;
			ros::NodeHandle m_node;
			ros::Timer m_update_t;
			ros::Subscriber m_tf_sub;
			ros::ServiceServer m_find_part_type_srv;
			ros::ServiceServer m_mark_part_used_srv;
			ros::ServiceServer m_release_part_srv;
			ros::ServiceServer m_gripper_part_srv;
			ros::ServiceServer m_box_parts_srv;
			ros::ServiceServer m_move_part_to_box_srv;
			ros::ServiceServer m_remove_part_srv;
			ros::ServiceServer m_part_pose_srv;
			ros::ServiceServer m_bin_location_srv;
			ros::ServiceServer m_box_location_srv;
			ros::ServiceServer m_goal_pose_srv;
			tf2_ros::Buffer m_tfBuf;
			tf2_ros::TransformListener m_tfListener;
			moveit::planning_interface::MoveGroupInterface m_move_group;



		public:
			
			WorldState() 
			:	m_bins{ Bin("BIN1"), Bin("BIN2"), Bin("BIN3"), Bin("BIN4"), Bin("BIN5") },
				m_boxes(),
				m_removed("None"),
				m_sensors(),
				m_gripper("Gripper"),
				m_parts(),
				m_robot(),
				m_graph(),
				m_tf_list(),
				m_node(),
				m_update_t(),
				m_tf_sub(),
				m_find_part_type_srv(),
				m_mark_part_used_srv(),
				m_release_part_srv(),
				m_gripper_part_srv(),
				m_box_parts_srv(),
				m_move_part_to_box_srv(),
				m_remove_part_srv(),
				m_part_pose_srv(),
				m_bin_location_srv(),
				m_box_location_srv(),
				m_goal_pose_srv(),
				m_tfBuf(),
				m_tfListener(m_tfBuf),
				m_move_group("manipulator")
			{
				if( initializeWorld() ){
					ROS_INFO("Initialized the world.");
				}
				else{
					ROS_ERROR("World initialization failed!");
				}
			}
			
			bool addNewPart( Part part, int bin );
			
			bool addNewPartToBox( Part part );
			
			bool addNewPartToGripper( Part part );
			
			bool removePart ( std::string part_name );
			
			bool addBox( Box box );
			
			bool removeBox();
			
			bool addCameraSensor( std::string sensor_name );
			
			bool addQualitySensor( std::string sensor_name );
			
			bool removeSensor( std::string sensor_name );
			
			bool addRobot( std::unique_ptr<Robot> robot );
			
			bool removeRobot( std::string robot_name );
			
			bool testFunction();
			
			bool sv_findPartType( world_state::FindPartType::Request & req, world_state::FindPartType::Response & rsp );
			
			bool sv_markPartUsed( world_state::MarkPartUsed::Request & req, world_state::MarkPartUsed::Response & rsp );
			
			bool sv_releasePart( world_state::ReleasePart::Request & req, world_state::ReleasePart::Response & rsp );
			
			bool sv_getGripperPart( world_state::GetGripperPart::Request & req, world_state::GetGripperPart::Response & rsp );
			
			bool sv_getBoxParts( world_state::GetBoxParts::Request & req, world_state::GetBoxParts::Response & rsp );
			
			bool sv_movePartToBox( world_state::MovePartToBox::Request & req, world_state::MovePartToBox::Response & rsp );
			
			bool sv_removePart( world_state::RemovePart::Request & req, world_state::RemovePart::Response & rsp );

			bool sv_getPartPose( world_state::GetPartPose::Request & req, world_state::GetPartPose::Response & rsp );
			
			bool sv_getBinLocation( world_state::GetBinLocation::Request & req, world_state::GetBinLocation::Response & rsp );
			
			bool sv_getBoxLocation( world_state::GetBoxLocation::Request & req, world_state::GetBoxLocation::Response & rsp );
			
			bool sv_computeGoalPose( world_state::ComputeGoalPose::Request & req, world_state::ComputeGoalPose::Response & rsp );
			
		private:
		
			bool initializeWorld();
			
			void cb_updateParts( const ros::TimerEvent & t );
			
			void cb_tfList( const tf2_msgs::TFMessage::ConstPtr & tf_list );
			
			
			
	};
	
}

#endif
