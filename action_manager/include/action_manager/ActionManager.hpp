#include <ros/ros.h>
#include <controller_server/MoveJoints.h>
#include <controller_server/MovePose.h>
#include <controller_server/MoveCartesian.h>
#include <controller_server/ActivateGripper.h>
#include <action_manager/DoAction.h>


namespace control {


	class ActionManager {

		private:

			ros::NodeHandle node_;
			ros::AsyncSpinner spinner_;
			ros::ServiceClient cartesian_srv_;
			ros::ServiceClient joints_srv_;
			ros::ServiceClient pose_srv_;
			ros::ServiceClient gripper_srv_;
			ros::ServiceServer action_srv_;


		public:


			ActionManager() 
			: 	node_(),
				spinner_(2),
				cartesian_srv_(),
				joints_srv_(),
				pose_srv_(),
				gripper_srv_(),
				action_srv_()

			{
				// DEBUG: Wait for startup prints to finish
				ros::Duration(1.2).sleep();

				ROS_INFO("ActionManager is waiting for services to appear...");
				cartesian_srv_ = node_.serviceClient<controller_server::MoveCartesian>( "controller_server/move_cartesian" );
				joints_srv_ = node_.serviceClient<controller_server::MoveJoints>( "controller_server/move_joints" );
				pose_srv_ = node_.serviceClient<controller_server::MovePose>( "controller_server/move_pose" );
				gripper_srv_ = node_.serviceClient<controller_server::ActivateGripper>( "controller_server/activate_gripper" );
				cartesian_srv_.waitForExistence();
				joints_srv_.waitForExistence();
				pose_srv_.waitForExistence();
				gripper_srv_.waitForExistence();
				ROS_INFO("ActionManager services have appeared.");

				action_srv_ = node_.advertiseService( "action_manager/do_action", &ActionManager::sv_processAction, this );


				spinner_.start();
				ROS_INFO("ActionManager is ready.");
			}


			bool sv_processAction( action_manager::DoAction::Request & req, action_manager::DoAction::Response & rsp );

			bool approachPart( std::string part_name, std::string & error_message );
			bool grabPart( std::string part_name, std::string & error_message );
			bool placePart( geometry_msgs::Pose goal_pose, std::string & error_message );
			bool moveToBin( std::string bin_name, std::string & error_message );
			bool moveToBox( std::string box_name, std::string & error_message );
	};

}