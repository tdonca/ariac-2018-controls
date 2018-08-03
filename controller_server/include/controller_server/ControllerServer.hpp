#include <ros/ros.h>
#include <iiwa14_controller/ExecuteCartesian.h>
#include <iiwa14_controller/ExecuteJoints.h>
#include <iiwa14_controller/ExecutePose.h>
#include <iiwa14_controller/ExecuteGripper.h>
#include <controller_server/MoveJoints.h>
#include <controller_server/MovePose.h>
#include <controller_server/MoveCartesian.h>
#include <controller_server/ActivateGripper.h>


namespace control {

	class ControllerServer {

		private:

			ros::NodeHandle node_;
			ros::AsyncSpinner spinner_;
			ros::ServiceClient cartesian_srv_;
			ros::ServiceClient joints_srv_;
			ros::ServiceClient pose_srv_;
			ros::ServiceClient gripper_srv_;
			ros::ServiceServer joints_srvs_;
			ros::ServiceServer pose_srvs_;
			ros::ServiceServer cartesian_srvs_;
			ros::ServiceServer gripper_srvs_;


		public:

			ControllerServer()
			:	node_(),
				spinner_(2),
				cartesian_srv_(),
				joints_srv_(),
				pose_srv_(),
				gripper_srv_(),
				joints_srvs_(),
				pose_srvs_(),
				cartesian_srvs_(),
				gripper_srvs_()

			{	
				// DEBUG: Wait for startup prints to finish
				ros::Duration(1.1).sleep();

				ROS_INFO("ControllerServer is waiting for services to appear...");
				cartesian_srv_ = node_.serviceClient<iiwa14_controller::ExecuteCartesian>( "iiwa14_controller/execute_cartesian" );
				joints_srv_ = node_.serviceClient<iiwa14_controller::ExecuteJoints>( "iiwa14_controller/execute_joints" );
				pose_srv_ = node_.serviceClient<iiwa14_controller::ExecutePose>( "iiwa14_controller/execute_pose" );
				gripper_srv_ = node_.serviceClient<iiwa14_controller::ExecuteGripper>( "iiwa14_controller/execute_gripper" );
				cartesian_srv_.waitForExistence();
				joints_srv_.waitForExistence();
				pose_srv_.waitForExistence();
				gripper_srv_.waitForExistence();
				ROS_INFO("ControllerServer services have appeared.");

				joints_srvs_ = node_.advertiseService( "controller_server/move_joints", &ControllerServer::sv_moveJoints, this );
				pose_srvs_ = node_.advertiseService( "controller_server/move_pose", &ControllerServer::sv_movePose, this );
				cartesian_srvs_ = node_.advertiseService( "controller_server/move_cartesian", &ControllerServer::sv_moveCartesian, this );
				gripper_srvs_ = node_.advertiseService( "controller_server/activate_gripper", &ControllerServer::sv_activateGripper, this );


				spinner_.start();
				ROS_INFO("ControllerServer is ready.");
			}



			bool sv_moveJoints( controller_server::MoveJoints::Request & req, controller_server::MoveJoints::Response & rsp );
			bool sv_movePose( controller_server::MovePose::Request & req, controller_server::MovePose::Response & rsp );
			bool sv_moveCartesian( controller_server::MoveCartesian::Request & req, controller_server::MoveCartesian::Response & rsp );
			bool sv_activateGripper( controller_server::ActivateGripper::Request & req, controller_server::ActivateGripper::Response & rsp );

			bool moveJoints( std::vector<std::string> joint_names, std::vector<double> joint_values, std::string & error_message );
			bool movePose( geometry_msgs::Pose goal, std::string & error_message );
			bool moveCartesian( std::vector<geometry_msgs::Pose> waypoints, std::string & error_message );
			bool activateGripper( bool enable, std::string & error_message );

	};



}