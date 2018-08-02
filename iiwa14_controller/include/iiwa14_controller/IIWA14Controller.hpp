#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <iiwa14_controller/ExecuteCartesian.h>
#include <iiwa14_controller/ExecuteJoints.h>
#include <iiwa14_controller/ExecutePose.h>
#include <iiwa14_controller/ExecuteGripper.h>
#include <osrf_gear/VacuumGripperControl.h>


namespace control{
namespace iiwa14 {

	class IIWA14Controller {

		private:
			ros::NodeHandle node_;
			ros::AsyncSpinner spinner_;
			moveit::planning_interface::MoveGroupInterface move_group_;
			ros::ServiceServer cartesian_srv_;
			ros::ServiceServer joints_srv_;
			ros::ServiceServer pose_srv_;
			ros::ServiceServer gripper_srv_;
			ros::ServiceClient vacuum_srv_;


		public:

			IIWA14Controller() 
			:	node_(),
				spinner_(2),
				move_group_("manipulator"),
				cartesian_srv_(),
				joints_srv_(),
				pose_srv_(),
				gripper_srv_(),
				vacuum_srv_()

			{	
				vacuum_srv_ = node_.serviceClient<osrf_gear::VacuumGripperControl>("ariac/gripper/control");

				cartesian_srv_ = node_.advertiseService( "iiwa14_controller/execute_cartesian", &IIWA14Controller::sv_executeCartesian, this );
				joints_srv_ = node_.advertiseService( "iiwa14_controller/execute_joints", &IIWA14Controller::sv_executeJoints, this );
				pose_srv_ = node_.advertiseService( "iiwa14_controller/execute_pose", &IIWA14Controller::sv_executePose, this );
				gripper_srv_ = node_.advertiseService( "iiwa14_controller/execute_gripper", &IIWA14Controller::sv_executeGripper, this );

				spinner_.start();

			}



		private:
			bool sv_executeCartesian( iiwa14_controller::ExecuteCartesian::Request & req, iiwa14_controller::ExecuteCartesian::Response & rsp );
			bool sv_executeJoints( iiwa14_controller::ExecuteJoints::Request & req, iiwa14_controller::ExecuteJoints::Response & rsp );
			bool sv_executePose( iiwa14_controller::ExecutePose::Request & req, iiwa14_controller::ExecutePose::Response & rsp );
			bool sv_executeGripper( iiwa14_controller::ExecuteGripper::Request & req, iiwa14_controller::ExecuteGripper::Response & rsp );

			bool executeCartesian( std::vector<geometry_msgs::Pose> waypoints, std::string & error_message );
			bool executeJoints( std::vector<std::string> jn, std::vector<double> jv, std::string & error_message );
			bool executePose( geometry_msgs::Pose goal, std::string & error_message );
			bool executeGripper( bool enable, std::string & error_message );
	};



}
}