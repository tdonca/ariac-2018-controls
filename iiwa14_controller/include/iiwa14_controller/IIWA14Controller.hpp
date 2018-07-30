#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <iiwa14_controller/ExecuteCartesian.h>



namespace control{
namespace iiwa14 {

	class IIWA14Controller {

		private:
			ros::NodeHandle node_;
			ros::AsyncSpinner spinner_;
			moveit::planning_interface::MoveGroupInterface move_group_;
			ros::ServiceServer cartesian_srv_;


		public:

			IIWA14Controller() 
			:	node_(),
				spinner_(2),
				move_group_("manipulator"),
				cartesian_srv_()

			{	
				cartesian_srv_ = node_.advertiseService( "iiwa14_controller/execute_cartesian", &IIWA14Controller::sv_executeCartesian, this );

				spinner_.start();

			}



		private:
			bool sv_executeCartesian( iiwa14_controller::ExecuteCartesian::Request & req, iiwa14_controller::ExecuteCartesian::Response & rsp );
			bool sv_executeJoints( iiwa14_controller::ExecuteJoints::Request & req, iiwa14_controller::ExecuteJoints::Response & rsp );
			bool sv_executeMotion( iiwa14_controller::ExecuteMotion::Request & req, iiwa14_controller::ExecuteMotion::Response & rsp );
			bool sv_executeGripper( iiwa14_controller::ExecuteGripper::Request & req, iiwa14_controller::ExecuteGripper::Response & rsp );

			bool executeCartesian( std::vector<geometry_msgs::Pose> waypoints, std::string & error_message );
			bool executeJoints( std::vector<std::string> jn, std::vector<double> jv, std::string & error_message );
			bool executeMotion( geometry_msgs::Pose goal, std::string & error_message );
			bool executeGripper( bool enable, std::string & error_message );
	};



}
}