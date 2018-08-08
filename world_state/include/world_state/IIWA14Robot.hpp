#ifndef WORLD_ROBOT_IIWA14
#define WORLD_ROBOT_IIWA14

#include <world_state/Robot.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

namespace world {
	
	class StateGraph;
	class IIWA14Robot: public Robot {
		
		public:
			
			static const int INVALID = 0;
			static const int AT_CONTAINER = 1;
			static const int IN_TRANSITION = 2;
			
			
			IIWA14Robot( std::string name, moveit::planning_interface::MoveGroupInterface & move_group )
			:	m_name(name),
				m_state(INVALID),
				m_graph_state(),
				m_joints(8),
				m_move_group(move_group)
				
			{}
			
			virtual std::string getName() { return m_name; }
			
			virtual std::string getState();
			
			virtual std::string getGraphState() { return m_graph_state; }
			
			virtual std::vector<double> getJoints() { return m_joints; }
			
			virtual bool initialize( StateGraph & graph );
			
			virtual void printRobot();
			
			virtual ~IIWA14Robot() {}
		
		
		private:
			
			std::string m_name;
			int m_state;
			std::string m_graph_state;
			std::vector<double> m_joints;
			moveit::planning_interface::MoveGroupInterface & m_move_group;
		
	};
	
}


#endif
