#ifndef WORLD_SENSOR_LOGICALCAMERA
#define WORLD_SENSOR_LOGICALCAMERA

#include <world_state/Sensor.hpp>
#include <world_state/Part.hpp>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_listener.h>

namespace world{
	
	typedef std::map< std::string, std::pair<std::string, ros::Time> > TFMap;
	
	class LogicalCameraSensor: public Sensor {
		
		public:
			
			LogicalCameraSensor( std::string name, tf2_ros::Buffer* tfBuf, TFMap & tf_list )
			:	m_name(name),
				m_parts(),
				m_tf_list(tf_list),
				m_node(),
				m_update_t(),
				m_tfBuf(tfBuf)
			{
				
			}
			
			virtual std::string getName() { return m_name; }
			
			virtual bool start();
			
			virtual std::vector<SensorPart> getVisibleParts();
			
			virtual void printSensor();
			
			virtual ~LogicalCameraSensor() {};
		
		
		private:
			
			void addPart( SensorPart part );
			
			void removePart( std::string part_name );
			
			void cb_updateParts( const ros::TimerEvent & t );
			
		
			std::string m_name;
			SensorPartsMap m_parts;
			TFMap & m_tf_list;
			
			ros::NodeHandle m_node;
			ros::Timer m_update_t;
			
			tf2_ros::Buffer* m_tfBuf;
	};
	
	
	
	
}

#endif
