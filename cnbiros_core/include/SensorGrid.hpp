#ifndef CNBIROS_CORE_SENSORGRID_HPP
#define CNBIROS_CORE_SENSORGRID_HPP

#include <grid_map_ros/grid_map_ros.hpp>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>

namespace cnbiros {
	namespace core {

class SensorGrid : public grid_map::GridMap {

	public:
		SensorGrid(void);
		SensorGrid(const float x, const float y, const float r);
		~SensorGrid(void);

		void AddLayer(const std::string& layer);
		bool RemoveLayer(const std::string& layer);
		bool Exists(const std::string& layer);
		void SetGeometry(const float x, const float y, const float r);
		grid_map_msgs::GridMap ToMessage(void);
		
		void SetFrame(const std::string& frame);
		std::string GetFrame(void);
		void Reset(const std::string& layer, float value = 0.0f);
		void Reset(float value = 0.0f);
		
		bool Sum(const std::string& target);
		bool Sum(const std::string& target, const std::vector<std::string>& layers);


		bool ReplaceNaN(const std::string& layer, float value = 0.0f);
		bool ReplaceNaN(float value = 0.0f);

		bool SetMin(const std::string& layer, float minimum);
		bool SetMax(const std::string& layer, float maximum);
		bool SetMinMax(const std::string& layer, float minimum, float maximum);


		void Update(const std::string& layer, sensor_msgs::LaserScan& msg, float radius);
		void Update(const std::string& layer, grid_map::Matrix& data);

		//bool Transform(const std::string& layer, const std::string& parent);

	private:
		//tf::TransformListener 	rostf_listener_;
};

	}
}


#endif
