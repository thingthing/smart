#ifndef		__AGENT_AHRS__
#define		__AGENT_AHRS__

#include "IAgent.hh"
#include "myahrs_plus.hpp"

namespace WithRobot
{
	class AgentAhrs : public iMyAhrsPlus
	{
	public:
		AgentAhrs(::IAgent *agent, std::string port="", unsigned int baudrate=115200)
		: iMyAhrsPlus(port, baudrate), sample_count(0), _agent(agent) {}

		~AgentAhrs() {}

		bool				initialize();
		
		inline void			get_data(SensorData &data) {
			LockGuard _l(_lock);
			data = _sensor_data;
		}
		inline SensorData 	get_data() {
			LockGuard _l(_lock);
			return (_sensor_data);
		}

		void				updateAgent();
		static double     	roundValue(double value, double limit);

		int					sample_count;
		static const char* 	DIVIDER;

	protected:
		/*
	     * 	override event handler
	     */
		virtual void 		OnSensorData(int sensor_id, SensorData data);
		virtual void		OnAttributeChange(int sensor_id, std::string attribute_name, std::string value);

	private:
		::IAgent			*_agent;
		Platform::Mutex 	_lock;
		SensorData 			_sensor_data;
	};
};

#endif		/*! __AGENT_AHRS__ */