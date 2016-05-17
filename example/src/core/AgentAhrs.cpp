#include <stdio.h>
#include "AgentAhrs.hh"

const char* WithRobot::AgentAhrs::DIVIDER = "100";  // 100 Hz

double          WithRobot::AgentAhrs::roundValue(double value, double limit)
{
  return (std::nearbyint(value * limit) / limit);
}

bool 	WithRobot::AgentAhrs::initialize()
{
	bool ok = false;
    do {
        if(start() == false) break;
        if(cmd_binary_data_format("QUATERNION, IMU") == false) break;
        if(cmd_divider(DIVIDER) == false) break;
        if(cmd_mode("BC") == false) break;
        ok = true;
    } while(0);

    return ok;
}

void		WithRobot::AgentAhrs::updateAgent()
{
	static double old_time = time(NULL);
	static double new_time = 0;
	double        delta = 0;

	pcl::PointXYZ new_velocity;
	pcl::PointXYZ new_pos;
	pcl::PointXYZ acceleration;
	pcl::PointXYZ velocity;
	static pcl::PointXYZ gravity = pcl::PointXYZ(0, 0, -1);
	pcl::PointXYZ current_gravity;
	double        ax, ay, az = 0.0;

	new_time = time(NULL);
	delta = 100.0 / (std::stoi(DIVIDER)); // 100 HZ
  old_time = new_time;

  WithRobot::Quaternion& q = _sensor_data.quaternion;
  WithRobot::ImuData<float>& imu = _sensor_data.imu;
  WithRobot::EulerAngle e = _sensor_data.euler_angle;
  
  std::string line(50, '-');
  printf("%s\n", line.c_str());
  printf("%04d) Quaternion(xyzw)=%.4f,%.4f,%.4f,%.4f, Angle(rpy)=%.1f, %.1f, %.1f, Accel(xyz)=%.4f,%.4f,%.4f, Gyro(xyz)=%.4f,%.4f,%.4f, Magnet(xyz)=%.2f,%.2f,%.2f\n",
              sample_count,
              q.x, q.y, q.z, q.w,
              e.roll, e.pitch, e.yaw,
              imu.ax, imu.ay, imu.az,
              imu.gx, imu.gy, imu.gz,
              imu.mx, imu.my, imu.mz);

  ax = roundValue(imu.ax, 10.0);
  ay = roundValue(imu.ay, 10.0);
  az = roundValue(imu.az, 10.0);

 	_agent->setPitch(roundValue(e.pitch, 10.0));
  _agent->setRoll(roundValue(e.roll, 10.0));
  _agent->setYaw(roundValue(e.yaw, 10.0));
  
  // Translation of gravity vector with roll(x) and pitch(y) axis
  Eigen::Affine3f transfo = pcl::getTransformation (0, 0, 0, -e.roll, -e.pitch, -e.yaw);
  //std::cerr << "Roll == " << _agent->getRoll() << " -- pitch == " << _agent->getPitch() << " -- yaw == " << _agent->getYaw() << std::endl;
 // std::cerr << "gx == " << imu.gx * 180.0 / M_PI << " -- gy == " << imu.gy * 180.0 / M_PI << " -- gz == " << imu.gz * 180.0 / M_PI << std::endl;
  //std::cerr << "Gravity before == " << gravity << std::endl;
  current_gravity = pcl::transformPoint(gravity, transfo);
  current_gravity.x = roundValue(current_gravity.x, 10.0);
  current_gravity.y = roundValue(current_gravity.y, 10.0);
  current_gravity.z = roundValue(current_gravity.z, 10.0);

  //std::cerr << "Current gravity == " << current_gravity << std::endl;
  //std::cerr << "Current acceleration == " << ax << " -- " << ay << " -- " << az << std::endl;
  acceleration = _agent->getAcceleration();
  //std::cerr << "delta time is == " << delta << std::endl;
  if (delta > 0) {
  	velocity = _agent->getVelocity();
  	new_pos = _agent->getPos();
    //std::cerr << "New pos before == " << _agent->getPos() << std::endl;

    	//Using double integration to get position
    new_velocity.x = velocity.x + ((ax - current_gravity.x) + acceleration.x) / (2 * delta);
		new_pos.x = new_pos.x + (new_velocity.x + velocity.x) / (2 * delta);
    new_velocity.y = velocity.y + ((ay - current_gravity.y) + acceleration.y) / (2 * delta);
    new_pos.y = new_pos.y + (new_velocity.y + velocity.y) / (2 * delta);
    new_velocity.z = velocity.z + ((az - current_gravity.z) + acceleration.z) / (2 * delta);
    new_pos.z = new_pos.z + (new_velocity.z + velocity.z) / (2 * delta);

    //Just acceleration
    // new_velocity.x = _velocity.x + (imu.ax - gravity.x) * delta;
    // new_pos.x = _pos.x + new_velocity.x * delta;
    // new_velocity.y = _velocity.y + (imu.ay - gravity.y) * delta;
    // new_pos.y = _pos.y + (new_velocity.y * delta);
    // new_velocity.z = _velocity.z + ((imu.az - gravity.z) * delta);
    // new_pos.z = _pos.z + (new_velocity.z * delta);
    
    _agent->setVelocity(new_velocity);
    _agent->setPos(roundValue(new_pos.x, 10.0), roundValue(new_pos.y, 10.0), roundValue(new_pos.z, 10.0));
    //std::cerr << "New pos == " << _agent->getPos() << std::endl;
    //std::cerr << "Posx == " << new_pos.x << std::endl;
  }
  acceleration.x = roundValue(ax - current_gravity.x, 10.0);
  acceleration.y = roundValue(ay - current_gravity.y, 10.0);
  acceleration.z = roundValue(az - current_gravity.z, 10.0);
  _agent->setAcceleration(acceleration);
}

void 		WithRobot::AgentAhrs::OnSensorData(int sensor_id, SensorData data)
{
	this->sample_count++;
    {
        LockGuard _l(_lock);
        _sensor_data = data;
        _sensor_data.euler_angle = _sensor_data.quaternion.to_euler_angle();
    }

    /*
     * 	do something for arrived data.
     */
     this->updateAgent();
}

void		WithRobot::AgentAhrs::OnAttributeChange(int sensor_id, std::string attribute_name, std::string value)
{
	std::cout << "OnAttributeChange(id " << sensor_id << ", " << attribute_name << ", " << value << ")"<< std::endl;
}
