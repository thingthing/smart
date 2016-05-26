#include "IAgent.hh"

IAgent::IAgent(double degreePerScan, double cameraProblem, std::string const &name, int battery, IAgent::e_mode mode)
  : degreePerScan(degreePerScan), cameraProblem(cameraProblem), _yaw(0),
    _name(name), _status("OK"), _roll(0), _pitch(0), _prev_roll(0), _prev_pitch(0), _prev_yaw(0),
     _battery(battery), _send_data(false), _mode(mode)
{
  this->_pos.x = 0;
  this->_pos.y = 0;
  this->_pos.z = 0;
  this->_velocity.x = 0.0;
  this->_velocity.y = 0.0;
  this->_velocity.z = 0.0;
  this->_acceleration.x = 0.0;
  this->_acceleration.y = 0.0;
  this->_acceleration.z = 0.0;
}

IAgent::~IAgent()
{
}

int              IAgent::getBattery() const
{
  return (_battery);
}

void             IAgent::setBattery(int new_battery_value)
{
  _battery = new_battery_value;
}

std::string const &IAgent::status(std::string const &status)
{
  _status = status;
  this->dispatch("SendStatusEvent", status);
  return (_status);
}


pcl::PointXYZ const &IAgent::getPos() const
{
  return (this->_pos);
}

double    IAgent::getRoll() const
{
  return (this->_roll);
}

double    IAgent::getPrevRoll() const
{
  return (this->_prev_roll);
}

void    IAgent::setPos(pcl::PointXYZ const &pos)
{
  this->_pos.x = pos.x;
  this->_pos.y = pos.y;
  this->_pos.z = pos.z;
}

void    IAgent::setPos(double x, double y , double z)
{
  this->_pos.x = x;
  this->_pos.y = y;
  this->_pos.z = z;
}

void    IAgent::setRoll(double roll)
{
  this->_roll = roll;
}

void    IAgent::setPrevRoll(double roll)
{
  this->_prev_roll = roll;
}

ICapture *IAgent::getCapture() const
{
  return (this->_capture);
}

double          IAgent::getYaw() const
{
  return (this->_yaw);
}

double          IAgent::getPitch() const
{
  return (this->_pitch);
}

void            IAgent::setYaw(double yaw)
{
  this->_yaw = yaw;
}

void            IAgent::setPitch(double pitch)
{
  this->_pitch = pitch;
}

double          IAgent::getPrevYaw() const
{
  return (this->_prev_yaw);
}

double          IAgent::getPrevPitch() const
{
  return (this->_prev_pitch);
}

void            IAgent::setPrevYaw(double yaw)
{
  this->_prev_yaw = yaw;
}

void            IAgent::setPrevPitch(double pitch)
{
  this->_prev_pitch = pitch;
}