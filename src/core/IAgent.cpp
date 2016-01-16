#include "IAgent.hh"

IAgent::IAgent(double degreePerScan, double cameraProblem, std::string const &name, int battery)
  : degreePerScan(degreePerScan), cameraProblem(cameraProblem), _yaw(0),
    _name(name), _status("OK"), _roll(0), _pitch(0), _battery(battery)
{
  this->_pos.x = 0;
  this->_pos.y = 0;
  this->_pos.z = 0;
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
  std::cerr << "New status == [" << _status << "]" << std::endl;
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

ICapture const *IAgent::getCapture() const
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
