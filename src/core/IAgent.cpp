#include "IAgent.hh"

IAgent::IAgent(double degreePerScan, double cameraProblem, std::string const &name, int battery)
  : degreePerScan(degreePerScan), cameraProblem(cameraProblem), _bearing(0),
    _name(name), _status("OK"),  _thrust(0), _theta(0), _deltaTheta(0), _battery(battery)
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

double    IAgent::getBearing() const
{
  return (this->_bearing);
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

void    IAgent::setBearing(double bearing)
{
  this->_bearing = bearing;
}

ICapture const *IAgent::getCapture() const
{
  return (this->_capture);
}

double          IAgent::getThrust() const
{
  return (this->_thrust);
}

double          IAgent::getTheta() const
{
  return (this->_theta);
}

double          IAgent::getDeltaTheta() const
{
  return (this->_deltaTheta);
}

void            IAgent::setThrust(double thrust)
{
  this->_thrust = thrust;
}

void            IAgent::setTheta(double theta)
{
  this->_theta = theta;
}

void            IAgent::setDeltaTheta(double deltaTheta)
{
  this->_deltaTheta = deltaTheta;
}