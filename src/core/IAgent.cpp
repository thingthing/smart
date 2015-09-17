#include "IAgent.hh"

IAgent::IAgent(double degreePerScan, double cameraProblem, std::string const &name)
  : degreePerScan(degreePerScan), cameraProblem(cameraProblem), _bearing(0),
    _name(name)
{
  this->_pos.x = 0;
  this->_pos.y = 0;
  this->_pos.z = 0;
}

IAgent::~IAgent()
{
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

Capture const &IAgent::getCapture() const
{
  return (this->_capture);
}

