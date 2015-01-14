#include "Agent.hh"

Agent::Agent()
{
  this->_pos.x = 0;
  this->_pos.y = 0;
  this->_pos.z = 0;
  this->_angle = 0;
}

Agent::~Agent()
{

}

pcl::PointXYZ	const &Agent::getPos() const
{
  return (this->_pos);
}

double		Agent::getAngle() const
{
  return (this->_angle);
}

void		Agent::setPos(pcl::PointXYZ const &pos)
{
  this->_pos.x = pos.x;
  this->_pos.y = pos.y;
  this->_pos.z = pos.z;
}


void		Agent::setAngle(double angle)
{
  this->_angle = angle;
}
