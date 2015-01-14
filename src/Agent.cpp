#include "Agent.hh"

const double Agent::DEGREESPERSCAN = 0.5;
const double Agent::CAMERAPROBLEM = 4.1; // meters

Agent::Agent(double degreePerScan, double cameraProblem)
  : _angle(0), degreePerScan(degreePerScan), cameraProblem(cameraProblem)
{
  this->_pos.x = 0;
  this->_pos.y = 0;
  this->_pos.z = 0;
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

void		Agent::setPos(double x, double y , double z)
{
  this->_pos.x = x;
  this->_pos.y = y;
  this->_pos.z = z;
}


void		Agent::setAngle(double angle)
{
  this->_angle = angle;
}
