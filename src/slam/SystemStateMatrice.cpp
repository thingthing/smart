#include "SystemStateMatrice.hh"

SystemStateMatrice::SystemStateMatrice() :
  tetaRobot(0.0), posRobot(pcl::PointXYZ(0.0, 0.0, 0.0)), oldPosRobot(pcl::PointXYZ(0.0, 0.0, 0.0)), slamID(1)
{
}

SystemStateMatrice::SystemStateMatrice(float x, float y, float z, float teta) :
  tetaRobot(teta), posRobot(pcl::PointXYZ(x, y, z)), oldPosRobot(pcl::PointXYZ(x, y, z)), slamID(1)
{
}

SystemStateMatrice::SystemStateMatrice(pcl::PointXYZ const &posRobot, float teta) :
  tetaRobot(teta), posRobot(pcl::PointXYZ(posRobot)), oldPosRobot(pcl::PointXYZ(posRobot)), slamID(1)
{
}


SystemStateMatrice::SystemStateMatrice(IAgent const *agent) :
  tetaRobot(agent->getBearing()), posRobot(pcl::PointXYZ(agent->getPos())), oldPosRobot(pcl::PointXYZ(agent->getPos())), slamID(1)
{
}

SystemStateMatrice::~SystemStateMatrice()
{
}

/**
* Add landmark to matrice and return id
**/
unsigned int SystemStateMatrice::addLandmarkPosition(const pcl::PointXYZ &position)
{
	this->matrice[slamID] = pcl::PointXYZ(position);
  return (slamID++);
}

/**
* Add landmark to matrice and return id
**/
unsigned int SystemStateMatrice::addLandmarkPosition(float x, float y, float z)
{
	this->matrice[slamID] = pcl::PointXYZ(x,y,z);
  return (slamID++);
}

void SystemStateMatrice::updateLandmarkPosition(unsigned int landmarkNumber, float x, float y, float z)
{
  if (landmarkNumber < this->matrice.size())
    {
      this->matrice[landmarkNumber].x = x;
      this->matrice[landmarkNumber].y = y;
      this->matrice[landmarkNumber].z = z;
    }
}

void SystemStateMatrice::updateLandmarkPosition(unsigned int landmarkNumber, const pcl::PointXYZ &position)
{
    if (landmarkNumber < this->matrice.size())
    {
      this->matrice[landmarkNumber].x = position.x;
      this->matrice[landmarkNumber].y = position.y;
      this->matrice[landmarkNumber].z = position.z;
    }
}

void SystemStateMatrice::updateRobotState(IAgent const *agent)
{
	this->oldPosRobot.x = this->posRobot.x;
	this->oldPosRobot.y = this->posRobot.y;
	this->oldPosRobot.z = this->posRobot.z;
	this->oldTetaRobot = this->tetaRobot;

  this->posRobot.x += agent->getPos().x;
  this->posRobot.y += agent->getPos().y;
  this->posRobot.z += agent->getPos().z;
  this->tetaRobot += agent->getBearing();
}

void SystemStateMatrice::setRobotState(IAgent const *agent)
{
	this->oldPosRobot.x = this->posRobot.x;
	this->oldPosRobot.y = this->posRobot.y;
	this->oldPosRobot.z = this->posRobot.z;
	this->oldTetaRobot = this->tetaRobot;

  this->posRobot.x = agent->getPos().x;
  this->posRobot.y = agent->getPos().y;
  this->posRobot.z = agent->getPos().z;
  this->tetaRobot = agent->getBearing();
}

const pcl::PointXYZ &SystemStateMatrice::getPosition(unsigned int landmarkNumber) const
{
  return (this->matrice.at(landmarkNumber));
}

float SystemStateMatrice::getLandmarkXPosition(unsigned int landmarkNumber) const
{
  return (this->matrice.at(landmarkNumber).x);
}

float SystemStateMatrice::getLandmarkYPosition(unsigned int landmarkNumber) const
{
  return (this->matrice.at(landmarkNumber).y);
}

pcl::PointXYZ const &SystemStateMatrice::getRobotPos() const
{
  return (this->posRobot);
}

float SystemStateMatrice::getRobotTeta() const
{
  return (this->tetaRobot);
}

pcl::PointXYZ const &SystemStateMatrice::getRobotOldPos() const
{
  return (this->oldPosRobot);
}

float SystemStateMatrice::getRobotOldTeta() const
{
  return (this->oldTetaRobot);
}
