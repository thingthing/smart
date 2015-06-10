#include "SystemStateMatrice.hh"

SystemStateMatrice::SystemStateMatrice() :
  tetaRobot(0.0), posRobot(pcl::PointXYZ(0.0, 0.0, 0.0))
{
}

SystemStateMatrice::SystemStateMatrice(float x, float y, float z, float teta) :
  tetaRobot(teta), posRobot(pcl::PointXYZ(x, y, z))
{
}

SystemStateMatrice::SystemStateMatrice(pcl::PointXYZ const &posRobot, float teta) :
  tetaRobot(teta), posRobot(pcl::PointXYZ(posRobot))
{
}

SystemStateMatrice::SystemStateMatrice(Agent const &agent) :
  tetaRobot(agent.getBearing()), posRobot(pcl::PointXYZ(agent.getPos()))
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
  this->matrice.push_back(pcl::PointXYZ(position));
  return (this->matrice.size() - 1);
}

/**
* Add landmark to matrice and return id
**/
unsigned int SystemStateMatrice::addLandmarkPosition(float x, float y, float z)
{
  this->matrice.push_back(pcl::PointXYZ(x, y, z));
  return (this->matrice.size() - 1);
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

void SystemStateMatrice::updateRobotState(Agent const &agent)
{
  this->posRobot.x += agent.getPos().x;
  this->posRobot.y += agent.getPos().y;
  this->posRobot.z += agent.getPos().z;
  this->tetaRobot += agent.getBearing();
}

void SystemStateMatrice::setRobotState(Agent const &agent)
{
  this->posRobot.x = agent.getPos().x;
  this->posRobot.y = agent.getPos().y;
  this->posRobot.z = agent.getPos().z;
  this->tetaRobot = agent.getBearing();
}

const pcl::PointXYZ &SystemStateMatrice::getPosition(unsigned int landmarkNumber) const
{
  return (this->matrice[landmarkNumber]);
}

float SystemStateMatrice::getLandmarkXPosition(unsigned int landmarkNumber) const
{
  return (this->matrice[landmarkNumber].x);
}

float SystemStateMatrice::getLandmarkYPosition(unsigned int landmarkNumber) const
{
  return (this->matrice[landmarkNumber].y);
}

pcl::PointXYZ const &SystemStateMatrice::getRobotPos() const
{
  return (this->posRobot);
}

float SystemStateMatrice::getRobotTeta() const
{
  return (this->tetaRobot);
}
