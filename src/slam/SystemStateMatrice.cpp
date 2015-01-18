#include "SystemStateMatrice.hh"

SystemStateMatrice::SystemStateMatrice() :
  xPosRobot(0.0), yPosRobot(0.0), tetaRobot(0.0)
{
}

SystemStateMatrice::SystemStateMatrice(double x, double y, double teta) :
  xPosRobot(x), yPosRobot(y), tetaRobot(teta)
{
}

SystemStateMatrice::~SystemStateMatrice()
{
}

void SystemStateMatrice::addLandmarkPosition(const std::pair<double, double> &position)
{
  this->matrice.push_back(std::pair<double, double>(position));
}

void SystemStateMatrice::addLandmarkPosition(double x, double y)
{
  this->matrice.push_back(std::pair<double, double>(x, y));
}

void SystemStateMatrice::updateLandmarkPosition(unsigned int landmarkNumber, double x, double y)
{
  if (landmarkNumber < this->matrice.size())
    {
      this->matrice[landmarkNumber].first = x;
      this->matrice[landmarkNumber].second = y;
    }
}

const std::pair<double, double> &SystemStateMatrice::getPosition(int landmarkNumber) const
{
  return (this->matrice[landmarkNumber]);
}

double SystemStateMatrice::getLandmarkXPosition(unsigned int landmarkNumber) const
{
  return (this->matrice[landmarkNumber].first);
}

double SystemStateMatrice::getLandmarkYPosition(unsigned int landmarkNumber) const
{
  return (this->matrice[landmarkNumber].second);
}

double SystemStateMatrice::getRobotXPosition() const
{
  return (this->xPosRobot);
}

double SystemStateMatrice::getRobotYPosition() const
{
  return (this->yPosRobot);
}

double SystemStateMatrice::getRobotTeta() const
{
  return (this->tetaRobot);
}
