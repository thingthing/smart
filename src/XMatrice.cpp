#include "XMatrice.hh"

XMatrice::XMatrice() :
  xPosRobot(0.0), yPosRobot(0.0), tetaRobot(0.0)
{
}

XMatrice::XMatrice(double x, double y, double teta) :
  xPosRobot(x), yPosRobot(y), tetaRobot(teta)
{
}

XMatrice::~XMatrice()
{
}

void XMatrice::addLandmarkPosition(const std::pair<double, double> &position)
{
  this->matrice.push_back(std::pair<double, double>(position));
}

void XMatrice::addLandmarkPosition(double x, double y)
{
  this->matrice.push_back(std::pair<double, double>(x, y));
}

void XMatrice::updateLandmarkPosition(unsigned int landmarkNumber, double x, double y)
{
  if (landmarkNumber < this->matrice.size())
    {
      this->matrice[landmarkNumber].first = x;
      this->matrice[landmarkNumber].second = y;
    }
}

const std::pair<double, double> &XMatrice::getPosition(int landmarkNumber) const
{
  return (this->matrice[landmarkNumber]);
}

double XMatrice::getLandmarkXPosition(unsigned int landmarkNumber) const
{
  return (this->matrice[landmarkNumber].first);
}

double XMatrice::getLandmarkYPosition(unsigned int landmarkNumber) const
{
  return (this->matrice[landmarkNumber].second);
}

double XMatrice::getRobotXPosition() const
{
  return (this->xPosRobot);
}

double XMatrice::getRobotYPosition() const
{
  return (this->yPosRobot);
}

double XMatrice::getRobotTeta() const
{
  return (this->tetaRobot);
}
