#include "JacobianMatriceH.hh"

JacobianMatriceH::JacobianMatriceH()
{
	this->bearingH = 0;
	this->rangeH = 0;
}

JacobianMatriceH::~JacobianMatriceH()
{}

const std::tuple<double,double,double> &JacobianMatriceH::getJacobianRange(unsigned int landmarkNumber) const
{
  return this->rangeH;
}

const std::tuple<double,double,double> &JacobianMatriceH::getJacobianBearing(unsigned int landmarkNumber) const
{
  return this->bearingH;
}

void JacobianMatriceH::JacobiMath(unsigned int landmarkNumber, SystemStateMatrice stateM)
{
	float range = sqrt(pow(stateM.getLandmarkXPosition(landmarkNumber) - stateM.getRobotPos().x, 2.0) + 
		pow(stateM.getLandmarkYPosition(landmarkNumber) - stateM.getRobotPos().y, 2.0))
		+ range_innovation;
	float bearing = atan((stateM.getLandmarkYPosition(landmarkNumber) - stateM.getRobotPos().y) /
		(stateM.getLandmarkXPosition(landmarkNumber) - stateM.getRobotPos().x)) - 
		stateM.getRobotTeta() + bearing_innovation;

	double rangeX = (stateM.getRobotPos().x - stateM.getLandmarkXPosition(landmarkNumber)) / range;
	double rangeY = (stateM.getRobotPos().y - stateM.getLandmarkYPosition(landmarkNumber)) / range;
	std::get<0>(rangeH) = rangeX;
	std::get<1>(rangeH) = rangeY;
	std::get<2>(rangeH) = 0.0;

	double bearingX = (stateM.getLandmarkXPosition(landmarkNumber) - stateM.getRobotPos().x) / bearing;
	double bearingY = (stateM.getLandmarkYPosition(landmarkNumber) - stateM.getRobotPos().y) / bearing;
	std::get<0>(bearingH) = bearingX;
	std::get<1>(bearingH) = bearingY;
	std::get<2>(bearingH) = -1.0;
}
