#include "JacobianMatriceH.hh"

JacobianMatriceH::JacobianMatriceH()
{
	/*this->bearingH = std::tuple<double,double,double>(0.0, 0.0, 0.0);
	this->rangeH = std::tuple<double,double,double>(0.0, 0.0, 0.0);*/
}

JacobianMatriceH::~JacobianMatriceH()
{}

void JacobianMatriceH::setRnBMatrice(unsigned int landmarkNumber, SystemStateMatrice stateM)
{
	double range_innovation = 0;
	double bearing_innovation = 0;

	double range = sqrt(pow(stateM.getLandmarkXPosition(landmarkNumber) - stateM.getRobotPos().x, 2.0) + 
	pow(stateM.getLandmarkYPosition(landmarkNumber) - stateM.getRobotPos().y, 2.0)) + range_innovation;

	double bearing = atan((stateM.getLandmarkYPosition(landmarkNumber) - stateM.getRobotPos().y) /
	(stateM.getLandmarkXPosition(landmarkNumber) - stateM.getRobotPos().x)) - 
	stateM.getRobotTeta() + bearing_innovation;

	std::pair<double,double> pairing (range, bearing);
	this->rnbMatrice.push_back(pairing);
}

std::pair<double, double> JacobianMatriceH::getRnBMatrice(unsigned int landmarkNumber) const
{
	return this->rnbMatrice.at(landmarkNumber);
}

void JacobianMatriceH::JacobiMath(unsigned int landmarkNumber, SystemStateMatrice stateM, double range)
{
	double rangeX = (stateM.getRobotPos().x - stateM.getLandmarkXPosition(landmarkNumber)) / range;
	double rangeY = (stateM.getRobotPos().y - stateM.getLandmarkYPosition(landmarkNumber)) / range;
	std::pair<double,double> pairRange (-rangeX, -rangeY);

	this->matrice.push_back(pairRange);

	double bearingX = (stateM.getLandmarkXPosition(landmarkNumber) - stateM.getRobotPos().x) / pow(range,2.0);
	double bearingY = (stateM.getLandmarkYPosition(landmarkNumber) - stateM.getRobotPos().y) / pow(range,2.0);
	std::pair<double,double> pairBearing (-bearingX, -bearingY);

	this->matrice.push_back(pairBearing);
}

std::pair<double,double> JacobianMatriceH::getJacobianRange(unsigned int landmarkNumber) const
{
	return this->matrice.at(landmarkNumber*2);
}

std::pair<double,double> JacobianMatriceH::getJacobianBearing(unsigned int landmarkNumber) const
{
	return this->matrice.at(landmarkNumber*2 + 1);
}
