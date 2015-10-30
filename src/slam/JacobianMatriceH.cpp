#include "JacobianMatriceH.hh"

JacobianMatriceH::JacobianMatriceH()
{}

JacobianMatriceH::~JacobianMatriceH()
{}

//delete a landmark through its ID
void JacobianMatriceH::deleteLandmark(unsigned int landmarkNumber)
{
	matrice.erase(landmarkNumber);
	rnbMatrice.erase(landmarkNumber);
}

	/*matrice containing the calculations on range and bearing for each landmarks
	first element is range, second one is bearing.*/
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
	
	rnbMatrice[landmarkNumber] = pairing;
}

std::pair<double, double> JacobianMatriceH::getRnBMatrice(unsigned int landmarkNumber) const
{
	return rnbMatrice.at(landmarkNumber);
}

	/*for each landmark there are 4 elements, the first two for the range(X & Y) and
	the other two for the bearing(X & Y)*/
void JacobianMatriceH::JacobiAdd(unsigned int landmarkNumber, SystemStateMatrice stateM)
{
	//range between Robot and Landmark
	double range = sqrt(pow(stateM.getRobotPos().x - stateM.getLandmarkXPosition(landmarkNumber), 2) + pow(stateM.getRobotPos().y - stateM.getLandmarkYPosition(landmarkNumber), 2));

	double rangeX = (stateM.getRobotPos().x - stateM.getLandmarkXPosition(landmarkNumber)) / range;
	double rangeY = (stateM.getRobotPos().y - stateM.getLandmarkYPosition(landmarkNumber)) / range;

	double bearingX = (stateM.getLandmarkXPosition(landmarkNumber) - stateM.getRobotPos().x) / pow(range,2.0);
	double bearingY = (stateM.getLandmarkYPosition(landmarkNumber) - stateM.getRobotPos().y) / pow(range,2.0);

	std::tuple<double,double,double,double> grouping (rangeX,rangeY,bearingX,bearingY);
	matrice[landmarkNumber] = grouping;
}

std::pair<double,double> JacobianMatriceH::getJacobianRange(unsigned int landmarkNumber) const
{
	std::pair<double,double> pairing (-std::get<0>(matrice.at(landmarkNumber)),-std::get<1>(matrice.at(landmarkNumber)));
	return pairing;
}

std::pair<double,double> JacobianMatriceH::getJacobianBearing(unsigned int landmarkNumber) const
{
	std::pair<double,double> pairing (-std::get<2>(matrice.at(landmarkNumber)),-std::get<3>(matrice.at(landmarkNumber)));

	return pairing;
}
