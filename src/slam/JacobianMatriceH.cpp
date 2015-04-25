#include "JacobianMatriceH.hh"

JacobianMatriceH::JacobianMatriceH()
{}

JacobianMatriceH::~JacobianMatriceH()
{}

//delete a landmark through its ID
void JacobianMatriceH::deleteLandmark(unsigned int landmarkNumber)
{
	for (unsigned int i = 0; i < this->matrice.size(); i+=2)
	{
		if (std::get<0>(matrice.at(i)) == landmarkNumber)
			{
				matrice.erase(matrice.begin()+i);
				matrice.erase(matrice.begin()+i+1);
				rnbMatrice.erase(rnbMatrice.begin()+(i/2));
			}
	}
}

//resize the matrices. why not
void JacobianMatriceH::resize(int newSize)
{
	matrice.resize(newSize*2);
	rnbMatrice.resize(newSize);
}

/*
	matrice containing the calculations on range and bearing for each landmarks
	first element is range, second one is bearing.
	same here, the first part of the first pair is the landmark id
*/
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
	std::pair<unsigned int,std::pair<double,double>> IDPairing(landmarkNumber, pairing);

	this->rnbMatrice.push_back(IDPairing);
}

const std::pair<double, double> JacobianMatriceH::getRnBMatrice(unsigned int landmarkNumber) const
{
	return std::get<1>(this->rnbMatrice.at(landmarkNumber));
}

/*
	matrice 2x
	for each landmark there are two pairs, one for the range(X & Y)
	and one for the bearing (X & Y)
	the first part of the first pair is the landmark id, the second part is the range and the bearing
*/
void JacobianMatriceH::JacobiAdd(unsigned int landmarkNumber, SystemStateMatrice stateM, double range)
{
	double rangeX = (stateM.getRobotPos().x - stateM.getLandmarkXPosition(landmarkNumber)) / range;
	double rangeY = (stateM.getRobotPos().y - stateM.getLandmarkYPosition(landmarkNumber)) / range;
	std::pair<double,double> pairRange (-rangeX, -rangeY);

	std::pair<double,std::pair<double,double>> IDPairRange (landmarkNumber, pairRange);

	this->matrice.push_back(IDPairRange);

	double bearingX = (stateM.getLandmarkXPosition(landmarkNumber) - stateM.getRobotPos().x) / pow(range,2.0);
	double bearingY = (stateM.getLandmarkYPosition(landmarkNumber) - stateM.getRobotPos().y) / pow(range,2.0);
	std::pair<double,double> pairBearing (-bearingX, -bearingY);

	std::pair<double,std::pair<double,double>> IDPairBairing (landmarkNumber, pairBearing);

	this->matrice.push_back(IDPairBairing);
}

const std::pair<double,double> JacobianMatriceH::getJacobianRange(unsigned int landmarkNumber) const
{
	return std::get<1>(this->matrice.at(landmarkNumber*2));
}

const std::pair<double,double> JacobianMatriceH::getJacobianBearing(unsigned int landmarkNumber) const
{
	return std::get<1>(this->matrice.at(landmarkNumber*2 + 1));
}
