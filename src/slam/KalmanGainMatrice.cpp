#include "KalmanGainMatrice.hh"

KalmanGainMatrice::KalmanGainMatrice()
{}

KalmanGainMatrice::~KalmanGainMatrice()
{}

void KalmanGainMatrice::eraseLandmark(unsigned int landmarkNumber)
{
  this->matrice.erase(landmarkNumber);
  this->matrice.erase(landmarkNumber);
}

void KalmanGainMatrice::updateLandmark(JacobianMatriceH MatriceH, CovarianceMatrice MatriceC)
{
	std::pair <double,double> tempX;
	std::pair <double,double> tempY;
	std::pair <double,double> tempRX;
	std::pair <double,double> tempRY;

	this->RobotTheta = std::make_pair(0, pow(MatriceC.getRobotTheta(),2.0) * (-1));

	std::map<unsigned int, std::tuple<double,double,double,double>>::iterator it;

	for (it = MatriceH.matrice.begin(); it != MatriceH.matrice.end(); ++it) {

		//Robot gain for this landmark
		//X for the robot; range and bearing
		tempRX = std::make_pair( (MatriceC.getRobotX() * (-std::get<0>(MatriceH.getJacobianRange(it->first)))) *
		(-std::get<0>(MatriceH.getJacobianRange(it->first)) * MatriceC.getRobotX() * (-std::get<0>(MatriceH.getJacobianRange(it->first))))
		, (MatriceC.getRobotX() * (-std::get<0>(MatriceH.getJacobianBearing(it->first)))) *
		(-std::get<0>(MatriceH.getJacobianBearing(it->first)) * MatriceC.getRobotX() * (-std::get<0>(MatriceH.getJacobianBearing(it->first)))) );

		//Y for the robot; range and bearing
		tempRY = std::make_pair( (MatriceC.getRobotY() * (-std::get<1>(MatriceH.getJacobianRange(it->first)))) *
		(-std::get<1>(MatriceH.getJacobianRange(it->first)) * MatriceC.getRobotY() * (-std::get<1>(MatriceH.getJacobianRange(it->first))))
		, (MatriceC.getRobotY() * (-std::get<1>(MatriceH.getJacobianBearing(it->first)))) *
		(-std::get<1>(MatriceH.getJacobianBearing(it->first)) * MatriceC.getRobotY() * (-std::get<0>(MatriceH.getJacobianBearing(it->first)))) );

		this->Robot[it->first] = std::make_pair(tempRX, tempRY);

		//X range and bearing
		tempX = std::make_pair( (MatriceC.getLandmarkXCovariance(it->first) * std::get<0>(MatriceH.getJacobianRange(it->first))) *
		(std::get<0>(MatriceH.getJacobianRange(it->first)) * MatriceC.getLandmarkXCovariance(it->first) * std::get<0>(MatriceH.getJacobianRange(it->first)))
		,(MatriceC.getLandmarkXCovariance(it->first) * std::get<0>(MatriceH.getJacobianBearing(it->first))) *
		(std::get<0>(MatriceH.getJacobianBearing(it->first)) * MatriceC.getLandmarkXCovariance(it->first) * std::get<0>(MatriceH.getJacobianBearing(it->first))) );

		//Y range and bearing
		tempY = std::make_pair( (MatriceC.getLandmarkYCovariance(it->first) * std::get<1>(MatriceH.getJacobianRange(it->first))) *
		(std::get<1>(MatriceH.getJacobianRange(it->first)) * MatriceC.getLandmarkYCovariance(it->first) * std::get<1>(MatriceH.getJacobianRange(it->first)))
		,(MatriceC.getLandmarkYCovariance(it->first) * std::get<1>(MatriceH.getJacobianBearing(it->first))) *
		(std::get<1>(MatriceH.getJacobianBearing(it->first)) * MatriceC.getLandmarkYCovariance(it->first) * std::get<1>(MatriceH.getJacobianBearing(it->first))) );

		//landmark gain for X(range, bearing) and Y(range, bearing)
		this->matrice[it->first] = std::make_pair(tempX, tempY);
	}

	double stockRX = 0;
	double stockBX = 0;
	double stockRY = 0;
	double stockBY = 0;
	int inc = 0;
	std::map<unsigned int, std::pair<std::pair<double, double>,std::pair<double, double>>>::iterator it2;

	for (it2 = Robot.begin(); it2 != Robot.end(); ++it2) {
		stockRX += std::get<0>(std::get<0>(it2->second));
		stockBX += std::get<0>(std::get<1>(it2->second));
		stockRY += std::get<1>(std::get<0>(it2->second));
		stockBY += std::get<1>(std::get<1>(it2->second));
		inc++;
	}
	this->RobotX = std::make_pair(stockRX / inc, stockBX / inc);
	this->RobotY = std::make_pair(stockRY / inc, stockBY / inc);
}

const std::pair<double, double> KalmanGainMatrice::getXLandmarkKalmanGain(unsigned int landmarkNumber) const
{
std::pair<double,double> pairing = std::get<0>(this->matrice.at(landmarkNumber));
  return pairing;
}

const std::pair<double, double> KalmanGainMatrice::getYLandmarkKalmanGain(unsigned int landmarkNumber) const
{
std::pair<double,double> pairing = std::get<1>(this->matrice.at(landmarkNumber));
  return pairing;
}
