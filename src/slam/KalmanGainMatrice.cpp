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

void KalmanGainMatrice::updateLandmark(JacobianMatriceH const &MatriceH, CovarianceMatrice const &MatriceC)
{
	std::pair <double, double> tempX;
	std::pair <double, double> tempY;

	// this->RobotX = std::make_pair( (MatriceC.getRobotX() * std::get<0>(MatriceH.getJacobianRange(0))) /
	//                                (std::get<0>(MatriceH.getJacobianRange(0)) * MatriceC.getRobotX() * std::get<0>(MatriceH.getJacobianRange(0)))
	//                                , (MatriceC.getRobotX() * std::get<0>(MatriceH.getJacobianBearing(0))) / (std::get<0>(MatriceH.getJacobianBearing(0)) * MatriceC.getRobotX() * std::get<0>(MatriceH.getJacobianBearing(0))) );

	// this->RobotY = std::make_pair( (MatriceC.getRobotY() * std::get<1>(MatriceH.getJacobianRange(0))) /
	//                                (std::get<1>(MatriceH.getJacobianRange(0)) * MatriceC.getRobotY() * std::get<1>(MatriceH.getJacobianRange(0)))
	//                                , (MatriceC.getRobotY() * std::get<1>(MatriceH.getJacobianBearing(0))) / (std::get<1>(MatriceH.getJacobianBearing(0)) * MatriceC.getRobotY() * std::get<0>(MatriceH.getJacobianBearing(0))) );

	// this->RobotTheta = std::make_pair(0, pow(MatriceC.getRobotTheta(), 2.0) * (-1));

	std::map<unsigned int, std::tuple<double, double, double, double>>::const_iterator it;

	for (it = MatriceH.matrice.begin(); it != MatriceH.matrice.end(); ++it) {
		//X range and bearing
		tempX = std::make_pair( (MatriceC.getLandmarkXCovariance(it->first) * std::get<0>(MatriceH.getJacobianRange(it->first))) /
		                        (std::get<0>(MatriceH.getJacobianRange(it->first)) * MatriceC.getLandmarkXCovariance(it->first) * std::get<0>(MatriceH.getJacobianRange(it->first)))
		                        , (MatriceC.getLandmarkXCovariance(it->first) * std::get<0>(MatriceH.getJacobianBearing(it->first))) /
		                        (std::get<0>(MatriceH.getJacobianBearing(it->first)) * MatriceC.getLandmarkXCovariance(it->first) * std::get<0>(MatriceH.getJacobianBearing(it->first))) );

		//Y range and bearing
		tempY = std::make_pair( (MatriceC.getLandmarkYCovariance(it->first) * std::get<1>(MatriceH.getJacobianRange(it->first))) /
		                        (std::get<1>(MatriceH.getJacobianRange(it->first)) * MatriceC.getLandmarkYCovariance(it->first) * std::get<1>(MatriceH.getJacobianRange(it->first)))
		                        , (MatriceC.getLandmarkYCovariance(it->first) * std::get<1>(MatriceH.getJacobianBearing(it->first))) /
		                        (std::get<1>(MatriceH.getJacobianBearing(it->first)) * MatriceC.getLandmarkYCovariance(it->first) * std::get<1>(MatriceH.getJacobianBearing(it->first))) );

		//landmark gain for X(range, bearing) and Y(range, bearing)
		this->matrice[it->first] = std::make_pair(tempX, tempY);
	}
}

const std::pair<double, double> KalmanGainMatrice::getXLandmarkKalmanGain(unsigned int landmarkNumber) const
{
	std::pair<double, double> pairing = std::get<0>(this->matrice.at(landmarkNumber));
	return pairing;
}

const std::pair<double, double> KalmanGainMatrice::getYLandmarkKalmanGain(unsigned int landmarkNumber) const
{
	std::pair<double, double> pairing = std::get<1>(this->matrice.at(landmarkNumber));
	return pairing;
}
