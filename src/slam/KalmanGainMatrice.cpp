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
	std::pair <double,double> tempX;
	std::pair <double,double> tempY;
	std::pair <double,double> tempRX;
	std::pair <double,double> tempRY;
	double HJRX, HJBX, CRX, HJRY, HJBY, CRY;
	double CLX, CLY;

	// this->RobotTheta = std::make_pair(0, pow(MatriceC.getRobotTheta(), 2.0) * (-1));

	std::map<unsigned int, std::tuple<double, double, double, double>>::const_iterator it;

	for (it = MatriceH.matrice.begin(); it != MatriceH.matrice.end(); ++it) {

		//Robot gain for this landmark
		//X for the robot; range and bearing
		HJRX = -std::get<0>(MatriceH.getJacobianRange(it->first));
		HJBX = -std::get<0>(MatriceH.getJacobianBearing(it->first));
		CRX = MatriceC.getRobotX();

		tempRX = std::make_pair((CRX * HJRX) * (HJRX * CRX * HJRX), (CRX * HJBX) * (HJBX * CRX * HJBX));

		//Y for the robot; range and bearing
		HJRY = -std::get<1>(MatriceH.getJacobianRange(it->first));
		HJBY = -std::get<1>(MatriceH.getJacobianBearing(it->first));
		CRY = MatriceC.getRobotY();

		tempRY = std::make_pair((CRY * HJRY) * (HJRY * CRY * HJRY), (CRY * HJBY) * (HJBY * CRY * HJBY));

		this->Robot[it->first] = std::make_pair(tempRX, tempRY);

		//X range and bearing for the landmark
		CLX = MatriceC.getLandmarkXCovariance(it->first);

		tempX = std::make_pair((CLX * (-HJRX)) * ((-HJRX) * CLX * (-HJRX)), (CLX * (-HJBX)) * ((-HJBX) * CLX * (-HJBX)));

		//Y range and bearing for the landmark
		CLY = MatriceC.getLandmarkYCovariance(it->first); 

		tempY = std::make_pair((CLY * (-HJRY)) * ((-HJRY) * CLY * (-HJRY)), (CLY * (-HJBY)) * ((-HJBY) * CLY * (-HJBY)));


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
	std::pair<double, double> pairing = std::get<0>(this->matrice.at(landmarkNumber));
	return pairing;
}

const std::pair<double, double> KalmanGainMatrice::getYLandmarkKalmanGain(unsigned int landmarkNumber) const
{
	std::pair<double, double> pairing = std::get<1>(this->matrice.at(landmarkNumber));
	return pairing;
}
