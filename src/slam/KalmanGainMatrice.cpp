#include "KalmanGainMatrice.hh"

KalmanGainMatrice::KalmanGainMatrice()
{
  this->_matrice = std::vector<std::pair<double, double > >(3);
}

KalmanGainMatrice::~KalmanGainMatrice()
{}

void KalmanGainMatrice::addLandmark(std::pair<double, double> pairX, std::pair<double,double> pairY)
{
_matrice.push_back(pairX);
_matrice.push_back(pairY);
}

void KalmanGainMatrice::eraseLandmark(unsigned int landmarkNumber)
{
this->_matrice.erase(_matrice.begin()+(1 + 2 * landmarkNumber));
this->_matrice.erase(_matrice.begin()+(2 + 2 * landmarkNumber));
}

void KalmanGainMatrice::updateLandmark(unsigned int landmarkNumber, std::pair<double, double> pairX, std::pair<double, double> pairY)
{
this->_matrice[1 + 2 * landmarkNumber] = pairX;
this->_matrice[2 + 2 * landmarkNumber] = pairY;
}

const std::pair<double, double> &KalmanGainMatrice::getXLandmarkKalmanGain(unsigned int landmarkNumber) const
{
  return this->_matrice[1 + 2 * landmarkNumber]; // 3 + 2*n - 1
}

const std::pair<double, double> &KalmanGainMatrice::getYLandmarkKalmanGain(unsigned int landmarkNumber) const
{
  return this->_matrice[2 + 2 * landmarkNumber]; // 3 + 2*n
}

void KalmanGainMatrice::refresh(const std::vector<Landmarks *> &landmarks, double, double, double)
{
  this->_matrice.resize(3 + 2 * landmarks.size());

  // formula ...
}
