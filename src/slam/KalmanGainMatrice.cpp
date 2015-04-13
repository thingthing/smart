#include "KalmanGainMatrice.hh"

KalmanGainMatrice::KalmanGainMatrice()
{
  this->_matrice = std::vector<std::pair<double, double > >(3);
  //By default assume that robotPosition is perfect
  this->_matrice[0] = std::make_pair(0.0, 0.0);
  this->_matrice[1] = std::make_pair(0.0, 0.0);
  this->_matrice[2] = std::make_pair(0.0, 0.0);
}

KalmanGainMatrice::~KalmanGainMatrice()
{}

void KalmanGainMatrice::addLandmark(std::pair<double, double> pairX, std::pair<double,double> pairY, unsigned int landmarkNumber)
{
  unsigned int oldSize = _matrice.size();
  //Two cases by landmark + original size 3 (x, y and theta of agent)
  unsigned int index = landmarkNumber * 2 + 3;

  _matrice.resize(oldSize + 2);
  _matrice[index] = pairX;
  _matrice[index + 1] = pairY;
}

void KalmanGainMatrice::eraseLandmark(unsigned int landmarkNumber)
{
  this->_matrice.erase(_matrice.begin()+(3 + 2 * landmarkNumber));
  this->_matrice.erase(_matrice.begin()+(4 + 2 * landmarkNumber));
}

void KalmanGainMatrice::updateLandmark(unsigned int landmarkNumber, std::pair<double, double> pairX, std::pair<double, double> pairY)
{
  this->_matrice[3 + 2 * landmarkNumber] = pairX;
  this->_matrice[4 + 2 * landmarkNumber] = pairY;
}

const std::pair<double, double> &KalmanGainMatrice::getXLandmarkKalmanGain(unsigned int landmarkNumber) const
{
  return this->_matrice[3 + 2 * landmarkNumber]; // 3 + 2*n - 1
}

const std::pair<double, double> &KalmanGainMatrice::getYLandmarkKalmanGain(unsigned int landmarkNumber) const
{
  return this->_matrice[4 + 2 * landmarkNumber]; // 3 + 2*n
}

void KalmanGainMatrice::refresh(const std::vector<Landmarks *> &landmarks, double, double, double)
{
  this->_matrice.resize(3 + 2 * landmarks.size());

  // formula ...
}
