#include "KalmanGainMatrice.hh"

KalmanGainMatrice::KalmanGainMatrice()
{
  this->_matrice = std::vector<std::pair<double, double > >(3);
}

KalmanGainMatrice::~KalmanGainMatrice()
{}

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
