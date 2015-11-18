#ifndef KALMANGAINMATRICE_H_
# define KALMANGAINMATRICE_H_

# include <vector>
# include <map>
# include <tuple>
# include <cmath>

# include "CovarianceMatrice.hh"
# include "JacobianMatriceH.hh"
# include "Landmarks.hh"

class KalmanGainMatrice
{
public:
  KalmanGainMatrice();
  virtual ~KalmanGainMatrice();

  void eraseLandmark(unsigned int landmarkNumber);
  void refresh(const std::vector<Landmarks *> &landmarks, double x, double y, double theta);
  void updateLandmark(JacobianMatriceH const &MatriceH, CovarianceMatrice const &MatriceC);

	const std::pair<double, double> getXLandmarkKalmanGain(unsigned int landmarkNumber) const;
  const std::pair<double, double> getYLandmarkKalmanGain(unsigned int landmarkNumber) const;

private:
  KalmanGainMatrice(const KalmanGainMatrice &);
  KalmanGainMatrice &operator=(const KalmanGainMatrice &);

protected:
	std::map<unsigned int, std::pair<std::pair<double, double>,std::pair<double, double>>> matrice;
	std::map<unsigned int, std::pair<std::pair<double, double>,std::pair<double, double>>> Robot;
	std::pair<double, double> RobotX;
	std::pair<double, double> RobotY;
	std::pair<double, double> RobotTheta;

};

#endif /* !KALMANGAINMATRICE_H_ */
