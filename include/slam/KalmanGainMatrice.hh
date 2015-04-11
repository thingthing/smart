#ifndef KALMANGAINMATRICE_H_
# define KALMANGAINMATRICE_H_

# include <vector>
# include <map>

# include "Landmarks.hh"

class KalmanGainMatrice
{
public:
  KalmanGainMatrice();
  virtual ~KalmanGainMatrice();

  const std::pair<double, double> &getXLandmarkKalmanGain(unsigned int landmarkNumber) const;
  const std::pair<double, double> &getYLandmarkKalmanGain(unsigned int landmarkNumber) const;

  void addLandmark(std::pair<double, double> pairX, std::pair<double,double> pairY, unsigned int slamId);
  void eraseLandmark(unsigned int landmarkNumber);
  void refresh(const std::vector<Landmarks *> &landmarks, double x, double y, double theta);
  void updateLandmark(unsigned int landmarkNumber, std::pair<double, double> pairX, std::pair<double, double> pairY);
private:
  KalmanGainMatrice(const KalmanGainMatrice &);
  KalmanGainMatrice &operator=(const KalmanGainMatrice &);

protected:
  std::vector< std::pair<double, double> > _matrice;

};

#endif /* !KALMANGAINMATRICE_H_ */
