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

  void refresh(const std::vector<Landmarks *> &landmarks, double x, double y, double theta);

private:
  KalmanGainMatrice(const KalmanGainMatrice &);
  KalmanGainMatrice &operator=(const KalmanGainMatrice &);

protected:
  std::vector< std::pair<double, double> > _matrice;

};

#endif /* !KALMANGAINMATRICE_H_ */
