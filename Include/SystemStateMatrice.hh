#ifndef SYSTEMSTATEMATRICE_H_
# define SYSTEMSTATEMATRICE_H_

#include <vector>
#include <map>

class SystemStateMatrice
{
public:
  SystemStateMatrice();
  SystemStateMatrice(double X, double Y, double Teta);
  virtual ~SystemStateMatrice();

  void addLandmarkPosition(const std::pair<double, double> &position);
  void addLandmarkPosition(double x, double y);
  void updateLandmarkPosition(unsigned int landmarkNumber, double x, double y);
  const std::pair<double, double> &getPosition(int landmarkNumber) const;
  double getLandmarkXPosition(unsigned int landmarkNumber) const;
  double getLandmarkYPosition(unsigned int landmarkNumber) const;
  double getRobotXPosition() const;
  double getRobotYPosition() const;
  double getRobotTeta() const;

protected:
  double xPosRobot;
  double yPosRobot;
  double tetaRobot;
  std::vector<std::pair<double, double> > matrice;
};

#endif /* !SYSTEMSTATEMATRICE_H_ */
