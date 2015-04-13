#ifndef SYSTEMSTATEMATRICE_H_
# define SYSTEMSTATEMATRICE_H_

#include <pcl/common/common.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/common/projection_matrix.h>
#include <vector>
#include <map>
#include "Agent.hh"

class SystemStateMatrice
{
public:
  SystemStateMatrice();
  SystemStateMatrice(float X, float Y, float z, float Theta);
  SystemStateMatrice(pcl::PointXYZ const &posRobot, float Theta);
  SystemStateMatrice(Agent const &agent);
  virtual ~SystemStateMatrice();

  unsigned int addLandmarkPosition(const pcl::PointXY &position);
  unsigned int addLandmarkPosition(float x, float y);
  void updateLandmarkPosition(unsigned int landmarkNumber, float x, float y);
  void updateLandmarkPosition(unsigned int landmarkNumber, const pcl::PointXY &position);
  void updateRobotState(Agent const &agent);
  void setRobotState(Agent const &);

  const pcl::PointXY &getPosition(unsigned int landmarkNumber) const;
  float getLandmarkXPosition(unsigned int landmarkNumber) const;
  float getLandmarkYPosition(unsigned int landmarkNumber) const;
  pcl::PointXYZ const &getRobotPos() const;
  float getRobotTeta() const;

protected:
  float tetaRobot;
  pcl::PointXYZ posRobot;
  std::vector<pcl::PointXY> matrice;
};

#endif /* !SYSTEMSTATEMATRICE_H_ */
