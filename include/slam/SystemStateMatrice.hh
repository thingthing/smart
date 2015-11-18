#ifndef SYSTEMSTATEMATRICE_H_
# define SYSTEMSTATEMATRICE_H_

#include <pcl/common/common.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/common/projection_matrix.h>
#include <vector>
#include <map>
#include "IAgent.hh"

class SystemStateMatrice
{
public:
  SystemStateMatrice();
  SystemStateMatrice(float X, float Y, float z, float Theta);
  SystemStateMatrice(pcl::PointXYZ const &posRobot, float Theta);
  SystemStateMatrice(IAgent const &agent);
  virtual ~SystemStateMatrice();

  unsigned int addLandmarkPosition(const pcl::PointXYZ &position);
  unsigned int addLandmarkPosition(float x, float y, float z);
	void moveLandmarkPosition(unsigned int landmarkNumber, float x, float y, float z);
  void updateLandmarkPosition(unsigned int landmarkNumber, float x, float y, float z);
  void updateLandmarkPosition(unsigned int landmarkNumber, const pcl::PointXYZ &position);
  void updateRobotState(IAgent const &agent);
  void setRobotState(IAgent const &);

  const pcl::PointXYZ getPosition(unsigned int landmarkNumber);
  float getLandmarkXPosition(unsigned int landmarkNumber);
  float getLandmarkYPosition(unsigned int landmarkNumber);
  pcl::PointXYZ const &getRobotPos() const;
  float getRobotTeta() const;
	pcl::PointXYZ const &getRobotOldPos() const;
  float getRobotOldTeta() const;
	std::map<unsigned int, pcl::PointXYZ> getMatrice();

protected:
  float tetaRobot, oldTetaRobot;
  pcl::PointXYZ posRobot;
	pcl::PointXYZ oldPosRobot;
	unsigned int slamID;
  std::map<unsigned int, pcl::PointXYZ> matrice;

};

#endif /* !SYSTEMSTATEMATRICE_H_ */
