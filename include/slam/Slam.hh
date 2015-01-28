#ifndef		_SLAM_HH_
# define	_SLAM_HH_

#include <pcl/common/common.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/common/projection_matrix.h>
#include "DataAssociation.hh"
#include "KalmanGainMatrice.hh"
#include "SystemStateMatrice.hh"
#include "CovarianceMatrice.hh"
#include "Landmarks.hh"
#include "Agent.hh"

class		Slam
{
public:
  Slam(Agent *agent);
  ~Slam();

  void		updateState(pcl::PointCloud<pcl::PointXYZ> const &cloud, Agent const &agent);
  void		addLandmarks(pcl::PointCloud<pcl::PointXYZ> const &cloud);

private:
  void		updateStateWithLandmark(pcl::PointCloud<pcl::PointXYZ> const &cloud, Agent const &agent, std::vector<Landmarks::Landmark *> &newLandmarks, std::vector<Landmarks::Landmark *> &reobservedLandmarks);
  Slam();

private:
  Agent			*_agent;
  Landmarks		*_landmarkDb;
  DataAssociation	*_data;
  KalmanGainMatrice	_kg;
  SystemStateMatrice	*_state;
  CovarianceMatrice	*_covariance;
};


#endif		/*! _SLAM_HH_ */
