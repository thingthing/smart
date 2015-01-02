#ifndef		_SLAM_HH_
# define	_SLAM_HH_

#include <pcl-1.7/pcl/common/common.h>
#include <pcl-1.7/pcl/impl/point_types.hpp>
#include <pcl-1.7/pcl/common/projection_matrix.h>
#include "DataAssociation.hh"
#include "KalmanGainMatrice.hh"
#include "SystemStateMatrice.hh"
#include "CovarianceMatrice.hh"
#include "Agent.hh"

class		Slam
{
public:
  Slam(Agent *agent);
  ~Slam();

  void		updateState(pcl::PointXYZ cameradata[], int numberSample, Agent *agent);
  void		addLandmarks(pcl::PointXYZ cameradata[], int numberSample);

private:
  Slam();

private:
  Agent			*_agent;
  DataAssociation	*_data;
  KalmanGainMatrice	*_kg;
  SystemStateMatrice	*_state;
  CovarianceMatrice	*_covariance;
};


#endif		/*! _SLAM_HH_ */
