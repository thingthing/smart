/** @defgroup SLAM Slam files
 *
 * @brief Files use for SLAM algorithm
 */

/**
 * @class Slam
 *
 * @ingroup SLAM
 *
 * @brief Main Slam algorithm class
 *
 * This class is meant as a entry point to Slam algorithm for an Agent
 *
 * @author Nicolas
 *
 * @version 1.0
 *
 * @date 28/04/2015
 *
 */

#ifndef   _SLAM_HH_
# define  _SLAM_HH_

#include <pcl/common/common.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/common/projection_matrix.h>
#include "DataAssociation.hh"
#include "KalmanGainMatrice.hh"
#include "SystemStateMatrice.hh"
#include "CovarianceMatrice.hh"
#include "Landmarks.hh"
#include "JacobianMatriceA.hh"
#include "JacobianMatriceJxr.hh"
#include "JacobianMatriceJz.hh"
#include "JacobianMatriceH.hh"
#include "IAgent.hh"
#include "event/Dispatcher.h"

#include "test.hh"

class   Slam : public Utils::Dispatcher
{
public:
  /**
  * @brief SLAM constructor
  * @details Take an agent wich will be link to the SLAM algorithm and
  * initialize all SLAM matrices with default values
  *
  * @param agent Agent on wich the API is installed
  */
  Slam(IAgent *agent);
  ~Slam();

  /**
   * @brief Update agent state
   * @details Take the current mapping and the current agent state
   * (upated with the odometry), apply SLAM algorithm and update agent state
   * with new values
   * @warning Must be used after agent update odometry
   *
   * @param cloud Current mapping
   * @param agent Agent we want the state updated
   */
  void    updateState(pcl::PointCloud<pcl::PointXYZ> const &cloud, IAgent *agent);

  /**
  * @brief Add a landmark
  * @details Function to ass a landmark to SLAM matrices
  *
  * @param newLandmarks New landmark extracted from the current mapping
  */
  void    addLandmarks(std::vector<Landmarks::Landmark *> const &newLandmarks, IAgent *agent);

private:
  Slam();

private:
  IAgent     *_agent;
  DataAssociation *_data;
  KalmanGainMatrice *_kg;

public:
  SystemStateMatrice  *_state;
  Landmarks   *_landmarkDb;
	JacobianMatriceA *_jA;
  JacobianMatriceJxr *_jXR;
  JacobianMatriceJz *_jZ;
	JacobianMatriceH *_jH;
  CovarianceMatrice *_covariance;
	Test *_test;
};


#endif    /*! _SLAM_HH_ */
