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
#include "Agent.hh"

class   Slam
{
public:
  /**
  * @brief SLAM constructor
  * @details Take an agent wich will be link to the SLAM algorithm and
  * initialize all SLAM matrices with default values
  *
  * @param agent Agent on wich the API is installed
  */
  Slam(Agent *agent);
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
  void    updateState(pcl::PointCloud<pcl::PointXYZ> const &cloud, Agent &agent);

  /**
  * @brief Add a landmark
  * @details Function to ass a landmark to SLAM matrices
  *
  * @param newLandmarks New landmark extracted from the current mapping
  */
  void    addLandmarks(std::vector<Landmarks::Landmark *> const &newLandmarks);

private:
  Slam();

private:
  Agent     *_agent;
  Landmarks   *_landmarkDb;
  DataAssociation *_data;
  KalmanGainMatrice _kg;
  SystemStateMatrice  *_state;
  CovarianceMatrice *_covariance;
};


#endif    /*! _SLAM_HH_ */
