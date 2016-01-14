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
 * @author Nicolas, Martin
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
#include "Landmarks.hh"
#include "IAgent.hh"
#include "event/Dispatcher.h"

#include <map>
#include <cmath>
#include <iostream>

class   Slam : public Utils::Dispatcher
{
public:
  	enum State
	{
		UPDATING,
		MOVED,
		UPTODATE
	};

	class Case
	{
	public:
		Case();
		Case(const pcl::PointXYZ &landmark);
		Case(float x, float y, float z);
		virtual ~Case();
		State getState() const;
    void setState(State _state);
		pcl::PointXYZ getOldPosition() const;
		void setOldPosition(pcl::PointXYZ landmark);
		pcl::PointXYZ getCurrentPosition() const;
		void setCurrentPosition(pcl::PointXYZ landmark);
		void setCurrentPosition(float x, float y);

	protected:
		pcl::PointXYZ oldPosition;
		pcl::PointXYZ currentPosition;
		State state;
	};

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
  void    updateState(pcl::PointCloud<pcl::PointXYZRGBA> const &cloud, IAgent *agent);

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
  IAgent     *_agent;
  DataAssociation *_data;

public:
	unsigned int addLandmarkToMatrix(const pcl::PointXYZ &position);
	void moveLandmark(unsigned int landmarkNumber, const pcl::PointXYZ &position);
	void moveAgent(IAgent const *agent);
	void updatePositions(int trustPercentageOnRobotMovement);

  Landmarks   *_landmarkDb;

	pcl::PointXYZ oldRobotPos;
	pcl::PointXYZ currentRobotPos;
	std::map<unsigned int, Case> matrix;
	unsigned int landmarkNumber;

};


#endif    /*! _SLAM_HH_ */
