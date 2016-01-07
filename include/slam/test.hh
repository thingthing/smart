#ifndef   _TEST_HH_
# define  _TEST_HH_

#include <pcl/common/common.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/common/projection_matrix.h>
#include "DataAssociation.hh"
#include "IAgent.hh"
#include "event/Dispatcher.h"

# include <map>
# include <cmath>
#include <iostream>

class   Test
{
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

public:
	Test();
	Test(pcl::PointXYZ const &posRobot, float Theta);
	Test(float X, float Y, float z, float Theta);
	Test(IAgent const *agent);
	virtual ~Test();

	unsigned int addLandmark(const pcl::PointXYZ &position);
	unsigned int addLandmark(float x, float y, float z);
	void moveLandmark(unsigned int landmarkNumber, const pcl::PointXYZ &position);
	void moveLandmark(unsigned int landmarkNumber, float x, float y, float z);
	void moveAgent(pcl::PointXYZ const &posRobot, float Theta);
	void moveAgent(float X, float Y, float Z, float Theta);
	void moveAgent(IAgent const *agent);
	void updatePositions(int trustPercentageOnRobotMovement);

protected:
	pcl::PointXYZ oldRobotPos;
	pcl::PointXYZ currentRobotPos;
	float robotTheta;
	std::map<unsigned int, Case> matrix;
	unsigned int landmarkNumber;
};

#endif /* !_TEST_HH_ */
