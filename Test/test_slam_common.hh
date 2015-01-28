#ifndef		_TEST_SLAM_COMMON_HH_
# define	_TEST_SLAM_COMMON_HH_

#include "Landmarks.hh"
#include "Agent.hh"

#define AssertThatDetail(X,Y) Assert::That(X,Y,__FILE__, __LINE__)

/**
 * Landmark expected test result
 **/
namespace	Landmark_Result
{
extern const int		id;
extern const int		life;
extern const int		totalTimesObserved;
extern const double		range;
extern const double		bearing;
extern const double		pos[2];
};

/**
 * Landmarks expected test result
 **/
namespace	Landmarks_Result
{
extern const int		DBSize;
extern const int		EKFLandmarks;
extern const double		defaultdegreePerScan;
extern const double		degreePerScan;
extern const int		sizeIDtoID;
extern const int		sizelandmarkDB;
extern const std::pair<int, int> goodSlamId;
extern const std::pair<int, int> wrongSlamId;
};

namespace	TestSlamCommon
{
void	generateData(pcl::PointCloud<pcl::PointXYZ> &cloud, int numberSample);
}

#endif		/* !_TEST_SLAM_COMMON_HH_ */
