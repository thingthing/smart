#include "test_slam_common.hh"

/**
* Landmark expected test result
**/
const int	Landmark_Result::id = -1;
const int	Landmark_Result::life = Landmarks::LIFE;
const int	Landmark_Result::totalTimesObserved = 0;
const double	Landmark_Result::range = -1;
const double	Landmark_Result::bearing = -1;
const double	Landmark_Result::pos[2] = {0.0, 0.0};


/**
 * Landmarks expected test result
 **/
const int			Landmarks_Result::DBSize = 0;
const int			Landmarks_Result::EKFLandmarks = 0;
const double			Landmarks_Result::defaultdegreePerScan = Agent::DEGREESPERSCAN;
const double			Landmarks_Result::degreePerScan = 0.42;
const int			Landmarks_Result::sizeIDtoID = Landmarks::MAXLANDMARKS;
const int			Landmarks_Result::sizelandmarkDB = Landmarks::MAXLANDMARKS;
const std::pair<int, int>	Landmarks_Result::goodSlamId = std::make_pair(42,84);
const std::pair<int, int>	Landmarks_Result::wrongSlamId = std::make_pair(13,12);

void	TestSlamCommon::generateData(pcl::PointCloud<pcl::PointXYZ> &cloud, int numberSample)
{
  cloud.width    = numberSample;
  cloud.height   = 1;
  cloud.is_dense = false;
  cloud.points.resize (cloud.width * cloud.height);
  for (unsigned int i = 0; i < cloud.points.size(); ++i)
    {
      cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0);
      cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0);
      if(i % 2 == 0)
	cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0);
      else
	cloud.points[i].z = -1 * (cloud.points[i].x + cloud.points[i].y);
    }
}
