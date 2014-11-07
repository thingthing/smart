#ifndef		_TEST_DATA_HH_
# define	_TEST_DATA_HH_

#include "Landmarks.hh"

/**
 * Landmark expected test result
 **/
namespace	Landmark_Result
{
  int		id = -1;
  int		life = LIFE;
  int		totalTimesObserved = 0;
  double	range = -1;
  double	bearing = -1;
  double	pos[2] = {0.0, 0.0};
};

/**
 * Landmarks expected test result
 **/
namespace	Landmarks_Result
{
  int		DBSize = 0;
  int		EKFLandmarks = 0;
  double	defaultdegreePerScan = DEGREESPERSCAN;
  double	degreePerScan = 0.42; 
  int		sizeIDtoID = MAXLANDMARKS;
  int		sizelandmarkDB = MAXLANDMARKS;
  std::pair<int, int>		goodSlamId = std::make_pair(42,42);
  std::pair<int, int>		wrongSlamId = std::make_pair(13,12);
};

#endif		/* !_TEST_DATA_HH_ */
