#ifndef		_DATA_ASSOCIATION_HH_
# define	_DATA_ASSOCIATION_HH_

#include "Landmarks.hh"
#include "Agent.hh"

class DataAssociation
{
public:
  DataAssociation();
  DataAssociation(Landmarks *landmarkDb);
  ~DataAssociation();

  void validationGate(pcl::PointXYZ cameradata[], int numberSample, Agent const &agent, std::vector<Landmarks::Landmark *> &resultLandmarks, std::vector<Landmarks::Landmark *> &reobservedLandmarks);
  bool	associateLandmarks(Landmarks::Landmark *toAssociate) const;

  Landmarks	*getLandmarkDb() const;

private:
#ifdef UNITTEST
public: // ONLY FOR UNIT TESTS
#endif
  Landmarks	*_landmarkDb;

};

#endif		/* !_DATA_ASSOCIATION_HH_ */
