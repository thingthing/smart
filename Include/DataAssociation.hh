#ifndef		_DATA_ASSOCIATION_HH_
# define	_DATA_ASSOCIATION_HH_

#include "Landmarks.hh"

class DataAssociation
{
public:
  DataAssociation();
  DataAssociation(Landmarks *landmarkDb);
  ~DataAssociation();

  void	validationGate(double cameradata[], int numberSample, double robotPosition[]);
  bool	associateLandmarks(Landmarks::Landmark *toAssociate) const;

  Landmarks	*getLandmarkDb() const;

private:
#ifdef UNITTEST
public: // ONLY FOR UNIT TESTS
#endif
  Landmarks	*_landmarkDb;

};

#endif		/* !_DATA_ASSOCIATION_HH_ */
