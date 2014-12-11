#include "DataAssociation.hh"

DataAssociation::DataAssociation()
{
  this->_landmarkDb = new Landmarks();
}

DataAssociation::DataAssociation(Landmarks *landmarkDb)
  : _landmarkDb(landmarkDb)
{
}

DataAssociation::~DataAssociation()
{

}

Landmarks	*DataAssociation::getLandmarkDb() const
{
  return (this->_landmarkDb);
}

//Will be called after each data gathering to check for new landmarks
void	DataAssociation::validationGate(double cameradata[], int numberSample,
					double robotPosition[])
{
  std::vector<Landmarks::Landmark *>	currentLandmarks;
  std::vector<Landmarks::Landmark *>	newLandmarks;

  newLandmarks = this->_landmarkDb->extractLineLandmarks(cameradata, numberSample, robotPosition);
  currentLandmarks = this->_landmarkDb->getLandmarkDB();

  for (std::vector<Landmarks::Landmark *>::iterator it = newLandmarks.begin(); it != newLandmarks.end(); ++it)
    {
      //First associate landmark
      this->associateLandmarks(*it);
      //Then pass through the gate
      if (this->_landmarkDb->getAssociation(*(*it)) == -1)
	{
	  //Landmark not found, should add it
	  this->_landmarkDb->addToDB(*(*it));
	}
    }
}

//Set id and timeObserved for landamark base on closer landmark in db
bool	DataAssociation::associateLandmarks(Landmarks::Landmark *toAssociate) const
{
  int	id = -1;
  int	totalTimeObserved = -1;

  this->_landmarkDb->getClosestAssociation(toAssociate, id, totalTimeObserved);
  if (id != -1 && totalTimeObserved != -1)
    {
      toAssociate->id = id;
      toAssociate->totalTimeObserved = totalTimeObserved;
      return (true);
    }
  //No landmark have enough views in Db
  return (false);
}
