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
void		DataAssociation::validationGate(pcl::PointCloud<pcl::PointXYZ> const &cloud, Agent const &agent, std::vector<Landmarks::Landmark *> &resultLandmarks, std::vector<Landmarks::Landmark *> &reobservedLandmarks)
{
  std::vector<Landmarks::Landmark *>	newLandmarks;

  newLandmarks = this->_landmarkDb->extractLineLandmarks(cloud, agent);
  for (std::vector<Landmarks::Landmark *>::iterator it = newLandmarks.begin(); it != newLandmarks.end(); ++it)
    {
      //First associate landmark
      this->associateLandmarks(*it);
    }
  //Can't remove Doubles because not really associated yet
  //@TODO: Find where we can removeDouble
  //newLandmarks = this->_landmarkDb->removeDouble(newLandmarks, resultLandmarks);
  //Pass non doubles through gate
  for (std::vector<Landmarks::Landmark *>::iterator it = newLandmarks.begin(); it != newLandmarks.end(); ++it)
    {
      if (this->_landmarkDb->getAssociation(*(*it)) == -1)
  {
    //Landmark not found, should add it
    resultLandmarks.push_back(*it);
    //this->_landmarkDb->addToDB(*(*it));
  }
      else
  reobservedLandmarks.push_back(*it);
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
