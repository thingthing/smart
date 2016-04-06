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


Landmarks *DataAssociation::getLandmarkDb() const
{
  return (this->_landmarkDb);
}

void    DataAssociation::validationGate(pcl::PointCloud<pcl::PointXYZRGBA> const &cloud,
                                        IAgent const *agent,
                                        std::vector<Landmarks::Landmark *> &resultLandmarks,
                                        std::vector<Landmarks::Landmark *> &reobservedLandmarks)
{
  std::vector<Landmarks::Landmark *>  newLandmarks;

  try {
    if (!cloud.empty())
      newLandmarks = this->_landmarkDb->extractLineLandmarks(cloud, agent);
  } catch (...) {
    std::cerr << "Error during extractlineLandmarks " << this->_landmarkDb->getLandmarkDB().size() << std::endl;
    return;
  }
  std::cerr << "extractlineLandmarks end" << std::endl;

  try {
    for (std::vector<Landmarks::Landmark *>::iterator it = newLandmarks.begin(); it != newLandmarks.end(); ++it)
      {
	      //First associate landmark
        this->associateLandmarks(*it);
        //Can't remove Doubles because not really associated yet
        ///@todo: Find where we can removeDouble
        //newLandmarks = this->_landmarkDb->removeDouble(newLandmarks, resultLandmarks);

        //Pass non doubles through gate       
        if (this->_landmarkDb->getAssociation(*(*it)) == -1)
        {
          //Landmark not found, should add it
          resultLandmarks.push_back(*it);
          //this->_landmarkDb->addToDB(*(*it));
        }
        else
          reobservedLandmarks.push_back(*it);
        
      }
  } catch (...) {
    std::cerr << "Error during associatelandmarks" << std::endl;
  }

  // try {
  //   //Pass non doubles through gate
  //   for (std::vector<Landmarks::Landmark *>::iterator it = newLandmarks.begin(); it != newLandmarks.end(); ++it)
  //     {
	
  // } catch (...) {
  //   std::cerr << "Error during landmarkassociationresult" << std::endl;
  // }
}

bool  DataAssociation::associateLandmarks(Landmarks::Landmark *toAssociate) const
{
  int id = -1;
  int totalTimeObserved = -1;

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
