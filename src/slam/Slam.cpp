#include "Slam.hh"

Slam::Slam(Agent *agent)
{
  this->_agent = agent;
  this->_landmarkDb = new Landmarks(agent->degreePerScan);
  this->_data = new DataAssociation(this->_landmarkDb);
  this->_state = new SystemStateMatrice(*agent);
  this->_covariance = new CovarianceMatrice(agent->getPos().x, agent->getPos().y, agent->getBearing());
}

Slam::~Slam()
{
  if (this->_data)
    delete this->_data;
  if (this->_landmarkDb)
    delete this->_landmarkDb;
  if (this->_state)
    delete this->_state;
  if (this->_covariance)
    delete this->_covariance;
}

/**
 * To be used after agent update odometry
 **/
void		Slam::updateState(Agent const &agent, pcl::PointXYZ cameradata[], int numberSample)
{
  //Update state using odometry
  this->_state->updateRobotState(agent);
  //@TODO: update jacobian matrice
  //@TODO: update process noise matrice
  this->_covariance->setRobotPosition(agent);
  this->_covariance->calculationCovariance();

  //Update state using reobserved landmark
  std::vector<Landmarks::Landmark *> newLandmarks;
  std::vector<Landmarks::Landmark *> reobservedLandmarks;
  this->updateStateWithLandmark(agent, cameradata, numberSample, newLandmarks, reobservedLandmarks);
}

/**
 * Search for reobserved landmark and update state with them
 **/
void		Slam::updateStateWithLandmark(Agent const &agent, pcl::PointXYZ cameradata[], int numberSample, std::vector<Landmarks::Landmark *> &newLandmarks, std::vector<Landmarks::Landmark *> &reobservedLandmarks)
{
  //@TODO: Function that associate without adding new landmark, and return the vector with only new landmark (to be used after)
  this->_data->validationGate(cameradata, numberSample, agent, newLandmarks, reobservedLandmarks);
}

/**
 * @TODO: Add landmark to matrice
 **/
// void		Slam::addLandmarks(pcl::PointXYZ cameradata[], int numberSample)
// {
//   this->_data->validationGate(cameradata, numberSample, *this->_agent);
// }
