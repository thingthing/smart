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
void		Slam::updateState(Agent const &agent)
{
  //Update state using odometry
  this->_state->updateRobotState(agent);
  //@TODO: update jacobian matrice
  //@TODO: update process noise matrice
  this->_covariance->setRobotPosition(agent);
  this->_covariance->calculationCovariance();


}
// void		Slam::addLandmarks(pcl::PointXYZ cameradata[], int numberSample)
// {

// }
