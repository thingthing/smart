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
void		Slam::updateState(pcl::PointCloud<pcl::PointXYZ> const &cloud, Agent &agent)
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
  this->updateStateWithLandmark(cloud, agent, newLandmarks, reobservedLandmarks);
  this->addLandmarks(newLandmarks);

  for (std::vector<Landmarks::Landmark *>::iterator it = reobservedLandmarks.begin(); it != reobservedLandmarks.end(); ++it)
  {
    //@TODO: Caculate kalman gain and uncertainity
    //@TODO: Update state using kalman gain and uncertainity
  }
  
  agent.setPos(this->_state->getRobotPos());
  //After all, remove abd landmarks
  this->_landmarkDb->removeBadLandmarks(cloud, agent);
}

/**
 * Search for reobserved landmark and update state with them
 **/
void		Slam::updateStateWithLandmark(pcl::PointCloud<pcl::PointXYZ> const &cloud, Agent const &agent, std::vector<Landmarks::Landmark *> &newLandmarks, std::vector<Landmarks::Landmark *> &reobservedLandmarks)
{
  //@TODO: Function that associate without adding new landmark, and return the vector with only new landmark (to be used after)
  //@See alignLandmarkData
  this->_data->validationGate(cloud, agent, newLandmarks, reobservedLandmarks);
}

/**
 * Add landmark to matrice
 **/
void		Slam::addLandmarks(std::vector<Landmarks::Landmark *> const &newLandmarks)
{
  for (std::vector<Landmarks::Landmark *>::const_iterator it = newLandmarks.begin(); it != newLandmarks.end(); ++it)
  {
    int landmarkId = this->_landmarkDb->addToDB(**it);
    int slamId = this->_state->addLandmarkPosition((*it)->pos);
    this->_landmarkDb->addSlamId(landmarkId, slamId);

    this->_covariance->addLandmark((*it)->pos, slamId);
    this->_covariance->calculationCovariance();
  }
}
