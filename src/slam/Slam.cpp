#include "Slam.hh"

Slam::Slam(IAgent *agent)
{
  this->_agent = agent;
  this->_landmarkDb = new Landmarks(agent->degreePerScan);
  this->_data = new DataAssociation(this->_landmarkDb);
  this->_test = new Test();
}

Slam::~Slam()
{
  if (this->_data)
    delete this->_data;
  if (this->_landmarkDb)
    delete this->_landmarkDb;
}

void    Slam::updateState(pcl::PointCloud<pcl::PointXYZRGBA> const &cloud, IAgent *agent)
{
  /*//Update state using odometry*/
  //  std::cout << "SLAM updateState" << std::endl;
  //Update state using reobserved landmark
  std::vector<Landmarks::Landmark *> newLandmarks;
  std::vector<Landmarks::Landmark *> reobservedLandmarks;
  //std::cout << "Before validationGate" << std::endl;
  try {
    this->_data->validationGate(cloud, agent, newLandmarks, reobservedLandmarks);
  } catch (...) {
    std::cerr << "Error during dataassociation" << std::endl;
  }
  //std::cout << "Before add landmarks" << std::endl;
  try {
    this->addLandmarks(newLandmarks);
  } catch (...) {
    std::cerr << "Error during addlandmarks" << std::endl;
  }
    //std::cout << "After add landmarks" << std::endl;

  // this->dispatch("SendNewLandmarkEvent", newLandmarks);

  /*//update the covariance for the agent*/
  this->_test->moveAgent(agent);

  /*//calculation of Kalman gain.*/
  this->_test->updatePositions(0.0); //this shit should do the shit

  agent->setPos(this->_test->getNewRobotPos());

  //After all, remove bad landmarks
  //this->_landmarkDb->removeBadLandmarks(cloud, agent);
}

void    Slam::addLandmarks(std::vector<Landmarks::Landmark *> const &newLandmarks)
{
  for (std::vector<Landmarks::Landmark *>::const_iterator it = newLandmarks.begin(); it != newLandmarks.end(); ++it)
  {
    int landmarkId = this->_landmarkDb->addToDB(**it);
    int slamId = (int)this->_test->addLandmark((*it)->pos);
    this->_landmarkDb->addSlamId(landmarkId, slamId);

    //By default assume that landmark is perfectly observed
  }
}
