#include "Slam.hh"

Slam::Slam(IAgent *agent)
{
  this->_agent = agent;
  this->_landmarkDb = new Landmarks(agent->degreePerScan);
  this->_data = new DataAssociation(this->_landmarkDb);
  this->_state = new SystemStateMatrice(agent);
  this->_covariance = new CovarianceMatrice(agent);
	this->_jA = new JacobianMatriceA();
	this->_jXR = new JacobianMatriceJxr();
	this->_jZ = new JacobianMatriceJz();
	this->_jH = new JacobianMatriceH();
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
	if (this->_jA)
		delete this->_jA;
	if (this->_jXR)
		delete this->_jXR;
	if (this->_jZ)
		delete this->_jZ;
	if (this->_jH)
		delete this->_jH;
}

void    Slam::updateState(pcl::PointCloud<pcl::PointXYZ> const &cloud, IAgent *agent)
{
  //Update state using odometry
  this->_state->setRobotState(agent);
  this->_jA->JacobiMath(agent);
  ///@todo: update process noise matrice


  //Update state using reobserved landmark
  std::vector<Landmarks::Landmark *> newLandmarks;
  std::vector<Landmarks::Landmark *> reobservedLandmarks;
  this->_data->validationGate(cloud, agent, newLandmarks, reobservedLandmarks);
  this->addLandmarks(newLandmarks, agent);
this->dispatch("SendCloudEvent", cloud);
  this->dispatch("SendNewLandmarkEvent", newLandmarks);

	//update the covariance for the agent
  this->_covariance->setRobotPosition(agent);
	this->_covariance->step1RobotCovariance(*_jA);

 /* for (std::vector<Landmarks::Landmark *>::iterator it = reobservedLandmarks.begin(); it != reobservedLandmarks.end(); ++it)
  {
    ///@todo: Caculate kalman gain and uncertainity
    ///@todo: Update state using kalman gain and uncertainity
  }*/

	//calculation of Kalman gain.
	this->_kg.updateLandmark(this->_jH, this->_covariance);

  agent.setPos(this->_state->getRobotPos());
  //After all, remove bad landmarks
  this->_landmarkDb->removeBadLandmarks(cloud, agent);
}

void    Slam::addLandmarks(std::vector<Landmarks::Landmark *> const &newLandmarks, IAgent &agent)
{
	this->_jXR->JacobiMath(agent);
	this->_jZ->JacobiMath(agent);
  for (std::vector<Landmarks::Landmark *>::const_iterator it = newLandmarks.begin(); it != newLandmarks.end(); ++it)
  {
    int landmarkId = this->_landmarkDb->addToDB(**it);
    int slamId = (int)this->_state->addLandmarkPosition((*it)->pos);
    this->_landmarkDb->addSlamId(landmarkId, slamId);
    //By default assume that landmark is perfectly observed
    //this->_kg.addLandmark(std::make_pair(0.0, 0.0), std::make_pair(0.0, 0.0), slamId);

		//first add raw values to the covariance, then do step 3, new landmarks update.
		this->_covariance->addLandmark((*it)->pos, slamId);
		this->_covariance->step3Covariance(this->_jXR, this->_jZ, this->_state, slamId);
  }
}
