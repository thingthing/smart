#include "Slam.hh"

Slam::Case::Case() :
	state(UPTODATE)
{
	this->currentPosition.x = 0.0;
	this->currentPosition.y = 0.0;
	this->currentPosition.z = 0.0;

	this->oldPosition.x = 0.0;
	this->oldPosition.y = 0.0;
	this->oldPosition.z = 0.0;
}

Slam::Case::Case(const pcl::PointXYZ &landmark) :
	oldPosition(landmark), currentPosition(landmark), state(UPTODATE)
{
}

Slam::Case::Case(float x, float y, float z) :
	state(UPTODATE)
{
	this->currentPosition.x = x;
	this->currentPosition.y = y;
	this->currentPosition.z = z;

	this->oldPosition.x = x;
	this->oldPosition.y = y;
	this->oldPosition.z = z;
}

Slam::Case::~Case()
{
}

Slam::State Slam::Case::getState() const
{
	return (state);
}

void Slam::Case::setState(Slam::State _state)
{
	this->state = _state;
}

pcl::PointXYZ Slam::Case::getOldPosition() const
{
	return (this->oldPosition);
}

void Slam::Case::setOldPosition(pcl::PointXYZ landmark)
{
	this->oldPosition = landmark;
}

pcl::PointXYZ Slam::Case::getCurrentPosition() const
{
	return (this->currentPosition);
}

void Slam::Case::setCurrentPosition(pcl::PointXYZ landmark)
{
	this->currentPosition = landmark;
}

void Slam::Case::setCurrentPosition(float x, float y, float z)
{
	this->currentPosition.x = x;
	this->currentPosition.y = y;
	this->currentPosition.z = z;
}

Slam::Slam(IAgent *agent)
{
  this->_agent = agent;
	this->currentRobotPos = agent->getPos();
	this->oldRobotPos = agent->getPos();
  this->_landmarkDb = new Landmarks(agent->degreePerScan);
  this->_data = new DataAssociation(this->_landmarkDb);
	this->landmarkNumber = 0;
	_agent->registerCallback("getDataEvent", [this](pcl::PointCloud<pcl::PointXYZRGBA> const & cloud, IAgent *agent) {updateState(cloud, agent);});
}

Slam::~Slam()
{
  if (this->_data)
    delete this->_data;
  if (this->_landmarkDb)
    delete this->_landmarkDb;
  this->matrix.clear();
}

void Slam::moveAgent(IAgent const *agent)
{
	this->oldRobotPos = this->currentRobotPos;
	this->currentRobotPos = agent->getPos();
}

//trustPercentageOnRobotMovement must be between 0 and 1.
//0 means trust the landmarks; 1 means trust the agent's odometry
bool Slam::updatePositions(int trustPercentageOnRobotMovement, std::vector<Landmarks::Landmark *> &reobservedLandmarks)
{
float averageLandmarkMovementX = 0;
float averageLandmarkMovementY = 0;
float averageLandmarkMovementZ = 0;
int landmarksMoved = 0;

float supposedRobotDisplacementX = 0;
float supposedRobotDisplacementY = 0;
float supposedRobotDisplacementZ = 0;

float actualRobotDisplacementX = 0;
float actualRobotDisplacementY = 0;
float actualRobotDisplacementZ = 0;
bool data_moved = false;

	for (std::map<unsigned int, Case>::iterator it=matrix.begin(); it!=matrix.end(); ++it)
	{
		if (it->second.getState() == MOVED)
			{
				averageLandmarkMovementX += it->second.getCurrentPosition().x - it->second.getOldPosition().x;
				averageLandmarkMovementY += it->second.getCurrentPosition().y - it->second.getOldPosition().y;
				averageLandmarkMovementZ += it->second.getCurrentPosition().z - it->second.getOldPosition().z;

				landmarksMoved++;
				it->second.setState(UPDATING);
				data_moved = true;
			}
	}
std::cout << "landmark moved == " << landmarksMoved << std::endl;
	if (landmarksMoved > 0)
	{
		averageLandmarkMovementX /= landmarksMoved; 
		averageLandmarkMovementY /= landmarksMoved;
		averageLandmarkMovementZ /= landmarksMoved;
	}

	std::cout << "averageLandmarkMovementx  :: " << averageLandmarkMovementX << " -- averageLandmarkMovementY :: " << averageLandmarkMovementY << " -- averageLandmarkMovementZ :: " << averageLandmarkMovementZ << std::endl;

	supposedRobotDisplacementX = this->currentRobotPos.x - this->oldRobotPos.x;
	supposedRobotDisplacementY = this->currentRobotPos.y - this->oldRobotPos.y;
	supposedRobotDisplacementZ = this->currentRobotPos.z - this->oldRobotPos.z;

	actualRobotDisplacementX = (averageLandmarkMovementX * (1 - trustPercentageOnRobotMovement) + supposedRobotDisplacementX * trustPercentageOnRobotMovement);
	actualRobotDisplacementY = (averageLandmarkMovementY * (1 - trustPercentageOnRobotMovement) + supposedRobotDisplacementY * trustPercentageOnRobotMovement);
	actualRobotDisplacementZ = (averageLandmarkMovementZ * (1 - trustPercentageOnRobotMovement) + supposedRobotDisplacementZ * trustPercentageOnRobotMovement);
	
	actualRobotDisplacementX = std::nearbyint(actualRobotDisplacementX * 100) / 100;
	actualRobotDisplacementY = std::nearbyint(actualRobotDisplacementY * 100) / 100;
	actualRobotDisplacementZ = std::nearbyint(actualRobotDisplacementZ * 100) / 100;
  
  std::cout << "Old position x :: " << this->currentRobotPos.x << " -- Old position y :: " << this->currentRobotPos.y << " -- Old position z :: " << this->currentRobotPos.z << std::endl;
	this->currentRobotPos.x = this->oldRobotPos.x + actualRobotDisplacementX;
	this->currentRobotPos.y = this->oldRobotPos.y + actualRobotDisplacementY;
	this->currentRobotPos.z = this->oldRobotPos.z + actualRobotDisplacementZ;
  std::cout << "New position x :: " << this->currentRobotPos.x << " -- New position y :: " << this->currentRobotPos.y << " -- New position z :: " << this->currentRobotPos.z << std::endl;

  		// Round to 0.001 decimal
	this->currentRobotPos.x = std::nearbyint(this->currentRobotPos.x * 100) / 100;
	this->currentRobotPos.y = std::nearbyint(this->currentRobotPos.y * 100) / 100;
	this->currentRobotPos.z = std::nearbyint(this->currentRobotPos.z * 100) / 100;
	std::cout << "New position after round x :: " << this->currentRobotPos.x << " -- New position after  round y :: " << this->currentRobotPos.y << " -- New position after  round  z :: " << this->currentRobotPos.z << std::endl;

  for (std::vector<Landmarks::Landmark *>::const_iterator it = reobservedLandmarks.begin(); it != reobservedLandmarks.end(); ++it)
  {
  	int slamId = this->_landmarkDb->getSLamId((*it)->id);
  	if (this->matrix.at(slamId).getState() == UPDATING)
	{
		this->matrix.at(slamId).setCurrentPosition(this->matrix.at(slamId).getOldPosition().x + actualRobotDisplacementX, this->matrix.at(slamId).getOldPosition().y + actualRobotDisplacementY, this->matrix.at(slamId).getOldPosition().z + actualRobotDisplacementZ);
		this->matrix.at(slamId).setState(UPTODATE);
		(*it)->pos.x = this->matrix.at(slamId).getCurrentPosition().x;
		(*it)->pos.y = this->matrix.at(slamId).getCurrentPosition().y;
		(*it)->pos.z = this->matrix.at(slamId).getCurrentPosition().z;
	}
  }
	// for (std::map<unsigned int, Case>::iterator it=matrix.begin(); it!=matrix.end(); ++it)
	// {
	// 	if (it->second.getState() == UPDATING)
	// 	{
	// 		it->second.setCurrentPosition(it->second.getOldPosition().x + actualRobotDisplacementX, it->second.getOldPosition().y + actualRobotDisplacementY, it->second.getOldPosition().z + actualRobotDisplacementZ);
	// 		it->second.setState(UPTODATE);
	// 	}
	// }
	return data_moved;
}

void    Slam::updateState(pcl::PointCloud<pcl::PointXYZRGBA> const &cloud, IAgent *agent)
{
  //Update state using reobserved landmark
  std::vector<Landmarks::Landmark *> newLandmarks;
  std::vector<Landmarks::Landmark *> reobservedLandmarks;

  if (cloud.size() == 0)
  	return;
  
  try {
    this->_data->validationGate(cloud, agent, newLandmarks, reobservedLandmarks);
  } catch (...) {
    std::cerr << "Error during dataassociation" << std::endl;
  }
  try {
    this->addLandmarks(newLandmarks);
  } catch (...) {
    std::cerr << "Error during addlandmarks" << std::endl;
  }
std::cerr << "total landmarks length is " << this->_landmarkDb->getDBSize() << std::endl;
newLandmarks.clear();
  	std::cout << "reobserved landmarks before length is " << reobservedLandmarks.size() << std::endl;
  reobservedLandmarks = this->_landmarkDb->removeDouble(reobservedLandmarks, newLandmarks);
  try {
  	std::cout << "reobserved landmarks after length is " << reobservedLandmarks.size() << std::endl;
  	this->moveLandmarks(reobservedLandmarks);
  } catch (std::exception &e) {
    std::cerr << "error during move landmarks " << e.what() << std::endl;
  }
	this->moveAgent(agent);

	bool data_moved = this->updatePositions(0.0, reobservedLandmarks);
	if (data_moved && !agent->getSendData()) {
		agent->setSendData(data_moved);
	}
  agent->setPos(this->currentRobotPos);
  //After all, remove bad landmarks
  this->_landmarkDb->removeBadLandmarks(cloud, agent);
}

void    Slam::addLandmarks(std::vector<Landmarks::Landmark *> const &newLandmarks)
{
  for (std::vector<Landmarks::Landmark *>::const_iterator it = newLandmarks.begin(); it != newLandmarks.end(); ++it)
  {
    int landmarkId = this->_landmarkDb->addToDB(**it);
    if (landmarkId != -1) {
    	int slamId = (int)this->addLandmarkToMatrix((*it)->pos);
    	this->_landmarkDb->addSlamId(landmarkId, slamId);
    }
  }
}

unsigned int Slam::addLandmarkToMatrix(const pcl::PointXYZ &position)
{
	float tempX, tempXX, tempY, tempYY, tempZ, tempZZ;

	tempX = position.x * cos(this->_agent->getYaw()) - position.y * sin(this->_agent->getYaw());
	tempY = position.x * sin(this->_agent->getYaw()) + position.y * cos(this->_agent->getYaw());

	tempXX = tempX * cos(this->_agent->getPitch()) - position.z * sin(this->_agent->getPitch());
	tempZ = tempX * sin(this->_agent->getPitch()) + position.z * cos(this->_agent->getPitch());

	tempYY = tempY * cos(this->_agent->getYaw()) - tempZ * sin(this->_agent->getYaw());
	tempZZ = tempY * sin(this->_agent->getYaw()) + tempZ * cos(this->_agent->getYaw());

	Slam::Case tempCase = Slam::Case(tempXX, tempYY, tempZZ);

	this->matrix[this->landmarkNumber] = tempCase;

	return(this->landmarkNumber++);
}

void Slam::moveLandmarks(std::vector<Landmarks::Landmark *> const &reobservedLandmarks)
{
  for (std::vector<Landmarks::Landmark *>::const_iterator it = reobservedLandmarks.begin(); it != reobservedLandmarks.end(); ++it)
    this->moveLandmark(*it);
}

void Slam::moveLandmark(Landmarks::Landmark *landmark)
{
	float tempX, tempXX, tempY, tempYY, tempZ, tempZZ;

	tempX = landmark->pos.x * cos(this->_agent->getYaw()) - landmark->pos.y * sin(this->_agent->getYaw());
	tempY = landmark->pos.x * sin(this->_agent->getYaw()) + landmark->pos.y * cos(this->_agent->getYaw());

	tempXX = tempX * cos(this->_agent->getPitch()) - landmark->pos.z * sin(this->_agent->getPitch());
	tempZ = tempX * sin(this->_agent->getPitch()) + landmark->pos.z * cos(this->_agent->getPitch());

	tempYY = tempY * cos(this->_agent->getRoll()) - tempZ * sin(this->_agent->getRoll());
	tempZZ = tempY * sin(this->_agent->getRoll()) + tempZ * cos(this->_agent->getRoll());

  int slamId = this->_landmarkDb->getSLamId(landmark->id);

  	//std::cerr << "MOVING LANDMAKR " << slamId << " landmark == " << landmark->id << " Roll == " << this->_agent->getRoll() << " -- pitch == " << this->_agent->getPitch() << " -- yaw == " << this->_agent->getYaw() << std::endl;
  	//std::cerr << "PREV POS == x: " << landmark->pos.x << " -- y: " << landmark->pos.y << " -- z: " << landmark->pos.z << std::endl;
	this->matrix.at(slamId).setOldPosition(this->matrix.at(slamId).getCurrentPosition());
	this->matrix.at(slamId).setCurrentPosition(pcl::PointXYZ(tempXX, tempYY, tempZZ));
  	//std::cerr << "NEW POS DISPLACEMENT == x: " << tempXX << " -- y: " << tempYY << " -- z: " << tempZZ << std::endl;
  	if (landmark->totalTimeObserved > Landmarks::MINOBSERVATIONS) {
  		//std::cerr << "MOVING " << slamId << std::endl;
		this->matrix.at(slamId).setState(MOVED); // Observed enough so moving agent with it
  	}
	else
		this->matrix.at(slamId).setState(UPDATING); // Not obeserved enough, don't user it to move
}
