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

void Slam::Case::setCurrentPosition(float x, float y)
{
	this->currentPosition.x = x;
	this->currentPosition.y = y;
}

Slam::Slam(IAgent *agent)
{
  this->_agent = agent;
	this->currentRobotPos = agent->getPos();
	this->oldRobotPos = agent->getPos();
  this->_landmarkDb = new Landmarks(agent->degreePerScan);
  this->_data = new DataAssociation(this->_landmarkDb);
	this->landmarkNumber = 0;
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
void Slam::updatePositions(int trustPercentageOnRobotMovement)
{
float averageLandmarkMovementX = 0;
float averageLandmarkMovementY = 0;
int landmarksMoved = 0;

float supposedRobotDisplacementX = 0;
float supposedRobotDisplacementY = 0;

float actualRobotDisplacementX = 0;
float actualRobotDisplacementY = 0;

	for (std::map<unsigned int, Case>::iterator it=matrix.begin(); it!=matrix.end(); ++it)
	{
		if (it->second.getState() == MOVED)
			{
				averageLandmarkMovementX += it->second.getCurrentPosition().x - it->second.getOldPosition().x;
				averageLandmarkMovementY += it->second.getCurrentPosition().y - it->second.getOldPosition().y;

				landmarksMoved++;
				it->second.setState(UPDATING);
			}
	}

	if (landmarksMoved > 0)
	{
		averageLandmarkMovementX /= landmarksMoved; 
		averageLandmarkMovementY /= landmarksMoved;
	}

	supposedRobotDisplacementX = this->currentRobotPos.x - this->oldRobotPos.x;
	supposedRobotDisplacementY = this->currentRobotPos.y - this->oldRobotPos.y;

	actualRobotDisplacementX = (averageLandmarkMovementX * (1 - trustPercentageOnRobotMovement) + supposedRobotDisplacementX * trustPercentageOnRobotMovement);
	actualRobotDisplacementY = (averageLandmarkMovementY * (1 - trustPercentageOnRobotMovement) + supposedRobotDisplacementY * trustPercentageOnRobotMovement);

	this->currentRobotPos.x = this->oldRobotPos.x + actualRobotDisplacementX;
	this->currentRobotPos.y = this->oldRobotPos.y + actualRobotDisplacementY;

	for (std::map<unsigned int, Case>::iterator it=matrix.begin(); it!=matrix.end(); ++it)
	{
		if (it->second.getState() == UPDATING)
		{
			it->second.setCurrentPosition(it->second.getOldPosition().x + actualRobotDisplacementX, it->second.getOldPosition().y + actualRobotDisplacementY);
			it->second.setState(UPTODATE);
		}
	}
}

void    Slam::updateState(pcl::PointCloud<pcl::PointXYZRGBA> const &cloud, IAgent *agent)
{
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
	this->moveLandmarks(reobservedLandmarks);

	this->moveAgent(agent);

	this->updatePositions(0.0);

  agent->setPos(this->currentRobotPos);

  //After all, remove bad landmarks
  //this->_landmarkDb->removeBadLandmarks(cloud, agent);
}

void    Slam::addLandmarks(std::vector<Landmarks::Landmark *> const &newLandmarks)
{
  for (std::vector<Landmarks::Landmark *>::const_iterator it = newLandmarks.begin(); it != newLandmarks.end(); ++it)
  {
    int landmarkId = this->_landmarkDb->addToDB(**it);
    int slamId = (int)this->addLandmarkToMatrix((*it)->pos);
    this->_landmarkDb->addSlamId(landmarkId, slamId);
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

	this->matrix.at(landmark->id).setOldPosition(this->matrix.at(landmark->id).getCurrentPosition());
	this->matrix.at(landmark->id).setCurrentPosition(pcl::PointXYZ(tempXX, tempYY, tempZZ));

	this->matrix.at(landmarkNumber).setState(MOVED);
}
