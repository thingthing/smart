#include "test.hh"

Test::Case::Case() :
	state(UPTODATE)
{
	this->currentPosition.x = 0.0;
	this->currentPosition.y = 0.0;
	this->currentPosition.z = 0.0;

	this->oldPosition.x = 0.0;
	this->oldPosition.y = 0.0;
	this->oldPosition.z = 0.0;
}

Test::Case::Case(const pcl::PointXYZ &landmark) :
	oldPosition(landmark), currentPosition(landmark), state(UPTODATE)
{
}

Test::Case::Case(float x, float y, float z) :
	state(UPTODATE)
{
	this->currentPosition.x = x;
	this->currentPosition.y = y;
	this->currentPosition.z = z;

	this->oldPosition.x = x;
	this->oldPosition.y = y;
	this->oldPosition.z = z;
}

Test::Case::~Case()
{
}

Test::State Test::Case::getState() const
{
	return (state);
}

void Test::Case::setState(Test::State _state)
{
	this->state = _state;
}

pcl::PointXYZ Test::Case::getOldPosition() const
{
	return (this->oldPosition);
}

void Test::Case::setOldPosition(pcl::PointXYZ landmark)
{
	this->oldPosition = landmark;
}

pcl::PointXYZ Test::Case::getCurrentPosition() const
{
	return (this->currentPosition);
}

void Test::Case::setCurrentPosition(pcl::PointXYZ landmark)
{
	this->currentPosition = landmark;
}

void Test::Case::setCurrentPosition(float x, float y)
{
	this->currentPosition.x = x;
	this->currentPosition.y = y;
}

Test::Test()
{
	landmarkNumber = 0;
}

Test::Test(pcl::PointXYZ const &posRobot, float Theta)
{
	currentRobotPos = posRobot;
	oldRobotPos = posRobot;

	robotTheta = Theta;

	landmarkNumber = 0;
}

Test::Test(float X, float Y, float Z, float Theta)
{
	currentRobotPos.x = X;
	currentRobotPos.y = Y;
	currentRobotPos.z = Z;
	oldRobotPos.x = X;
	oldRobotPos.y = Y;
	oldRobotPos.z = Z;

	robotTheta = Theta;

	landmarkNumber = 0;
}

Test::Test(IAgent const *agent)
{
	this->currentRobotPos = agent->getPos();
	this->oldRobotPos = agent->getPos();

	this->robotTheta = agent->getBearing();

	landmarkNumber = 0;
}

Test::~Test()
{
}

unsigned int Test::addLandmark(const pcl::PointXYZ &position)
{
	float tempX, tempY;

	tempX = position.x * cos(this->robotTheta) - position.y * sin(this->robotTheta);
	tempY = position.x * sin(this->robotTheta) + position.y * cos(this->robotTheta);

	Test::Case tempCase = Test::Case(tempX, tempY, position.z);
	this->matrix[this->landmarkNumber] = tempCase;

	return(this->landmarkNumber++);
}

unsigned int Test::addLandmark(float x, float y, float z)
{
	float tempX, tempY;

	tempX = x * cos(this->robotTheta) - y * sin(this->robotTheta);
	tempY = x * sin(this->robotTheta) + y * cos(this->robotTheta);

	Test::Case tempCase = Test::Case(tempX, tempY, z);
	this->matrix[this->landmarkNumber] = tempCase;

	return(this->landmarkNumber++);
}

void Test::moveLandmark(unsigned int landmarkNumber, const pcl::PointXYZ &position)
{
	float tempX, tempY;

	tempX = position.x * cos(this->robotTheta) - position.y * sin(this->robotTheta);
	tempY = position.x * sin(this->robotTheta) + position.y * cos(this->robotTheta);

	this->matrix.at(landmarkNumber).setOldPosition(this->matrix.at(landmarkNumber).getCurrentPosition());
	this->matrix.at(landmarkNumber).setCurrentPosition(pcl::PointXYZ(tempX, tempY, position.z));

	this->matrix.at(landmarkNumber).setState(MOVED);
}

void Test::moveLandmark(unsigned int landmarkNumber, float x, float y, float z)
{
	float tempX, tempY;

	tempX = x * cos(this->robotTheta) - y * sin(this->robotTheta);
	tempY = x * sin(this->robotTheta) + y * cos(this->robotTheta);

	this->matrix.at(landmarkNumber).setOldPosition(this->matrix.at(landmarkNumber).getCurrentPosition());
	this->matrix.at(landmarkNumber).setCurrentPosition(pcl::PointXYZ(tempX, tempY, z));

	this->matrix.at(landmarkNumber).setState(MOVED);
}

void Test::moveAgent(pcl::PointXYZ const &posRobot, float Theta)
{
	this->oldRobotPos = this->currentRobotPos;
	this->currentRobotPos = posRobot;

	this->robotTheta = Theta;
}

void Test::moveAgent(float X, float Y, float Z, float Theta)
{
	this->oldRobotPos = this->currentRobotPos;
	this->currentRobotPos = pcl::PointXYZ(X, Y, Z);

	this->robotTheta = Theta;
}

void Test::moveAgent(IAgent const *agent)
{
	this->oldRobotPos = this->currentRobotPos;
	this->currentRobotPos = agent->getPos();

	this->robotTheta = agent->getBearing();
}

pcl::PointXYZ const &Test::getNewRobotPos() const {
	return this->currentRobotPos;
}

//trustPercentageOnRobotMovement must be between 0 and 1.
void Test::updatePositions(int trustPercentageOnRobotMovement)
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

	//std::cout << "this->currentRobotPos.x " << this->currentRobotPos.x << std::endl;
	//std::cout << "this->currentRobotPos.y " << this->currentRobotPos.y << std::endl;

	this->currentRobotPos.x = this->oldRobotPos.x + actualRobotDisplacementX;
	this->currentRobotPos.y = this->oldRobotPos.y + actualRobotDisplacementY;

	//std::cout << "this->currentRobotPos.x " << this->currentRobotPos.x << std::endl;
	//std::cout << "this->currentRobotPos.y " << this->currentRobotPos.y << std::endl;

	for (std::map<unsigned int, Case>::iterator it=matrix.begin(); it!=matrix.end(); ++it)
	{
		if (it->second.getState() == UPDATING)
		{
			it->second.setCurrentPosition(it->second.getOldPosition().x + actualRobotDisplacementX, it->second.getOldPosition().y + actualRobotDisplacementY);
			it->second.setState(UPTODATE);
		}
	} 
}

