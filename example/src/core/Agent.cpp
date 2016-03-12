#include "Agent.hh"

const double IAgent::DEGREESPERSCAN = 0.5;
const double IAgent::CAMERAPROBLEM = 4.1; // meters
const int    Agent::DEFAULTBATTERY = 1000;

Agent::Agent(double degreePerScan, double cameraProblem)
    : IAgent(degreePerScan, cameraProblem, "AgentVirtuel", Agent::DEFAULTBATTERY)
{
    this->_capture = new Capture();
    this->_pos.x = 0;
    this->_pos.y = 0;
    this->_pos.z = 0;
    _capture->registerCallback("takeDataEvent", [this]() {takeData();});
    _movement.connectArduinoSerial();
}

Agent::~Agent()
{
}

int             Agent::lowerBattery(int value_to_lower)
{
    // std::cerr << "LOWER BATTER" << std::endl;
    _battery -= value_to_lower;
    if (_battery <= 0)
    {
        this->status("NOBATTERY");
        _battery = 0;
    }
    else if (_battery < (5 * Agent::DEFAULTBATTERY) / 100)
        this->status("LOWBATTERY");
    return (_battery);
}

int             Agent::chargeBattery(int value_to_add)
{
    // std::cerr << "CHARGE BATTERY" << std::endl;
    _battery += value_to_add;
    if (_battery >= Agent::DEFAULTBATTERY)
    {
        _battery = Agent::DEFAULTBATTERY;
        this->status("FULLBATTERY");
    }
    return (_battery);
}

void            Agent::setGoalPos(pcl::PointXYZ const &pos)
{
    this->_goalPos.x = round(pos.x);
    this->_goalPos.y = round(pos.y);
    this->_goalPos.z = round(pos.z);
    std::cout << "recieve setGoalPos from serveur " << pos << std::endl;
}

void            Agent::setGoalPos(double x, double y, double z)
{
    std::cerr << "Setting goal pos to " << x << " " << y << " " << z << std::endl;
    this->_goalPos.x = x;
    this->_goalPos.y = y;
    this->_goalPos.z = z;
}

pcl::PointXYZ   const   &Agent::getGoalPos() const
{
    return (this->_goalPos);
}

pcl::PointCloud<pcl::PointXYZRGBA> const &Agent::takeData()
{
  pcl::PointCloud<pcl::PointXYZRGBA> cloud = _capture->getData();
  std::cerr << "Getting data in takeData == " << cloud.size() << std::endl;
  /// @todo: Move cloud according to rotation of agent
  // Last three parameters are in order: roll, pitch, yaw
  Eigen::Affine3f   transfo = pcl::getTransformation (_pos.x, _pos.y, _pos.z, _roll, _pitch, _yaw);
  pcl::transformPointCloud<pcl::PointXYZRGBA>(cloud, cloud, transfo);
  this->dispatch("SendCloudEvent", cloud);
  return (_capture->getData());
}


void             Agent::goTowardsGoal()
{
  static unsigned int i = 0;
  ++i;
  std::cout << i << std::endl;
      _movement.sendMotorSpeed(0, 1500);
      _movement.sendMotorSpeed(1, 1500);
      _movement.updateMotorsSpeed();

  if (i < 3)
    _movement.goForward();
  else if (i < 6)
    _movement.goBack();
  else if (i <9)
    _movement.goLeft();
  else if (i < 12)
    _movement.goRight();
  else if (i < 15)
    {
      _movement.sendMotorSpeed(0, 1500);
      _movement.sendMotorSpeed(1, 1500);
    }
  else
    i = 0;
  if (i < 16)
      _movement.updateMotorsSpeed();
  /*
  // TEST A ALA CON
    std::cout << "Moving to goal " << _goalPos.x << " " << _goalPos.y << " " << _goalPos.z << " with pos == "
    << _pos.x << " " << _pos.y << " " << _pos.z << std::endl;
    if (_pos.x != _goalPos.x)
        _pos.x += (_goalPos.x < _pos.x) ? -1 : 1;
    if (_pos.y != _goalPos.y)
        _pos.y += (_goalPos.y < _pos.y) ? -1 : 1;
    if (_pos.z != _goalPos.z)
        _pos.z += (_goalPos.z < _pos.z) ? -1 : 1;

std::cout << "After Moving to goal " << _goalPos.x << " " << _goalPos.y << " " << _goalPos.z << " with pos == "
    << _pos.x << " " << _pos.y << " " << _pos.z << std::endl;
*/
}

bool            Agent::isAtDestination() const
{
    return (_pos.x == _goalPos.x && _pos.y == _goalPos.y && _pos.z == _goalPos.z);
}

bool            Agent::isAtBase() const
{
    return (_pos.x == 0 && _pos.y == 0 && _pos.z == 0);
}



void            Agent::updateState()
{
    _movement.updateGyro();
    if (!this->isAtBase())
        this->lowerBattery(1);
    this->dispatch("SendPacketEvent", this);
    // std::cout << "GoalPos is " << _goalPos << std::endl;
    //    if (this->isAtDestination() == false)
    //  {
        std::cout << "Going goTowardsGoal" << std::endl;
        this->goTowardsGoal();
	//  } else if (this->isAtBase() && this->getBattery() < Agent::DEFAULTBATTERY)
	//  {
	//      this->chargeBattery(1);
	//  }
    _movement.updateSerial();
    this->setPitch(_movement.getPitchRollYaw().x);
    this->setRoll(_movement.getPitchRollYaw().y);
    this->setYaw(_movement.getPitchRollYaw().z);

}
