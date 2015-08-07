#include "Agent.hh"

const double IAgent::DEGREESPERSCAN = 0.5;
const double IAgent::CAMERAPROBLEM = 4.1; // meters
const int    Agent::DEFAULTBATTERY = 10000;

Agent::Agent(double degreePerScan, double cameraProblem)
  : IAgent(degreePerScan, cameraProblem, "Agent"), _battery(Agent::DEFAULTBATTERY)
{
  this->_pos.x = 0;
  this->_pos.y = 0;
  this->_pos.z = 0;
}

Agent::~Agent()
{
}

int              Agent::getBattery() const
{
  return (_battery);
}

void             Agent::setBattery(int new_battery_value)
{
  _battery = new_battery_value;
}

int             Agent::lowerBattery(int value_to_lower)
{
  _battery -= value_to_lower;
  if (_battery < (5 * Agent::DEFAULTBATTERY) / 100)
    this->status("LOWBATTERY");
  if (_battery <= 0)
  {
    this->status("NOBATTERY");
    _battery = 0;
  }
  return (_battery);
}

int             Agent::chargeBattery(int value_to_add)
{
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
  this->_goalPos.x = pos.x;
  this->_goalPos.y = pos.y;
  this->_goalPos.z = pos.z;
  std::cout << "recieve setGoalPos from serveur " << pos << std::endl;
}

void            Agent::setGoalPos(double x, double y, double z)
{
  this->_goalPos.x = x;
  this->_goalPos.y = y;
  this->_goalPos.z = z;
}

pcl::PointXYZ   const   &Agent::getGoalPos() const
{
  return (this->_goalPos);
}

pcl::PointCloud<pcl::PointXYZ> const &Agent::takeData()
{
  return (this->_capture.captureData());
}


void             Agent::goTowardsGoal()
{
    if (_pos.x != _goalPos.x)
        _pos.x += (_goalPos.x < _pos.x) ? -1 : 1;
    if (_pos.y != _goalPos.y)
        _pos.y += (_goalPos.y < _pos.y) ? -1 : 1;
    if (_pos.z != _goalPos.z)
        _pos.z += (_goalPos.z < _pos.z) ? -1 : 1;
}

bool            Agent::isAtDestination()
{
  return (_pos.x == _goalPos.x && _pos.y == _goalPos.y && _pos.z == _goalPos.z);
}

void            Agent::updateState()
{
  this->dispatch("SendPacketEvent");
  std::cout << "GoalPos is " << _goalPos << std::endl;
  if (this->isAtDestination() == false)
  {
    std::cout << "Going goTowardsGoal" << std::endl;
    this->goTowardsGoal();
  }
}
