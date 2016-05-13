#include "Agent.hh"

const int Agent::BAUDRATE = 115200;
const double IAgent::DEGREESPERSCAN = 0.5;
const double IAgent::CAMERAPROBLEM = 4.1; // meters
const int    Agent::DEFAULTBATTERY = 1000;
const char* Agent::SAVE_FILE_NAME = "capture.pcd";


void handle_error(const char* error_msg)
{
    std::cerr << "ERROR: " <<  error_msg << std::endl;
    throw new std::exception();
}

Agent::Agent(double degreePerScan, double cameraProblem)
    : IAgent(degreePerScan, cameraProblem, "AgentVirtuel", Agent::DEFAULTBATTERY, IAgent::DELAYED)
{
    this->_capture = new Capture();

    this->_pos.x = 0;
    this->_pos.y = 0;
    this->_pos.z = 0;
    _capture->registerCallback("takeDataEvent", [this]() {takeData();});
    _sensor = new WithRobot::AgentAhrs(this, "/dev/ttyACM0", BAUDRATE);
    if(_sensor->initialize() == false) {
        handle_error("_sensor.initialize() returns false");
    }
    //_movement.connectArduinoSerial();
}

Agent::~Agent()
{
  delete _sensor;
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
    //std::cout << "recieve setGoalPos from serveur " << pos << std::endl;
}

void            Agent::setGoalPos(double x, double y, double z)
{
    //std::cerr << "Setting goal pos to " << x << " " << y << " " << z << std::endl;
    this->_goalPos.x = x;
    this->_goalPos.y = y;
    this->_goalPos.z = z;
}

pcl::PointXYZ   const   &Agent::getGoalPos() const
{
    return (this->_goalPos);
}

void             Agent::executeDownload()
{
  pcl::PointCloud<pcl::PointXYZRGBA> cloud;
  pcl::io::loadPCDFile(Agent::SAVE_FILE_NAME, cloud);
  std::cerr << "SENDING STUFF cloud size == " << cloud.size() << std::endl;
  std::remove(Agent::SAVE_FILE_NAME); // delete file
  this->dispatch("SendCloudEvent", cloud);
}

pcl::PointCloud<pcl::PointXYZRGBA> const &Agent::takeData()
{
  //Get Cloud data
  pcl::PointCloud<pcl::PointXYZRGBA> cloud = _capture->getData();
  //Update state to have the state in correspondance with the cloud data
  this->updateState();

  //Tranform cloud data with possible position of agent
  //   transfo = pcl::getTransformation (_pos.x, _pos.y, _pos.z, _roll, _pitch, _yaw);
  // pcl::transformPointCloud<pcl::PointXYZRGBA>(cloud, cloud, transfo);
  //Call to slam to get the real position of agent with new data
  this->dispatch("getDataEvent", cloud, this);

  if (_send_data) {
    //std::cerr << "Getting data in takeData == " << cloud.size() << std::endl;
    //Tranform cloud data with actual position of agent
    Eigen::Affine3f transfo = pcl::getTransformation (_pos.x, _pos.y, _pos.z, _roll, _pitch, _yaw);
    pcl::transformPointCloud<pcl::PointXYZRGBA>(cloud, cloud, transfo);
    if (_mode == IAgent::DIRECT) {
      //Send new cloud data
      this->dispatch("SendCloudEvent", cloud);
    } else if (_mode == IAgent::DELAYED) {
      std::cerr << "Cloud size before == " << cloud.size() << std::endl;
      pcl::PointCloud<pcl::PointXYZRGBA> save;
      if (pcl::io::loadPCDFile(Agent::SAVE_FILE_NAME, save) == 0)
        cloud += save;
      std::cerr << "Cloud size after == " << cloud.size() << " with save size == " << save.size() << std::endl;
      //Save cloud in file
      pcl::io::savePCDFile(Agent::SAVE_FILE_NAME, cloud, true);
    }
  }
  return (_capture->getData());
}


void             Agent::goTowardsGoal()
{
  static unsigned int i = 0;

  if (this->isAtDestination() == false) {
    ++i;
    // std::cout << i << std::endl;
    //     _movement.sendMotorSpeed(0, 1500);
    //     _movement.sendMotorSpeed(1, 1500);
    //     _movement.updateMotorsSpeed();

    // if (i < 3)
    //   _movement.goForward();
    // else if (i < 6)
    //   _movement.goBack();
    // else if (i <9)
    //   _movement.goLeft();
    // else if (i < 12)
    //   _movement.goRight();
    // else if (i < 15)
    //   {
    //     _movement.sendMotorSpeed(0, 1500);
    //     _movement.sendMotorSpeed(1, 1500);
    //   }
    // else
    //   i = 0;
    // if (i < 16)
    //     _movement.updateMotorsSpeed();
    
    // TEST A ALA CON
      // std::cout << "Moving to goal " << _goalPos.x << " " << _goalPos.y << " " << _goalPos.z << " with pos == "
      // << _pos.x << " " << _pos.y << " " << _pos.z << std::endl;
      // if (_pos.x != _goalPos.x)
      //     _pos.x += (_goalPos.x < _pos.x) ? -1 : 1;
      // if (_pos.y != _goalPos.y)
      //     _pos.y += (_goalPos.y < _pos.y) ? -1 : 1;
      // if (_pos.z != _goalPos.z)
      //     _pos.z += (_goalPos.z < _pos.z) ? -1 : 1;

    // std::cout << "After Moving to goal " << _goalPos.x << " " << _goalPos.y << " " << _goalPos.z << " with pos == "
    //     << _pos.x << " " << _pos.y << " " << _pos.z << std::endl;
  }
}

bool            Agent::isAtDestination() const
{
    return (_pos.x == _goalPos.x && _pos.y == _goalPos.y && _pos.z == _goalPos.z);
}

bool            Agent::isAtBase() const
{
    return (_pos.x == 0 && _pos.y == 0 && _pos.z == 0);
}



void            Agent::updateState(bool true_update)
{
  // static int    round_value = 0;
  // static float  roll = 0.0;
  // static float  pitch = 0.0;
  // static float  yaw = 0.0;

    //@Todo: get real battery
    if (!this->isAtBase())
        this->lowerBattery(1);
    else if (this->isAtBase() && this->getBattery() < Agent::DEFAULTBATTERY)
    {
         this->chargeBattery(1);
    }
    
    this->dispatch("SendPacketEvent", this);
    //std::cout << "GoalPos is " << _goalPos << std::endl;
  //      if (this->isAtDestination() == false)
  //    {
  //       //std::cout << "Going goTowardsGoal" << std::endl;
  //       this->goTowardsGoal();
	 // } else if (this->isAtBase() && this->getBattery() < Agent::DEFAULTBATTERY)
	 // {
	 //     this->chargeBattery(1);
	 // }
    // _movement.updateSerial();
    // this->setPitch(_movement.getPitchRollYaw().x);
    // this->setRoll(_movement.getPitchRollYaw().y);
    // this->setYaw(_movement.getPitchRollYaw().z);

}
