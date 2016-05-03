#include "Agent.hh"

const int Agent::BAUDRATE = 115200;
const char* Agent::DIVIDER = "1";  // 100 Hz
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
    /*
     *  start communication with the myAHRS+.
     */
    if(_sensor.start("/dev/ttyACM0", BAUDRATE) == false) {
        handle_error("start() returns false");
    }

     /*
     *  set binary output format
     *   - select Quaternion and IMU data
     */
    if(_sensor.cmd_binary_data_format("QUATERNION, IMU") == false) {
        handle_error("cmd_ascii_data_format() returns false");
    }

    /*
     *  set divider
     *   - output rate(Hz) = max_rate/divider
     */
    if(_sensor.cmd_divider(DIVIDER) ==false) {
        handle_error("cmd_divider() returns false");
    }

    /*
     *  set transfer mode
      *   - BC : Binary Message & Continuous mode
     */
    if(_sensor.cmd_mode("BC") ==false) {
        handle_error("cmd_mode() returns false");
    }
    //_movement.connectArduinoSerial();
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

double          roundValue(double value, double limit)
{
  return (std::nearbyint(value * limit) / limit);
}

void            Agent::updateState(bool true_update)
{
  // static int    round_value = 0;
  // static float  roll = 0.0;
  // static float  pitch = 0.0;
  // static float  yaw = 0.0;
  static time_t old_time = time(NULL);
  static time_t new_time = 0;
  time_t        delta = 0;
  pcl::PointXYZ new_velocity;
  pcl::PointXYZ new_pos;
  static pcl::PointXYZ gravity;
  static bool   first = true;
  double        ax, ay, az = 0.0;

  //true_update = true;
    //_movement.updateGyro();
  if(_sensor.wait_data() == true) { // waiting for new data
      // read counter or not?
      //sample_count = _sensor.get_sample_count();

      // copy sensor data
      _sensor.get_data(_sensor_data);

      new_time = time(NULL);
      delta = new_time - old_time;
      old_time = new_time;

      WithRobot::Quaternion& q = _sensor_data.quaternion;
      WithRobot::ImuData<float>& imu = _sensor_data.imu;
      WithRobot::EulerAngle e = q.to_euler_angle();
      ax = roundValue(imu.ax, 10.0);
      ay = roundValue(imu.ay, 10.0);
      az = roundValue(imu.az, 10.0);
       // print euler angle
      // WithRobot::EulerAngle& e = _sensor_data.euler_angle;
      // roll += e.roll;
      // pitch += e.pitch;
      // yaw += e.yaw;
      // ++round_value;

      //if (true_update) {
        // roll = roll / round_value;
        // pitch = pitch / round_value;
        // yaw = yaw / round_value;
        
        // delta is the time elapsed since you last calculated the position (in a loop for instance),
        // imu.a/q the acceleration you read from the sensor,
        // _velocity the old speed, new_velocity the new speed,
        // _pos the old position and new_pos the new position,
      // std::cerr << "DELTA TIME IS == " << delta << std::endl;
        if (first) {
          gravity.x = ax;
          gravity.y = ay;
          gravity.z = az;
          first = false;
        }

        if (delta > 0) {
          //Using simple integration to smooth things over
          new_velocity.x = _velocity.x + ((ax - gravity.x) + _acceleration.x) / (2 * delta);
          new_pos.x = _pos.x + (new_velocity.x + _velocity.x) / (2 * delta);
          new_velocity.y = _velocity.y + ((ay - gravity.y) + _acceleration.y) / (2 * delta);
          new_pos.y = _pos.y + (new_velocity.y + _velocity.y) / (2 * delta);
          new_velocity.z = _velocity.z + ((az - gravity.z) + _acceleration.z) / (2 * delta);
          new_pos.z = _pos.z + (new_velocity.z + _velocity.z) / (2 * delta);

          //Just acceleration
          // new_velocity.x = _velocity.x + (imu.ax - gravity.x) * delta;
          // new_pos.x = _pos.x + new_velocity.x * delta;
          // new_velocity.y = _velocity.y + (imu.ay - gravity.y) * delta;
          // new_pos.y = _pos.y + (new_velocity.y * delta);
          // new_velocity.z = _velocity.z + ((imu.az - gravity.z) * delta);
          // new_pos.z = _pos.z + (new_velocity.z * delta);
          
          this->setVelocity(new_velocity);
          this->setPos(roundValue(new_pos.x, 10.0), roundValue(new_pos.y, 10.0), roundValue(new_pos.z, 10.0));
        }
        _acceleration.x = roundValue(ax - gravity.x, 10.0);
        _acceleration.y = roundValue(ay - gravity.y, 10.0);
        _acceleration.z = roundValue(az - gravity.z, 10.0);
        // this->setPitch(std::nearbyint(e.pitch));
        // this->setRoll(std::nearbyint(e.roll));
        // this->setYaw(std::nearbyint(e.yaw));
        // roll = 0.0;
        // pitch = 0.0;
        // yaw = 0.0;
        // round_value = 0;
        // std::cerr << "Roll == " << e.roll << " -- pitch == " << e.pitch << " -- yaw == " << e.yaw << std::endl;
        // std::cerr << "AGENT Roll == " << _roll << " -- pitch == " << _pitch << " -- yaw == " << _yaw << std::endl;
        // std::cerr << "AGENT posx == " << _pos.x << " -- posy == " << _pos.y << " -- posz == " << _pos.z << std::endl;
      //}
      
    }

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
