#ifndef     _AGENT_HH_
# define    _AGENT_HH_

#include <string>
#include <pcl/io/pcd_io.h>

#include "IAgent.hh"
#include "Capture.hh"
#include "myahrs_plus.hpp"
//#include "movement/Movement.h"

class       Agent : public IAgent
{
public:

    Agent(double degreePerScan = DEGREESPERSCAN, double cameraProblem = CAMERAPROBLEM);
    ~Agent();

    pcl::PointXYZ   const   &getGoalPos() const;

    int             lowerBattery(int value_to_lower);
    int             chargeBattery(int value_to_add);
    void            setGoalPos(pcl::PointXYZ const &pos);
    void            setGoalPos(double x, double y, double z);

    virtual pcl::PointCloud<pcl::PointXYZRGBA> const &takeData();
    virtual void            updateState(bool true_update = true);
    virtual void            goTowardsGoal();
    bool            isAtDestination() const;
    bool            isAtBase() const;
    void            executeDownload();


    static const int DEFAULTBATTERY;
    static const int BAUDRATE;
    static const char* DIVIDER;
    static const char* SAVE_FILE_NAME;

private:
    pcl::PointXYZ               _goalPos;
    WithRobot::MyAhrsPlus      _sensor;
    WithRobot::SensorData      _sensor_data;
    //Movement        _movement;
};


#endif      /* !_AGENT_HH_ */
