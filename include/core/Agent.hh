#ifndef     _AGENT_HH_
# define    _AGENT_HH_

#include <string>

#include "IAgent.hh"

class       Agent : public IAgent
{
public:

    Agent(double degreePerScan = DEGREESPERSCAN, double cameraProblem = CAMERAPROBLEM);
    ~Agent();

    pcl::PointXYZ   const   &getGoalPos() const;
    int                     getBattery() const;

    void            setBattery(int new_battery_value);
    int             lowerBattery(int value_to_lower);
    int             chargeBattery(int value_to_add);
    void            setGoalPos(pcl::PointXYZ const &pos);
    void            setGoalPos(double x, double y, double z);

    pcl::PointCloud<pcl::PointXYZ> const &takeData();
    void            updateState();
    void            goTowardsGoal();
    bool            isAtDestination() const;
    bool            isAtBase() const;

private:
    static const int DEFAULTBATTERY;
    pcl::PointXYZ   _goalPos;
    int             _battery;
};


#endif      /* !_AGENT_HH_ */
