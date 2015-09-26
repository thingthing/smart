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

    int             lowerBattery(int value_to_lower);
    int             chargeBattery(int value_to_add);
    void            setGoalPos(pcl::PointXYZ const &pos);
    void            setGoalPos(double x, double y, double z);

    virtual pcl::PointCloud<pcl::PointXYZ> const &takeData();
    virtual void            updateState();
    virtual void            goTowardsGoal();
    bool            isAtDestination() const;
    bool            isAtBase() const;

    static const int DEFAULTBATTERY;
private:
    pcl::PointXYZ   _goalPos;
};


#endif      /* !_AGENT_HH_ */
