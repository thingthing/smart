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

    void            setGoalPos(pcl::PointXYZ const &pos);
    void            setGoalPos(double x, double y, double z);

    pcl::PointCloud<pcl::PointXYZ> const &takeData();
    void            updateState();
    void            goTowardsGoal();
    bool            isAtDestination();

private:
    pcl::PointXYZ   _goalPos;


};


#endif      /* !_AGENT_HH_ */
