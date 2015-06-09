#ifndef     _AGENT_HH_
# define    _AGENT_HH_

#include <string>

#include <pcl-1.7/pcl/common/common.h>
#include <pcl-1.7/pcl/impl/point_types.hpp>
#include <pcl-1.7/pcl/common/projection_matrix.h>

#include "event/Dispatcher.h"
#include "Capture.hh"

class       Agent : public Utils::Dispatcher
{
public:

    //Defined in Agent.cpp
    static const double DEGREESPERSCAN; // meter
    static const double CAMERAPROBLEM; // meter


    Agent(double degreePerScan = DEGREESPERSCAN, double cameraProblem = CAMERAPROBLEM);
    ~Agent();

    pcl::PointXYZ   const   &getPos() const;
    pcl::PointXYZ   const   &getGoalPos() const;
    double          getBearing() const;
    Capture const &getCapture() const;

    void            setBearing(double bearing);
    void            setPos(pcl::PointXYZ const &pos);
    void            setPos(double x, double y, double z);
    void            setGoalPos(pcl::PointXYZ const &pos);
    void            setGoalPos(double x, double y, double z);

    pcl::PointCloud<pcl::PointXYZ> const &takeData();
    void            updateState();
    void            goTowardsGoal();
    bool            isAtDestination();

    inline std::string const &name() const { return (_name); }


    double const    degreePerScan;
    double const    cameraProblem;

private:
    double          _bearing;
    pcl::PointXYZ   _pos;
    pcl::PointXYZ   _goalPos;

protected:
    std::string     _name;
    Capture         _capture;

public:
    //Use to align class with pointCloud
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


#endif      /* !_AGENT_HH_ */
