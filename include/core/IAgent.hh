#ifndef     _IAGENT_HH_
# define    _IAGENT_HH_

#include <string>

#include <pcl-1.7/pcl/common/common.h>
#include <pcl-1.7/pcl/impl/point_types.hpp>
#include <pcl-1.7/pcl/common/projection_matrix.h>

#include "event/Dispatcher.h"
#include "Capture.hh"

class       IAgent : public Utils::Dispatcher
{
public:

    //Defined in IAgent.cpp
    static const double DEGREESPERSCAN; // meter
    static const double CAMERAPROBLEM; // meter


    IAgent(double degreePerScan = DEGREESPERSCAN, double cameraProblem = CAMERAPROBLEM, std::string const &name = "Default");
    virtual ~IAgent();

    pcl::PointXYZ   const   &getPos() const;
    double          getBearing() const;
    Capture const & getCapture() const;
    double          getThrust() const;
    double          getTheta() const;
    double          getDeltaTheta() const;

    void            setThrust(double thrust);
    void            setTheta(double theta);
    void            setDeltaTheta(double deltaTheta);
    void            setBearing(double bearing);
    void            setPos(pcl::PointXYZ const &pos);
    void            setPos(double x, double y, double z);

    virtual pcl::PointCloud<pcl::PointXYZ> const &takeData() = 0;
    virtual void            updateState() = 0;
    virtual void            goTowardsGoal() = 0;

    inline std::string const &name() const { return (_name); }
    inline std::string const &status() const {return (_status) ;}
    inline std::string const &status(std::string const &status) {_status = status; return (_status);}

    double const    degreePerScan;
    double const    cameraProblem;

protected:
    double          _bearing;
    pcl::PointXYZ   _pos;
    std::string     _name;
    std::string     _status;
    Capture         _capture;
    double          _thrust;
    double          _theta;
    double          _deltaTheta;

public:
    //Use to align class with pointCloud
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


#endif      /* !_IAGENT_HH_ */
