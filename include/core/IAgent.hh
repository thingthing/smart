#ifndef     _IAGENT_HH_
# define    _IAGENT_HH_

#include <string>

#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/common/projection_matrix.h>

#include "event/Dispatcher.h"
#include "ICapture.hh"

class       IAgent : public Utils::Dispatcher
{
public:

    //Defined in IAgent.cpp
    static const double DEGREESPERSCAN; // meter
    static const double CAMERAPROBLEM; // meter


    IAgent(double degreePerScan = DEGREESPERSCAN, double cameraProblem = CAMERAPROBLEM,
           std::string const &name = "Default", int battery = 0);
    virtual ~IAgent();

    pcl::PointXYZ   const   &getPos() const;
    double          getBearing() const;
    ICapture const  *getCapture() const;
    double          getThrust() const;
    double          getTheta() const;
    double          getDeltaTheta() const;
    int             getBattery() const;

    void            setBattery(int new_battery_value);
    void            setThrust(double thrust);
    void            setTheta(double theta);
    void            setDeltaTheta(double deltaTheta);
    void            setBearing(double bearing);
    void            setPos(pcl::PointXYZ const &pos);
    void            setPos(double x, double y, double z);
    std::string const &status(std::string const &status);

    virtual pcl::PointCloud<pcl::PointXYZRGBA> const &takeData() = 0;
    virtual void            updateState() = 0;
    virtual void            goTowardsGoal() = 0;

    inline std::string const &name() const { return (_name); }
    inline std::string const &status() const {return (_status) ;}

    double const    degreePerScan;
    double const    cameraProblem;

protected:
    double          _bearing;
    pcl::PointXYZ   _pos;
    std::string     _name;
    std::string     _status;
    ICapture         *_capture;
    double          _thrust;
    double          _theta;
    double          _deltaTheta;
    int             _battery;

public:
    //Use to align class with pointCloud
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


#endif      /* !_IAGENT_HH_ */
