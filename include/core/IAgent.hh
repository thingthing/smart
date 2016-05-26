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

    enum e_mode {
        DIRECT,
        DELAYED
    };

    IAgent(double degreePerScan = DEGREESPERSCAN, double cameraProblem = CAMERAPROBLEM,
           std::string const &name = "Default", int battery = 0, e_mode mode = DIRECT);
    virtual ~IAgent();

    pcl::PointXYZ   const   &getPos() const;
    double          getRoll() const;
    ICapture        *getCapture() const;
    double          getYaw() const;
    double          getPitch() const;
    double          getPrevYaw() const;
    double          getPrevPitch() const;
    double          getPrevRoll() const;
    int             getBattery() const;
    pcl::PointXYZ   getVelocity() const { return _velocity; }
    pcl::PointXYZ   getAcceleration() const { return _acceleration; }
    inline e_mode   getMode() const { return _mode; }

    inline bool     getSendData() { return _send_data; }
    inline void     setSendData(bool data) { _send_data = data; }

    void            setBattery(int new_battery_value);
    void            setRoll(double thrust);
    void            setYaw(double theta);
    void            setPitch(double deltaTheta);
    void            setPrevRoll(double deltaTheta);
    void            setPrevPitch(double deltaTheta);
    void            setPrevYaw(double deltaTheta);
    void            setPos(pcl::PointXYZ const &pos);
    void            setPos(double x, double y, double z);
    inline void     setVelocity(pcl::PointXYZ const &velo) { _velocity = velo; }
    inline void     setAcceleration(pcl::PointXYZ const &accel) { _acceleration = accel; }
    std::string const &status(std::string const &status);
    inline void     setMode(e_mode mode) { _mode = mode; }

    virtual pcl::PointCloud<pcl::PointXYZRGBA> const &takeData() = 0;
    virtual void            updateState(bool true_update = true) = 0;
    virtual void            goTowardsGoal() = 0;

    inline std::string const &name() const { return (_name); }
    inline std::string const &status() const {return (_status) ;}

    double const    degreePerScan;
    double const    cameraProblem;

protected:
    pcl::PointXYZ   _pos;
    double          _yaw;
    std::string     _name;
    std::string     _status;
    double	         _roll;
    double	         _pitch;
    double           _prev_roll;
    double           _prev_pitch;
    double           _prev_yaw;
    ICapture         *_capture;
    int             _battery;
    bool            _send_data;
    pcl::PointXYZ   _velocity;
    pcl::PointXYZ   _acceleration;
    e_mode          _mode;

public:
    //Use to align class with pointCloud
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


#endif      /* !_IAGENT_HH_ */
