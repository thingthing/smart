#ifndef		_AGENT_HH_
# define	_AGENT_HH_

#include <pcl-1.7/pcl/common/common.h>
#include <pcl-1.7/pcl/impl/point_types.hpp>
#include <pcl-1.7/pcl/common/projection_matrix.h>

class		Agent
{
public:

  //Defined in Agent.cpp
  static const double DEGREESPERSCAN; // meter
  static const double CAMERAPROBLEM; // meter


  Agent(double degreePerScan = DEGREESPERSCAN, double cameraProblem = CAMERAPROBLEM);
  ~Agent();

  pcl::PointXYZ	const	&getPos() const;
  double		getAngle() const;

  void		setAngle(double angle);
  void		setPos(pcl::PointXYZ const &pos);
  void		setPos(double x, double y, double z);

private:
  double	_angle;

public:
  double const	degreePerScan;
  double const	cameraProblem;

private:
  pcl::PointXYZ _pos;
};


#endif		/* !_AGENT_HH_ */
