#ifndef		_AGENT_HH_
# define	_AGENT_HH_

#include <pcl-1.7/pcl/common/common.h>
#include <pcl-1.7/pcl/impl/point_types.hpp>
#include <pcl-1.7/pcl/common/projection_matrix.h>

class		Agent
{
public:
  Agent();
  ~Agent();

  pcl::PointXYZ	const &getPos() const;
  void		setPos(pcl::PointXYZ const &pos);

private:
  pcl::PointXYZ _pos;
};


#endif		/* !_AGENT_HH_ */
