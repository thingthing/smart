#ifndef JACOBIANMATRICEH_H_
# define JACOBIANMATRICEH_H_

# include <vector>
# include <map>
# include <tuple>
# include <cmath>
# include <pcl/common/common.h>
# include <pcl/impl/point_types.hpp>
# include <pcl/common/projection_matrix.h>
# include "Landmarks.hh"

class JacobianMatriceH
{
public:
  JacobianMatriceH();
  virtual ~JacobianMatriceH();
	//values for <x,y,theta> according to the range
  const std::tuple<double,double,double> &getJacobianRange(unsigned int landmarkNumber) const;
	//values for <x,y,theta> according to the bearing
  const std::tuple<double,double,double> &getJacobianBearing(unsigned int landmarkNumber) const;
	//set the values of H for the specified landmark
	void JacobiMath(unsigned int landmarkNumber, SystemStateMatrice stateM);

private:
  JacobianMatriceH(const JacobianMatriceH &);
  JacobianMatriceH &operator=(const JacobianMatriceH &);

protected:
  std::tuple<double,double,double> bearingH;
  std::tuple<double,double,double> rangeH;
};

#endif /* !JACOBIANMATRICEH_H_ */
