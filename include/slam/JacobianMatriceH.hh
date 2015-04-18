#ifndef JACOBIANMATRICEH_H_
# define JACOBIANMATRICEH_H_

# include <vector>
# include <map>
# include <tuple>
# include <cmath>
#include <pcl/common/common.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/common/projection_matrix.h>
#include "SystemStateMatrice.hh"
#include "Landmarks.hh"

class JacobianMatriceH
{
public:
  JacobianMatriceH();
  virtual ~JacobianMatriceH();
	//values for <x,y> according to the range
  std::pair<double,double> getJacobianRange(unsigned int landmarkNumber) const;
	//values for <x,y> according to the bearing
  std::pair<double,double> getJacobianBearing(unsigned int landmarkNumber) const;
	//set the values of H for the specified landmark
	void JacobiMath(unsigned int landmarkNumber, SystemStateMatrice stateM, double range);

	void setRnBMatrice(unsigned int landmarkNumber, SystemStateMatrice stateM);
	std::pair<double, double> getRnBMatrice(unsigned int landmarkNumber) const;

private:
  JacobianMatriceH(const JacobianMatriceH &);
  JacobianMatriceH &operator=(const JacobianMatriceH &);

protected:
	/*
	matrice 2x
	for each landmark there are two pairs, one for the range(X & Y)
	and one for the bearing (X & Y)
	*/
	std::vector<std::pair<double, double>> matrice;
	//reminder, the derivative for the theta of the robot is 0 for the range and -1 for the bearing

	/*
	matrice containing the calculations on range and bearing for each landmarks
	first element is range, second one is bearing
	*/
	std::vector<std::pair<double, double>> rnbMatrice;

};

#endif /* !JACOBIANMATRICEH_H_ */
