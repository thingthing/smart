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
	void JacobiAdd(unsigned int landmarkNumber, SystemStateMatrice const &stateM, double range);

	//Utilities
	void deleteLandmark(unsigned int landmarkNumber);

	void setRnBMatrice(unsigned int landmarkNumber, SystemStateMatrice const &stateM);
	std::pair<double, double> getRnBMatrice(unsigned int landmarkNumber) const;

//protected:
	/*matrice containing the calculations on range and bearing for each landmarks
	first element is range, second one is bearing.*/
	std::map<unsigned int, std::pair<double,double>> rnbMatrice;
	/*for each landmark there are 4 elements, the first two for the range(X & Y) and
	the other two for the bearing(X & Y)*/
	std::map<unsigned int, std::tuple<double,double,double,double>> matrice;

//private:
  JacobianMatriceH(const JacobianMatriceH &);
  JacobianMatriceH &operator=(const JacobianMatriceH &);


};

#endif /* !JACOBIANMATRICEH_H_ */
