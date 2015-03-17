#ifndef JACOBIANMATRICEA_H_
# define JACOBIANMATRICEA_H_

# include <vector>
# include <tuple>
# include <cmath>
#	include <pcl/common/common.h>
#	include <pcl/impl/point_types.hpp>
#	include <pcl/common/projection_matrix.h>
# include "Landmarks.hh"
#	include	"Agent.hh"

class JacobianMatriceH
{
public:
	JacobianMatriceA();
	virtual ~JacobianMatriceA();
	void JacobiMath(pcl::PointXYZ const &posRobot, float teta);
	const std::vector<double> &getMatrice() const;
	const std::tuple<double,double,double> &getPrediction() const;

private:
	JacobianMatriceA(const JacobianMatriceA &);
	JacobianMatriceA &operator=(const JacobianMatriceA &);

protected:
	std::tuple<double,double,double> predictionF;
	std::vector<double> matrice;
};

#endif /* !JACOBIANMATRICEA_H_ */
