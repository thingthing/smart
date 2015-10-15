#ifndef JACOBIANMATRICEA_H_
#	define JACOBIANMATRICEA_H_

#	include <vector>
#	include <tuple>
#	include <cmath>
#	include <pcl/common/common.h>
#	include <pcl/impl/point_types.hpp>
#	include <pcl/common/projection_matrix.h>
#	include "Landmarks.hh"
#	include	"IAgent.hh"

class JacobianMatriceA
{
public:
	JacobianMatriceA();
	virtual ~JacobianMatriceA();
	void JacobiMath(IAgent const &agent);
	const std::vector<double> &getMatrice() const;

private:
	JacobianMatriceA(const JacobianMatriceA &);
	JacobianMatriceA &operator=(const JacobianMatriceA &);

protected:
	std::vector<double> matrice;
};

#endif /* !JACOBIANMATRICEA_H_ */
