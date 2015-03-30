#ifndef JACOBIANMATRICEJZ_H_
#	define JACOBIANMATRICEJZ_H_

#	include <vector>
#	include <cmath>
#	include <pcl/common/common.h>
#	include <pcl/impl/point_types.hpp>
#	include <pcl/common/projection_matrix.h>
#	include	"Agent.hh"

class	JacobianMatriceJz
{
public:
	JacobianMatriceJz();
	virtual ~JacobianMatriceJz();
	void JacobiMath(Agent const &agent);
	const std::vector<double> &getMatrice() const;

private:
	JacobianMatriceJz(const JacobianMatriceJz &);
	JacobianMatriceJz &operator=(const JacobianMatriceJz &);

protected:
	std::vector<double> matrice;
};

#endif /* !JACOBIANMATRICEJXR_H_ */
