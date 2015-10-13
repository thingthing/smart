#ifndef JACOBIANMATRICEJXR_H_
# define JACOBIANMATRICEJXR_H_

#	include <vector>
#	include <cmath>
#	include <pcl/common/common.h>
#	include <pcl/impl/point_types.hpp>
#	include <pcl/common/projection_matrix.h>
#	include	"IAgent.hh"

class	JacobianMatriceJxr
{
public:
	JacobianMatriceJxr();
	virtual ~JacobianMatriceJxr();
	void JacobiMath(IAgent const *agent);
	const std::vector<double> &getMatrice() const;

private:
	JacobianMatriceJxr(const JacobianMatriceJxr &);
	JacobianMatriceJxr &operator=(const JacobianMatriceJxr &);

protected:
	std::vector<double> matrice;
};

#endif /* !JACOBIANMATRICEJXR_H_ */
