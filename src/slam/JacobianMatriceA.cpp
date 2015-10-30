#include "JacobianMatriceA.hh"

JacobianMatriceA::JacobianMatriceA()
{
	matrice = std::vector<double>(9, 0);
	    std::cerr << "JacobianMatriceA constructor before" << std::endl;
	matrice.at(0) = 1;
	matrice.at(4) = 1;
	matrice.at(8) = 1;
		    std::cerr << "JacobianMatriceA constructor after" << std::endl;
}

JacobianMatriceA::~JacobianMatriceA()
{}

void JacobianMatriceA::JacobiMath(IAgent const *agent)
{
	//warning, thrust != speed

	    std::cerr << "JacobianMatriceA JacobiMath before" << std::endl;
	//-deltaY
	matrice.at(2) = -agent->getThrust() * sin(agent->getTheta());
	//deltaX
	matrice.at(5) = agent->getThrust() * cos(agent->getTheta());
		    std::cerr << "JacobianMatriceA constructor after" << std::endl;
}

const std::vector<double> &JacobianMatriceA::getMatrice() const
{
	return this->matrice;
}
