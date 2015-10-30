#include "JacobianMatriceJxr.hh"

JacobianMatriceJxr::JacobianMatriceJxr()
{
	matrice = std::vector<double>(6, 0);
        std::cerr << "JacobianMatriceJxR constructor before" << std::endl;
	matrice.at(0) = 1;
	matrice.at(4) = 1;
          std::cerr << "JacobianMatriceJxr constructor after" << std::endl;
}

JacobianMatriceJxr::~JacobianMatriceJxr()
{}

void JacobianMatriceJxr::JacobiMath(IAgent const *agent)
{
          std::cerr << "JacobianMatriceJxr JacobiMath before" << std::endl;
	//-deltaY
	matrice.at(2) = -agent->getThrust() * sin(agent->getTheta());
	//deltaX
	matrice.at(5) = agent->getThrust() * cos(agent->getTheta());
        std::cerr << "JacobianMatriceJxr JacobiMath after" << std::endl;
}

const std::vector<double> &JacobianMatriceJxr::getMatrice() const
{
	return this->matrice;
}

