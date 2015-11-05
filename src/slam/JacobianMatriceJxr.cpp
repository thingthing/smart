#include "JacobianMatriceJxr.hh"

JacobianMatriceJxr::JacobianMatriceJxr()
{
	matrice = std::vector<double>(6, 0);
	matrice.at(0) = 1;
	matrice.at(4) = 1;
}

JacobianMatriceJxr::~JacobianMatriceJxr()
{}

void JacobianMatriceJxr::JacobiMath(IAgent const *agent)
{
	//-deltaY
	matrice.at(2) = -agent->getThrust() * sin(agent->getTheta());
	//deltaX
	matrice.at(5) = agent->getThrust() * cos(agent->getTheta());
}

const std::vector<double> &JacobianMatriceJxr::getMatrice() const
{
	return this->matrice;
}

