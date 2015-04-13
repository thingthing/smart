#include "JacobianMatriceA.hh"

JacobianMatriceA::JacobianMatriceA()
{
	matrice = std::vector<double>(9, 0);
	matrice.at(0) = 1;
	matrice.at(4) = 1;
	matrice.at(8) = 1;
}

JacobianMatriceA::~JacobianMatriceA()
{}

void JacobianMatriceA::JacobiMath(Agent const &agent)
{
	//warning, thrust != speed

	//-deltaY
	matrice.at(2) = -agent.getThrust() * sin(agent.getTheta());
	//deltaX
	matrice.at(5) = agent.getThrust() * cos(agent.getTheta());
}

const std::vector<double> &JacobianMatriceA::getMatrice() const
{
	return this->matrice;
}
