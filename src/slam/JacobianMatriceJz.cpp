#include "JacobianMatriceJz.hh"

JacobianMatriceJz::JacobianMatriceJz()
{
	matrice = std::vector<double>(4);
}

JacobianMatriceJz::~JacobianMatriceJz()
{}

void JacobianMatriceJz::JacobiMath(IAgent const &agent)
{
	matrice.at(0) = cos(agent.getDeltaTheta() + agent.getTheta());
	matrice.at(1) = -agent.getThrust() * sin(agent.getDeltaTheta() + agent.getTheta());
	matrice.at(2) = sin(agent.getDeltaTheta() + agent.getTheta());
	matrice.at(3) = agent.getThrust() * cos(agent.getDeltaTheta() + agent.getTheta());
}

const std::vector<double> &JacobianMatriceJz::getMatrice() const
{
	return this->matrice;
}

