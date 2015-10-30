#include "JacobianMatriceJz.hh"

JacobianMatriceJz::JacobianMatriceJz()
{
	matrice = std::vector<double>(4, 0);
}

JacobianMatriceJz::~JacobianMatriceJz()
{}

void JacobianMatriceJz::JacobiMath(IAgent const *agent)
{
        std::cerr << "JacobianMatriceJz JacobiMath before" << std::endl;
	matrice.at(0) = cos(agent->getDeltaTheta() + agent->getTheta());
	matrice.at(1) = -agent->getThrust() * sin(agent->getDeltaTheta() + agent->getTheta());
	matrice.at(2) = sin(agent->getDeltaTheta() + agent->getTheta());
	matrice.at(3) = agent->getThrust() * cos(agent->getDeltaTheta() + agent->getTheta());
          std::cerr << "JacobianMatriceJz JacobiMath after" << std::endl;
}

const std::vector<double> &JacobianMatriceJz::getMatrice() const
{
	return this->matrice;
}

