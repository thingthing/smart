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
	//we need the noise
	double x = agent.getPos().x + agent.getThrust() * cos(agent.getTheta());//+ 'noise' * thrust *cos(theta)
	double y = agent.getPos().y + agent.getThrust() * sin(agent.getTheta());//+ 'noise' * thrust *sin(theta)
	double angle = agent.getTheta() + agent.getDeltaTheta();//+ 'noise' * deltaTheta

	std::get<0>(predictionF) = x;
	std::get<1>(predictionF) = y;
	std::get<2>(predictionF) = angle;

	//-deltaY
	matrice.at(2) = -agent.getThrust() * sin(agent.getTheta());
	//deltaX
	matrice.at(5) = agent.getThrust() * cos(agent.getTheta());
}

const std::vector<double> &JacobianMatriceA::getMatrice() const
{
	return this->matrice;
}

const std::tuple<double,double,double> &JacobianMatriceA::getPrediction() const
{
	return this->predictionF;
}
