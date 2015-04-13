#include "CovarianceMatrice.hh"

const unsigned int CovarianceMatrice::SIZEINIT = 3;

CovarianceMatrice::Case::Case() :
  _value(0.0), _state(CALCULATION)
{
}

CovarianceMatrice::Case::Case(State state) :
  _value(0.0), _state(state)
{
}

CovarianceMatrice::Case::~Case()
{}

void CovarianceMatrice::Case::setValue(float value)
{
  this->_value = value;
}

void CovarianceMatrice::Case::setState(CovarianceMatrice::State state)
{
  this->_state = state;
}

CovarianceMatrice::State CovarianceMatrice::Case::getState() const
{
  return this->_state;
}

float CovarianceMatrice::Case::getValue() const
{
  return this->_value;
}

CovarianceMatrice::CovarianceMatrice()
{
  this->_matrice.resize(CovarianceMatrice::SIZEINIT);
  for (unsigned int i = 0 ; i < CovarianceMatrice::SIZEINIT; ++i)
    this->_matrice[i].resize(CovarianceMatrice::SIZEINIT);
  this->_matrice[0][0].setState(CovarianceMatrice::POSITION);
  this->_matrice[0][0].setValue(0.0);

  this->_matrice[1][1].setState(CovarianceMatrice::POSITION);
  this->_matrice[1][1].setValue(0.0);

  this->_matrice[2][2].setState(CovarianceMatrice::POSITION);
  this->_matrice[2][2].setValue(0.0);

  this->_matrice[0][1].setState(CovarianceMatrice::NOTUSED);
  this->_matrice[0][2].setState(CovarianceMatrice::NOTUSED);
  this->_matrice[1][0].setState(CovarianceMatrice::NOTUSED);
  this->_matrice[1][2].setState(CovarianceMatrice::NOTUSED);
  this->_matrice[2][0].setState(CovarianceMatrice::NOTUSED);
  this->_matrice[2][1].setState(CovarianceMatrice::NOTUSED);
}

CovarianceMatrice::CovarianceMatrice(float X, float Y, float theta)
{
  this->_matrice.resize(CovarianceMatrice::SIZEINIT);
  for (unsigned int i = 0 ; i < CovarianceMatrice::SIZEINIT; ++i)
    this->_matrice[i].resize(CovarianceMatrice::SIZEINIT);
  this->_matrice[0][0].setState(CovarianceMatrice::POSITION);
  this->_matrice[0][0].setValue(X);

  this->_matrice[1][1].setState(CovarianceMatrice::POSITION);
  this->_matrice[1][1].setValue(Y);

  this->_matrice[2][2].setState(CovarianceMatrice::POSITION);
  this->_matrice[2][2].setValue(theta);

  this->_matrice[0][1].setState(CovarianceMatrice::NOTUSED);
  this->_matrice[0][2].setState(CovarianceMatrice::NOTUSED);
  this->_matrice[1][0].setState(CovarianceMatrice::NOTUSED);
  this->_matrice[1][2].setState(CovarianceMatrice::NOTUSED);
  this->_matrice[2][0].setState(CovarianceMatrice::NOTUSED);
  this->_matrice[2][1].setState(CovarianceMatrice::NOTUSED);
}

CovarianceMatrice::CovarianceMatrice(pcl::PointXYZ const &pos, float theta)
{
    this->_matrice.resize(CovarianceMatrice::SIZEINIT);
  for (unsigned int i = 0 ; i < CovarianceMatrice::SIZEINIT; ++i)
    this->_matrice[i].resize(CovarianceMatrice::SIZEINIT);
  this->_matrice[0][0].setState(CovarianceMatrice::POSITION);
  this->_matrice[0][0].setValue(pos.x);

  this->_matrice[1][1].setState(CovarianceMatrice::POSITION);
  this->_matrice[1][1].setValue(pos.y);

  this->_matrice[2][2].setState(CovarianceMatrice::POSITION);
  this->_matrice[2][2].setValue(theta);

  this->_matrice[0][1].setState(CovarianceMatrice::NOTUSED);
  this->_matrice[0][2].setState(CovarianceMatrice::NOTUSED);
  this->_matrice[1][0].setState(CovarianceMatrice::NOTUSED);
  this->_matrice[1][2].setState(CovarianceMatrice::NOTUSED);
  this->_matrice[2][0].setState(CovarianceMatrice::NOTUSED);
  this->_matrice[2][1].setState(CovarianceMatrice::NOTUSED);
}

CovarianceMatrice::CovarianceMatrice(Agent const &agent)
{
    this->_matrice.resize(CovarianceMatrice::SIZEINIT);
  for (unsigned int i = 0 ; i < CovarianceMatrice::SIZEINIT; ++i)
    this->_matrice[i].resize(CovarianceMatrice::SIZEINIT);
  this->_matrice[0][0].setState(CovarianceMatrice::POSITION);
  this->_matrice[0][0].setValue(agent.getPos().x);

  this->_matrice[1][1].setState(CovarianceMatrice::POSITION);
  this->_matrice[1][1].setValue(agent.getPos().y);

  this->_matrice[2][2].setState(CovarianceMatrice::POSITION);
  this->_matrice[2][2].setValue(agent.getBearing());

  this->_matrice[0][1].setState(CovarianceMatrice::NOTUSED);
  this->_matrice[0][2].setState(CovarianceMatrice::NOTUSED);
  this->_matrice[1][0].setState(CovarianceMatrice::NOTUSED);
  this->_matrice[1][2].setState(CovarianceMatrice::NOTUSED);
  this->_matrice[2][0].setState(CovarianceMatrice::NOTUSED);
  this->_matrice[2][1].setState(CovarianceMatrice::NOTUSED);
}

CovarianceMatrice::~CovarianceMatrice()
{}

float CovarianceMatrice::getRobotX() const
{
  return this->_matrice[0][0].getValue();
}

float CovarianceMatrice::getRobotY() const
{
  return this->_matrice[1][1].getValue();
}

float CovarianceMatrice::getRobotTheta() const
{
  return this->_matrice[2][2].getValue();
}

void CovarianceMatrice::setRobotPosition(float X, float Y, float theta)
{
  this->_matrice[0][0].setValue(X);
  this->_matrice[1][1].setValue(Y);
  this->_matrice[2][2].setValue(theta);
}

void CovarianceMatrice::setRobotPosition(pcl::PointXYZ const &pos, float theta)
{
  this->_matrice[0][0].setValue(pos.x);
  this->_matrice[1][1].setValue(pos.y);
  this->_matrice[2][2].setValue(theta);
}

void CovarianceMatrice::setRobotPosition(Agent const &agent)
{
  this->_matrice[0][0].setValue(agent.getPos().x);
  this->_matrice[1][1].setValue(agent.getPos().y);
  this->_matrice[2][2].setValue(agent.getBearing());
}

void CovarianceMatrice::addLandmark(float x, float y, unsigned int slamId)
{
  unsigned int oldSize = this->_matrice.size();
  this->_matrice.resize(oldSize + 2);
  for (unsigned int i = 0; i < oldSize + 2; ++i)
    this->_matrice[i].resize(oldSize + 2);

  // Two cases by landmark
  unsigned int index = (slamId * 2) + CovarianceMatrice::SIZEINIT;
  this->_matrice[index][index].setValue(x);
  this->_matrice[index][index].setState(CovarianceMatrice::POSITION);

  this->_matrice[index + 1][index + 1].setValue(y);
  this->_matrice[index + 1][index + 1].setState(CovarianceMatrice::POSITION);

  this->_matrice[index][index + 1].setState(CovarianceMatrice::NOTUSED);
  this->_matrice[index + 1][index].setState(CovarianceMatrice::NOTUSED);
}

void CovarianceMatrice::addLandmark(pcl::PointXY const &pos, unsigned int slamId)
{
  unsigned int oldSize = this->_matrice.size();
  this->_matrice.resize(oldSize + 2);
  for (unsigned int i = 0; i < oldSize + 2; ++i)
    this->_matrice[i].resize(oldSize + 2);

  // Two cases by landmark
  unsigned int index = (slamId * 2) + CovarianceMatrice::SIZEINIT;
  this->_matrice[index][index].setValue(pos.x);
  this->_matrice[index][index].setState(CovarianceMatrice::POSITION);

  this->_matrice[index + 1][index + 1].setValue(pos.y);
  this->_matrice[index + 1][index + 1].setState(CovarianceMatrice::POSITION);

  this->_matrice[index][index + 1].setState(CovarianceMatrice::NOTUSED);
  this->_matrice[index + 1][index].setState(CovarianceMatrice::NOTUSED);
}

void CovarianceMatrice::calculateRobotCovariance(SystemStateMatrice state, JacobianMatriceA JA)
{
	//Step1
	//Prr = A * Prr * A + Q

	std::vector<double> Prr(9,0);
	Prr.at(0) = JA.getMatrice().at(0) * this->_matrice[0][0].getValue() + JA.getMatrice().at(1) * this->_matrice[1][0].getValue() + JA.getMatrice().at(2) * this->_matrice[2][0].getValue();
	Prr.at(1) = JA.getMatrice().at(0) * this->_matrice[0][1].getValue() + JA.getMatrice().at(1) * this->_matrice[1][1].getValue() + JA.getMatrice().at(2) * this->_matrice[2][1].getValue();
	Prr.at(2) = JA.getMatrice().at(0) * this->_matrice[0][2].getValue() + JA.getMatrice().at(1) * this->_matrice[1][2].getValue() + JA.getMatrice().at(2) * this->_matrice[2][2].getValue();
	Prr.at(3) = JA.getMatrice().at(3) * this->_matrice[0][0].getValue() + JA.getMatrice().at(4) * this->_matrice[1][0].getValue() + JA.getMatrice().at(5) * this->_matrice[2][0].getValue();
	Prr.at(4) = JA.getMatrice().at(3) * this->_matrice[0][1].getValue() + JA.getMatrice().at(4) * this->_matrice[1][1].getValue() + JA.getMatrice().at(5) * this->_matrice[2][1].getValue();
	Prr.at(5) = JA.getMatrice().at(3) * this->_matrice[0][2].getValue() + JA.getMatrice().at(4) * this->_matrice[1][2].getValue() + JA.getMatrice().at(5) * this->_matrice[2][2].getValue();
	Prr.at(6) = JA.getMatrice().at(6) * this->_matrice[0][0].getValue() + JA.getMatrice().at(7) * this->_matrice[1][0].getValue() + JA.getMatrice().at(8) * this->_matrice[2][0].getValue();
	Prr.at(7) = JA.getMatrice().at(6) * this->_matrice[0][1].getValue() + JA.getMatrice().at(7) * this->_matrice[1][1].getValue() + JA.getMatrice().at(8) * this->_matrice[2][1].getValue();
	Prr.at(8) = JA.getMatrice().at(6) * this->_matrice[0][2].getValue() + JA.getMatrice().at(7) * this->_matrice[1][2].getValue() + JA.getMatrice().at(8) * this->_matrice[2][2].getValue();

	std::vector<double> APrrA(9,0);
	APrrA.at(0) = Prr.at(0) * JA.getMatrice().at(0) + Prr.at(1) * JA.getMatrice().at(3) + Prr.at(2) * JA.getMatrice().at(6);
	APrrA.at(1) = Prr.at(0) * JA.getMatrice().at(1) + Prr.at(1) * JA.getMatrice().at(4) + Prr.at(2) * JA.getMatrice().at(7);
	APrrA.at(2) = Prr.at(0) * JA.getMatrice().at(2) + Prr.at(1) * JA.getMatrice().at(5) + Prr.at(2) * JA.getMatrice().at(8);
	APrrA.at(3) = Prr.at(3) * JA.getMatrice().at(0) + Prr.at(4) * JA.getMatrice().at(3) + Prr.at(5) * JA.getMatrice().at(6);
	APrrA.at(4) = Prr.at(3) * JA.getMatrice().at(1) + Prr.at(4) * JA.getMatrice().at(4) + Prr.at(5) * JA.getMatrice().at(7);
	APrrA.at(5) = Prr.at(3) * JA.getMatrice().at(2) + Prr.at(4) * JA.getMatrice().at(5) + Prr.at(5) * JA.getMatrice().at(8);
	APrrA.at(6) = Prr.at(6) * JA.getMatrice().at(0) + Prr.at(7) * JA.getMatrice().at(3) + Prr.at(8) * JA.getMatrice().at(6);
	APrrA.at(7) = Prr.at(6) * JA.getMatrice().at(1) + Prr.at(7) * JA.getMatrice().at(4) + Prr.at(8) * JA.getMatrice().at(7);
	APrrA.at(8) = Prr.at(6) * JA.getMatrice().at(2) + Prr.at(7) * JA.getMatrice().at(5) + Prr.at(8) * JA.getMatrice().at(8);

	//need to add noise Q to APrrA
	this->_matrice[0][0].setValue(APrrA.at(0));
	this->_matrice[0][1].setValue(APrrA.at(1));
	this->_matrice[0][2].setValue(APrrA.at(2));
	this->_matrice[1][0].setValue(APrrA.at(3));
	this->_matrice[1][1].setValue(APrrA.at(4));
	this->_matrice[1][2].setValue(APrrA.at(5));
	this->_matrice[2][0].setValue(APrrA.at(6));
	this->_matrice[2][1].setValue(APrrA.at(7));
	this->_matrice[2][2].setValue(APrrA.at(8));

	//Pri = A * Pri
	std::vector<double> APri(9,0);
	for (unsigned int j = 3; j < this->_matrice.size() - 2; j += 3)
	{
		APri.at(0) = JA.getMatrice().at(0) * this->_matrice[0][j].getValue() + JA.getMatrice().at(1) * this->_matrice[1][j].getValue() + JA.getMatrice().at(2) * this->_matrice[2][j].getValue();
		APri.at(1) = JA.getMatrice().at(0) * this->_matrice[0][j+1].getValue() + JA.getMatrice().at(1) * this->_matrice[1][j+1].getValue() + JA.getMatrice().at(2) * this->_matrice[2][j+1].getValue();
		APri.at(2) = JA.getMatrice().at(0) * this->_matrice[0][j+2].getValue() + JA.getMatrice().at(1) * this->_matrice[1][j+2].getValue() + JA.getMatrice().at(2) * this->_matrice[2][j+2].getValue();
		APri.at(3) = JA.getMatrice().at(3) * this->_matrice[0][j].getValue() + JA.getMatrice().at(4) * this->_matrice[1][j].getValue() + JA.getMatrice().at(5) * this->_matrice[2][j].getValue();
		APri.at(4) = JA.getMatrice().at(3) * this->_matrice[0][j+1].getValue() + JA.getMatrice().at(4) * this->_matrice[1][j+1].getValue() + JA.getMatrice().at(5) * this->_matrice[2][j+1].getValue();
		APri.at(5) = JA.getMatrice().at(3) * this->_matrice[0][j+2].getValue() + JA.getMatrice().at(4) * this->_matrice[1][j+2].getValue() + JA.getMatrice().at(5) * this->_matrice[2][j+2].getValue();
		APri.at(6) = JA.getMatrice().at(6) * this->_matrice[0][j].getValue() + JA.getMatrice().at(7) * this->_matrice[1][j].getValue() + JA.getMatrice().at(8) * this->_matrice[2][j].getValue();
		APri.at(7) = JA.getMatrice().at(6) * this->_matrice[0][j+1].getValue() + JA.getMatrice().at(7) * this->_matrice[1][j+1].getValue() + JA.getMatrice().at(8) * this->_matrice[2][j+1].getValue();
		APri.at(8) = JA.getMatrice().at(6) * this->_matrice[0][j+2].getValue() + JA.getMatrice().at(7) * this->_matrice[1][j+2].getValue() + JA.getMatrice().at(8) * this->_matrice[2][j+2].getValue();

		this->_matrice[0][j].setValue(APri.at(0));
		this->_matrice[1][j].setValue(APri.at(1));
		this->_matrice[2][j].setValue(APri.at(2));
		this->_matrice[0][j+1].setValue(APri.at(3));
		this->_matrice[1][j+1].setValue(APri.at(4));
		this->_matrice[2][j+1].setValue(APri.at(5));
		this->_matrice[0][j+2].setValue(APri.at(6));
		this->_matrice[1][j+2].setValue(APri.at(7));
		this->_matrice[2][j+2].setValue(APri.at(8));
	}

}

void CovarianceMatrice::calculationCovariance()
{
  for (unsigned int i = 3; i < this->_matrice.size(); ++i)
    {
      for (unsigned int j = 3; j < this->_matrice.size(); ++j)
	{
	  if (this->_matrice[i][j].getState() != CovarianceMatrice::CALCULATION)
	    continue;

	  float firstValue;
	  float secondValue;

	  for (unsigned int x = 0; x < this->_matrice.size(); ++x)
	    {
	      if(this->_matrice[i][x].getState() == CovarianceMatrice::POSITION)
		{
		  firstValue = this->_matrice[i][x].getValue();
		  break;
		}
	    }

	  for (unsigned int x = 0; x < this->_matrice.size(); ++x)
	    {
	      if(this->_matrice[x][j].getState() == CovarianceMatrice::POSITION)
		{
		  secondValue = this->_matrice[x][j].getValue();
		  break;
		}
	    }

	  if(i > j) // different formula
	    {
	      // partie du bas de la matrice
	      // pas la bonne formule, a trouver la vrai
	      this->_matrice[i][j].setValue(firstValue + secondValue);
	    }
	  else // different formula
	    {
	      // partie du haut de la matrice
	      // pareillement
	      this->_matrice[i][j].setValue(firstValue - secondValue);
	    }
	}
    }
}
