#include "CovarianceMatrice.hh"

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

void CovarianceMatrice::Case::setValue(double value)
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

double CovarianceMatrice::Case::getValue() const
{
  return this->_value;
}

CovarianceMatrice::CovarianceMatrice()
{
  this->_matrice.resize(3);
  for (unsigned int i = 0 ; i < 3; ++i)
    this->_matrice[i].resize(3);
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

CovarianceMatrice::CovarianceMatrice(double X, double Y, double theta)
{
  this->_matrice.resize(3);
  for (unsigned int i = 0 ; i < 3; ++i)
    this->_matrice[i].resize(3);
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

CovarianceMatrice::~CovarianceMatrice()
{}

double CovarianceMatrice::getRobotX() const
{
  return this->_matrice[0][0].getValue();
}

double CovarianceMatrice::getRobotY() const
{
  return this->_matrice[1][1].getValue();
}

double CovarianceMatrice::getRobotTheta() const
{
  return this->_matrice[2][2].getValue();
}

void CovarianceMatrice::addLandmark(double x, double y)
{
  unsigned int oldSize = this->_matrice.size();
  this->_matrice.resize(oldSize + 2);
  for (unsigned int i = 0; i < oldSize + 2; ++i)
    this->_matrice[i].resize(oldSize + 2);

  this->_matrice[oldSize][oldSize].setValue(x);
  this->_matrice[oldSize][oldSize].setState(CovarianceMatrice::POSITION);

  this->_matrice[oldSize + 1][oldSize + 1].setValue(y);
  this->_matrice[oldSize + 1][oldSize + 1].setState(CovarianceMatrice::POSITION);

  this->_matrice[oldSize][oldSize + 1].setState(CovarianceMatrice::NOTUSED);
  this->_matrice[oldSize + 1][oldSize].setState(CovarianceMatrice::NOTUSED);
}

void CovarianceMatrice::calculationCovariance()
{
  for (unsigned int i = 3; i < this->_matrice.size(); ++i)
    {
      for (unsigned int j = 3; j < this->_matrice.size(); ++j)
	{
	  if (this->_matrice[i][j].getState() != CovarianceMatrice::CALCULATION)
	    continue;

	  double firstValue;
	  double secondValue;

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
