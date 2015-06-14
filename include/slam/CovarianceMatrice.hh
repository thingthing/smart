#ifndef COVARIANCEMATRICE_H_
# define COVARIANCEMATRICE_H_

#include <pcl/common/common.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/common/projection_matrix.h>

#include <vector>
#include "Agent.hh"
#include "SystemStateMatrice.hh"
#include "JacobianMatriceA.hh"
#include "JacobianMatriceJz.hh"
#include "JacobianMatriceJxr.hh"

#define RobotID -1

class CovarianceMatrice
{
  enum State
    {
      NOTUSED,
      POSITION,
      CALCULATION
    };

  class Case
  {
  public:
    Case();
    Case(State state);
    virtual ~Case();
    float getValue() const;
    void setValue(float value);
		int getSlamID() const;
    void setSlamID(int slamID);
    State getState() const;
    void setState(State state);

  protected:
    float _value;
    State _state;
		//id for the robot is -1
		int _slamID;
  };


private:
  static const unsigned int SIZEINIT;

public:
  CovarianceMatrice();
  CovarianceMatrice(float X, float Y, float theta);
  CovarianceMatrice(pcl::PointXYZ const &pos, float theta);
  CovarianceMatrice(Agent const &agent);

  virtual ~CovarianceMatrice();

  void addLandmark(float x, float y, int slamId);
  void addLandmark(pcl::PointXY const &pos, int slamId);

  float getRobotX() const;
  float getRobotY() const;
  float getRobotTheta() const;

  void setRobotPosition(float X, float Y, float theta);
  void setRobotPosition(pcl::PointXYZ const &pos, float theta);
  void setRobotPosition(Agent const &agent);
  void calculationCovariance();
  void step1RobotCovariance(JacobianMatriceA JA);
	void step3Covariance(JacobianMatriceJxr Jxr, JacobianMatriceJz Jz, SystemStateMatrice stateM, int slamID);

private:
  CovarianceMatrice(const CovarianceMatrice &);
  CovarianceMatrice &operator=(const CovarianceMatrice &);
protected:
  std::vector< std::vector<Case> > _matrice;

  /*

    x in the matrice => not used
    p => position

     robot|l1 |l2 |l3 |l4 |...
     x y t x y x y x y x y
    ______________________
r  x|p.x.x|0.0|0.0|0.0|0.0
b  y|x.p.x|0.0|0.0|0.0|0.0
t  t|x.x.p|0.0|0.0|0.0|0.0
    |-----|---|---|---|---
l  x|0 0 0|p.x|0.0|0.0|0.0
1  y|0.0.0|x.p|0.0|0.0|0.0
    |-----|---|---|---|---
l  x|0.0.0|0.0|p.x|0.0|0.0
2  y|0.0.0|0.0|x.p|0.0|0.0
    |-----|---|---|---|---
l  x|0.0.0|0.0|0.0|p.x|0.0
3  y|0.0.0|0.0|0.0|x.p|0.0
    |-----|---|---|---|---
l  x|0.0.0|0.0|0.0|0.0|p.x
4  y|0.0.0|0.0|0.0|0.0|x.p


  */
};

#endif /* !COVARIANCEMATRICE_H_ */
