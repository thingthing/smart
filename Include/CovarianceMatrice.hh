#ifndef COVARIANCEMATRICE_H_
# define COVARIANCEMATRICE_H_

# include <vector>

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
    double getValue() const;
    void setValue(double value);
    State getState() const;
    void setState(State state);

  protected:
    double _value;
    State _state;
  };

public:
  CovarianceMatrice();
  CovarianceMatrice(double X, double Y, double theta);
  virtual ~CovarianceMatrice();

  void addLandmark(double x, double y);
  double getRobotX() const;
  double getRobotY() const;
  double getRobotTheta() const;
  void setRobotPosition(double X, double Y, double theta);
  void calculationCovariance();

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
