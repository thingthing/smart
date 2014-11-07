#include "Landmarks.hh"

Landmarks::Landmark::Landmark()
{
  this->pos[0] = 0.0;
  this->pos[1] = 0.0;
  this->id = -1;
  this->life = LIFE;
  this->totalTimeObserved = 0;
  this->range = -1;
  this->bearing = -1;
}

Landmarks::Landmark::~Landmark()
{}

Landmarks::Landmarks(double degrees)
{
  this->DBSize = 0;
  this->EKFLandmarks = 0;
  this->degreePerScan = degrees;
  this->IDtoID.reserve(MAXLANDMARKS);
  this->landmarkDB.reserve(MAXLANDMARKS);
}

Landmarks::~Landmarks()
{}

int Landmarks::getDBSize() const
{
  return (this->DBSize);
}

int Landmarks::getSLamId(int id) const
{
  for(int i = 0; i < this->EKFLandmarks; ++i)
    {
      if(IDtoID[i].first == id)
	return (IDtoID[i].second);
    }
  return (-1);
}

int Landmarks::addSlamId(int landmarkID, int slamID)
{
  std::pair<int, int> newSlamID;
  newSlamID.first = landmarkID;
  newSlamID.first = slamID;
  this->IDtoID.push_back(newSlamID);
  ++this->EKFLandmarks;
  return (0);
}

int Landmarks::getAssociation(Landmark &lm)
{
  for(int i = 0; i < this->DBSize; ++i)
    {
      if(this->distance(lm, (*landmarkDB[i])) < MAXERROR && landmarkDB[i]->id != -1)
	{
	  landmarkDB[i]->life = LIFE;
	  ++landmarkDB[i]->totalTimeObserved;
	  landmarkDB[i]->bearing = lm.bearing;
	  landmarkDB[i]->range = lm.range;
	  return (landmarkDB[i]->id);
	}
    }
  return -1;
}

double Landmarks::distance(double x1, double y1, double x2, double y2) const
{
  return (sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2)));
}

double Landmarks::distance(const Landmark &lm1, const Landmark &lm2) const
{
  return (sqrt(pow(lm1.pos[0] - lm2.pos[0], 2) + pow(lm1.pos[1] - lm2.pos[1], 2)));
}
