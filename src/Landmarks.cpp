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
  std::vector<std::pair<int, int> > idCpy(MAXLANDMARKS);
  std::vector<Landmarks::Landmark *> landmarkCpy(MAXLANDMARKS);
  this->DBSize = 0;
  this->EKFLandmarks = 0;
  this->degreePerScan = degrees;
  this->IDtoID = idCpy;
  this->landmarkDB = landmarkCpy;
}

Landmarks::~Landmarks()
{
  for (unsigned int i = 0; i < this->landmarkDB.size(); ++i)
    {
      delete (this->landmarkDB[i]);
    }
}

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
  newSlamID.second = slamID;
  this->IDtoID[EKFLandmarks] = newSlamID;
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
  return (-1);
}

double Landmarks::distance(double x1, double y1, double x2, double y2) const
{
  return (sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2)));
}

double Landmarks::distance(const Landmark &lm1, const Landmark &lm2) const
{
  return (sqrt(pow(lm1.pos[0] - lm2.pos[0], 2) + pow(lm1.pos[1] - lm2.pos[1], 2)));
}

int Landmarks::addToDB(const Landmark &lm)
{
  Landmarks::Landmark	*new_elem;

  if(static_cast<unsigned int>(DBSize + 1) < this->landmarkDB.size())
    {
      new_elem = new Landmarks::Landmark();

      new_elem->pos[0] = lm.pos[0];
      new_elem->pos[1] = lm.pos[1];
      new_elem->life = LIFE;
      new_elem->id = DBSize;
      new_elem->totalTimeObserved = 1;
      new_elem->bearing = lm.bearing;
      new_elem->range = lm.range;
      new_elem->a = lm.a;
      new_elem->b = lm.b;

      landmarkDB[DBSize] = new_elem;
      ++DBSize;
      return (DBSize - 1);
    }
  return (-1);
}

double Landmarks::distanceToLine(double x, double y, double a, double b)
{
  // calculate point on line closest to (x, y)
  // use this point to calculate distance between them

  double ao = -1.0 / a;
  double bo = y - ao * x;
  double px = (b - bo) / (ao - a);
  double py = ((ao * (b - bo)) / (ao - a)) + bo;
  return (this->distance(x, y, px, py));
}

std::vector<Landmarks::Landmark *> Landmarks::getLandmarkDB() const
{
  std::vector<Landmark *> res(this->DBSize);

  for (int i = 0; i < this->DBSize; ++i)
    res[i] = new Landmarks::Landmark(*this->landmarkDB[i]);
  return (res);
}

void Landmarks::getClosestAssociation(Landmark *lm, int &id, int &totalTimeObserved)
{
  int closestLandmark = 0;
  double temp;
  double leastDistance = 99999; // 100k meter, big distance

  for(int i = 0; i < DBSize; ++i)
    {
      if(static_cast<unsigned int>(landmarkDB[i]->totalTimeObserved) > MINOBSERVATIONS)
	{
	  temp = this->distance(*lm, *landmarkDB[i]);
	  if(temp < leastDistance)
	    {
	      leastDistance = temp;
	      closestLandmark = landmarkDB[i]->id;
	    }
	}
    }
  if(leastDistance == 99999)
    id = -1;
  else
    {
      id = landmarkDB[closestLandmark]->id;
      totalTimeObserved = landmarkDB[closestLandmark]->totalTimeObserved;
    }
}

void Landmarks::leastSquaresLineEstimate(double cameradata[], double robotPosition[], int selectPoints[], int arraySize, double &a, double &b)
{
  double y; //y coordinate
  double x; //x coordinate
  double sumY = 0; //sum of y coordinates
  double sumYY = 0; //sum of y^2 for each coordinate
  double sumX = 0; //sum of x coordinates
  double sumXX = 0; //sum of x^2 for each coordinate
  double sumYX = 0; //sum of y*x for each point

  for(int i = 0; i < arraySize; ++i)
    {
      //convert ranges and bearing to coordinates
      x = (cos((selectPoints[i] * this->degreePerScan * CONVERSION) + robotPosition[2] * CONVERSION) * cameradata[selectPoints[i]]) + robotPosition[0];
      y = (sin((selectPoints[i] * this->degreePerScan * CONVERSION) + robotPosition[2] * CONVERSION) * cameradata[selectPoints[i]]) + robotPosition[1];
      sumY += y;
      sumYY += pow(y,2);
      sumX += x;
      sumXX += pow(x,2);
      sumYX += y * x;
    }
  b = (sumY * sumXX - sumX * sumYX) / (arraySize * sumXX - pow(sumX, 2));
  a = (arraySize * sumYX - sumX * sumY) / (arraySize * sumXX - pow(sumX, 2));
}

/**
 * Need to do again, and add some comment
 */
Landmarks::Landmark *Landmarks::getLandmark(double range, int readingNo, double robotPosition[])
{
  Landmarks::Landmark *lm = new Landmarks::Landmark();
  int id = -1;
  int totalTimeObserved = 0;

  lm->pos[0] = (cos((readingNo * this->degreePerScan * CONVERSION) +
		    (robotPosition[2] * CONVERSION)) * range) + robotPosition[0];
  lm->pos[1] = (sin((readingNo * this->degreePerScan * CONVERSION) +
		    (robotPosition[2] * CONVERSION)) * range) + robotPosition[1];
  lm->range = range;
  lm->bearing = readingNo;
  //Possiblement envoyé une exception si on ne trouve pas de landmark, sinon ça risque de poser problème
  this->getClosestAssociation(lm, id, totalTimeObserved);
  lm->id = id;
  return (lm);
}

Landmarks::Landmark *Landmarks::updateLandmark(Landmarks::Landmark *lm)
{
  int newId = this->getAssociation(*lm);

  if (newId == -1)
    newId = this->addToDB(*lm);

  lm->id = newId;
  return (lm);
}

// @TODO: correct it (the part when it does not exist, not the other one)
Landmarks::Landmark *Landmarks::updateLandmark(bool matched, int id, double distance, double readingNo, double robotPosition[])
{
  Landmarks::Landmark *lm;

  if (matched && this->landmarkDB.size() > static_cast<unsigned int>(id))
    {
      // it exists in the DB
      ++this->landmarkDB[id]->totalTimeObserved;
      lm = this->landmarkDB[id];
    }
  else
    {
      // doesn't exist in the DB/fail to matched, so that, we've to add this sample
      lm = new Landmarks::Landmark();

      lm->pos[0] = cos((readingNo * this->degreePerScan * CONVERSION) +
		       (robotPosition[2] * CONVERSION)) * distance + robotPosition[0];
      lm->pos[1] = sin((readingNo * this->degreePerScan * CONVERSION) +
		       (robotPosition[2] * CONVERSION)) * distance + robotPosition[1];
      lm->bearing = readingNo;
      lm->range = distance;
      lm->id = this->addToDB(*lm);
    }
  return (lm);
}

int Landmarks::updateLineLandmark(Landmark &lm) // cannot be const, getassociation modify it
{
  int id = this->getAssociation(lm);

  if (id == -1)
    id = this->addToDB(lm);
  return (id);
}

Landmarks::Landmark *Landmarks::getOrigin()
{
  Landmarks::Landmark *lm = new Landmarks::Landmark();
  int id = -1;
  int totalTimesObserved = 0;

  lm->pos[0] = 0;
  lm->pos[1] = 0;
  lm->range = -1;
  lm->bearing = -1;

  //associate landmark to closest landmark.
  this->getClosestAssociation(lm, id, totalTimesObserved);
  lm->id = id;
  return (lm);
}

Landmarks::Landmark *Landmarks::getLine(double a, double b)
{
  Landmarks::Landmark *lm = new Landmarks::Landmark();

  //our goal is to calculate point on line closest to origin (0,0)
  //calculate line perpendicular to input line. a*ao = -1
  double ao = -1.0 / a;
  double x = b / (ao - a);

  //get intersection between y = ax + b and y = aox
  //so aox = ax + b => aox - ax = b => x = b/(ao - a), y = ao*b/(ao - a)
  double y = (ao * b) / (ao - a);
  int id = -1;
  int totalTimesObserved = 0;

  //convert landmark to map coordinate
  lm->pos[0] =x;
  lm->pos[1] =y;
  lm->range = -1;
  lm->bearing = -1;
  lm->a = a;
  lm->b = b;
  //associate landmark to closest landmark.
  this->getClosestAssociation(lm, id, totalTimesObserved);
  lm->id = id;

  return (lm);
}

Landmarks::Landmark *Landmarks::getLineLandmark(double a, double b, double robotPosition[])
{
  //our goal is to calculate point on line closest to origin (0,0)
  //calculate line perpendicular to input line. a*ao = -1
  double ao = -1.0 / a;
  //landmark position
  double x = b / (ao - a);
  double y = (ao * b) / (ao - a);
  double range = sqrt(pow(x - robotPosition[0], 2) + pow(y - robotPosition[1], 2));
  double bearing = atan((y - robotPosition[1]) / (x-robotPosition[0])) - robotPosition[2];
  //now do same calculation but get point on wall closest to robot instead
  //y = aox + bo => bo = y - aox
  double bo = robotPosition[1] - ao * robotPosition[0];
  //get intersection between y = ax + b and y = aox + bo
  //so aox + bo = ax + b => aox - ax = b - bo => x = (b - bo)/(ao - a), y = ao*(b - bo)/(ao - a) + bo
  double px = (b - bo) / (ao - a);
  double py = ((ao * (b - bo)) / (ao - a)) + bo;
  double rangeError = this->distance(robotPosition[0], robotPosition[1], px, py);
  double bearingError = atan((py - robotPosition[1]) / (px - robotPosition[0]))
    - robotPosition[2];  //do you subtract or add robot bearing? I am not sure!

  Landmarks::Landmark *lm = new Landmarks::Landmark();
  int id = 0;
  int totalTimesObserved = 0;

  //convert landmark to map coordinate
  lm->pos[0] = x;
  lm->pos[1] =y;
  lm->range = range;
  lm->bearing = bearing;
  lm->a = a;
  lm->b = b;
  lm->rangeError = rangeError;
  lm->bearingError = bearingError;

  //associate landmark to closest landmark.
  this->getClosestAssociation(lm, id, totalTimesObserved);
  lm->id = id;
  lm->totalTimeObserved = totalTimesObserved;

  return (lm);
}

std::vector<Landmarks::Landmark *> Landmarks::extractSpikeLandmarks(double cameradata[], unsigned int sampleNumber, double robotPosition[])
{
  //have a large array to keep track of found landmarks
  std::vector<Landmarks::Landmark *> tempLandmarks;
  for(unsigned int i = 0; i < 400; ++i)
    tempLandmarks.push_back(new Landmarks::Landmark());
  for (unsigned int i = 1; i < sampleNumber - 1 /* == cameradata.Length - 1 */; i++)
    {
      // Check for error measurement in laser data
      if (cameradata[i - 1] < 8.1 && cameradata[i + 1] < 8.1)
	{
  	  if ((cameradata[i - 1] - cameradata[i]) + (cameradata[i + 1] - cameradata[i]) > 0.5)
  	    tempLandmarks[i] = this->getLandmark(cameradata[i], i, robotPosition);
  	  else
  	    if((cameradata[i - 1] - cameradata[i]) > 0.3)
	      tempLandmarks[i] = this->getLandmark(cameradata[i], i, robotPosition);
  	    else if (cameradata[i + 1] < 8.1)
  	      if((cameradata[i + 1] - cameradata[i]) > 0.3)
  		tempLandmarks[i] = this->getLandmark(cameradata[i], i, robotPosition);
	}
    }

  // copy landmarks into another vector
  std::vector<Landmarks::Landmark *> foundLandmarks;
  for(unsigned int i = 0; i < tempLandmarks.size(); ++i)
    {
      if(((int)tempLandmarks[i]->id) != -1)
	foundLandmarks.push_back(new Landmarks::Landmark(*tempLandmarks[i]));
      delete (tempLandmarks[i]);
    }
  return (foundLandmarks);
}

std::vector<Landmarks::Landmark *> Landmarks::removeDouble(std::vector<Landmarks::Landmark *> extractedLandmarks)
{
  // int uniquelmrks = 0;
  double leastDistance = 99999;
  double temp;
  std::vector<Landmarks::Landmark *> uniqueLandmarks;
  for(unsigned int i = 0; i < extractedLandmarks.size(); ++i)
    {
      //remove landmarks that didn't get associated and also pass
      //landmarks through our temporary landmark validation gate
      if(extractedLandmarks[i]->id != -1 && this->getAssociation(*extractedLandmarks[i]) != -1)
	{
	  //remove doubles in extractedLandmarks
	  //if two observations match same landmark, take closest landmark

	  leastDistance = 99999;
	  for(unsigned int j = 0; j < extractedLandmarks.size(); ++j)
	    {
	      if(extractedLandmarks[i]->id == extractedLandmarks[j]->id)
		{
		  if (j < i)
		    break;
		  temp = this->distance(*extractedLandmarks[j], *landmarkDB[extractedLandmarks[j]->id]);
		  if(temp < leastDistance)
		    {
		      leastDistance = temp;
		      // NOT SURE
		      uniqueLandmarks.push_back(extractedLandmarks[j]);
		      //uniqueLandmarks[uniquelmrks] = extractedLandmarks[j];
		    }
		}
	    }
	}
      // NOTE SURE
      // if (leastDistance != 99999)
      // 	++uniquelmrks;
    }
  return (uniqueLandmarks);
  //copy landmarks over into an array of correct dimensions

  // NOT SURE
  // extractedLandmarks = new landmark[uniquelmrks];
  // for(int i = 0; i < uniquelmrks; ++i)
  //   extractedLandmarks[i] = uniqueLandmarks[i];
  // return extractedLandmarks;
}
