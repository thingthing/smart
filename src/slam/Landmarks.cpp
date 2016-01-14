#include <cstdlib>
#include <algorithm>
#include "Landmarks.hh"

const double Landmarks::CONVERSION = (M_PI / 180.0); // Convert to radians
const unsigned int Landmarks::MAXLANDMARKS = 3000; // Max number of landmarks
const double Landmarks::MAXERROR = 0.5; // If a landmarks is within this distance of another landmarks, its the same landmarks
const unsigned int Landmarks::MINOBSERVATIONS = 15; // Number of times a landmark must be observed to be recongnized as a landmark
const unsigned int Landmarks::LIFE = 40; // Use to reset life counter (counter use to determine whether to discard a landmark or not)
const unsigned int Landmarks::MAXTRIALS = 100; // RANSAC: max times to run algorithm
const unsigned int Landmarks::MAXSAMPLE = 10; // RANSAC: randomly select x points
const unsigned int Landmarks::MINLINEPOINTS = 30; // RANSAC: if less than x points left, don't bother trying to find a consensus (stop algorithm)
const double  Landmarks::RANSAC_TOLERANCE = 0.05; // RANSAC: if point is within x distance of line, its part of the line
const unsigned int Landmarks::RANSAC_CONSENSUS = 30; // RANSAC: at leat x votes required to determine if its a line
const double Landmarks::MAX_DIFFERENCE = 0.5; // meter
const double Landmarks::MIN_DIFFERENCE = 0.3; // meter

Landmarks::Landmark::Landmark()
{
  this->pos = pcl::PointXYZ(0.0, 0.0, 0.0);
  this->id = -1;
  this->life = Landmarks::LIFE;
  this->totalTimeObserved = 0;
  this->range = -1;
  this->bearing = -1;
  this->robotPos = pcl::PointXYZ(0.0, 0.0, 0.0);
}

Landmarks::Landmark::~Landmark()
{}

Landmarks::Landmarks(double degrees)
{
  std::vector<std::pair<int, int> > idCpy(Landmarks::MAXLANDMARKS);
  std::vector<Landmarks::Landmark *> landmarkCpy(Landmarks::MAXLANDMARKS);
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

unsigned int Landmarks::getDBSize() const
{
  return (this->DBSize);
}

int Landmarks::getSLamId(int id) const
{
  for (int i = 0; i < this->EKFLandmarks; ++i)
  {
    if (IDtoID[i].first == id)
      return (IDtoID[i].second);
  }
  return (-1);
}

void Landmarks::addSlamId(int landmarkID, int slamID)
{
  std::pair<int, int> newSlamID;

  newSlamID.first = landmarkID;
  newSlamID.second = slamID;
  this->IDtoID[EKFLandmarks] = newSlamID;
  ++this->EKFLandmarks;
}

///@todo: Should add innovation to getAssociation one day (after matrice implementation)
int Landmarks::getAssociation(Landmark &lm)
{
  for (int i = 0; i < this->DBSize; ++i)
  {
    double dist = this->distance(lm, (*landmarkDB[i]));

    if (dist < Landmarks::MAXERROR && landmarkDB[i]->id != -1)
    {
      landmarkDB[i]->life = Landmarks::LIFE;
      ++landmarkDB[i]->totalTimeObserved;
      landmarkDB[i]->bearing = lm.bearing;
      landmarkDB[i]->pos.x = lm.pos.x;
      landmarkDB[i]->pos.y = lm.pos.y;
      landmarkDB[i]->range = lm.range;
      landmarkDB[i]->robotPos = lm.robotPos;
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
  return (sqrt(pow(lm1.pos.x - lm2.pos.x, 2) + pow(lm1.pos.y - lm2.pos.y, 2)));
}

int Landmarks::addToDB(const Landmark &lm)
{
  Landmarks::Landmark *new_elem;

  if (static_cast<unsigned int>(DBSize + 1) < this->landmarkDB.size())
  {
    new_elem = new Landmarks::Landmark();

    new_elem->pos.x = lm.pos.x;
    new_elem->pos.y = lm.pos.y;
    new_elem->life = Landmarks::LIFE;
    new_elem->id = DBSize;
    new_elem->totalTimeObserved = 1;
    new_elem->bearing = lm.bearing;
    new_elem->range = lm.range;
    new_elem->robotPos = lm.robotPos;
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

  for (int i = 0; i < DBSize; ++i)
  {
    if (static_cast<unsigned int>(landmarkDB[i]->totalTimeObserved) > MINOBSERVATIONS)
    {
      temp = this->distance(*lm, *landmarkDB[i]);
      if (temp < leastDistance)
      {
        leastDistance = temp;
        closestLandmark = landmarkDB[i]->id;
      }
    }
  }
  if (leastDistance == 99999)
    id = -1;
  else
  {
    id = landmarkDB[closestLandmark]->id;
    totalTimeObserved = landmarkDB[closestLandmark]->totalTimeObserved;
  }
}

// Parametric convert equation:
// x = (x_view * cos(bearing) - y_view * sin(bearing)) + agentx
// y = (x_view * sin(bearing) + y_view * cos(bearing)) + agenty
static void parametricConvert(IAgent const *agent, double x_view, double y_view, double &x, double &y)
{
  x = (cos(agent->getYaw() * Landmarks::CONVERSION) * x_view - sin(agent->getYaw() * Landmarks::CONVERSION) * y_view) + agent->getPos().x;
  y = (sin(agent->getYaw() * Landmarks::CONVERSION) * x_view + cos(agent->getYaw() * Landmarks::CONVERSION) * y_view) + agent->getPos().y;
}

double Landmarks::calculateBearing(double x, double y, IAgent const *agent) const
{
  return (atan((y - agent->getPos().y) / (x - agent->getPos().x)) - agent->getYaw());
}

void Landmarks::leastSquaresLineEstimate(pcl::PointCloud<pcl::PointXYZRGBA> const &cloud, IAgent const *agent, int selectPoints[], int arraySize, double &a, double &b)
{
  double y; //y coordinate
  double x; //x coordinate
  double sumY = 0; //sum of y coordinates
  double sumYY = 0; //sum of y^2 for each coordinate
  double sumX = 0; //sum of x coordinates
  double sumXX = 0; //sum of x^2 for each coordinate
  double sumYX = 0; //sum of y*x for each point

  for (int i = 0; i < arraySize; ++i)
  {
    //convert ranges and bearing to coordinates
    parametricConvert(agent, cloud.points[selectPoints[i]].x, cloud.points[selectPoints[i]].y, x, y);
    sumY += y;
    sumYY += pow(y, 2);
    sumX += x;
    sumXX += pow(x, 2);
    sumYX += y * x;
  }
  b = (sumY * sumXX - sumX * sumYX) / (arraySize * sumXX - pow(sumX, 2));
  a = (arraySize * sumYX - sumX * sumY) / (arraySize * sumXX - pow(sumX, 2));
}

/**
 * @todo Do not use range only, use point (maybe)
 */
Landmarks::Landmark *Landmarks::getLandmark(double x_view, double y_view, IAgent const *agent)
{
  Landmarks::Landmark *lm = new Landmarks::Landmark();
  int id = -1;
  int totalTimeObserved = 0;
  double x;
  double y;

  parametricConvert(agent, x_view, y_view, x, y);
  lm->pos.x = x;
  lm->pos.y = y;
  lm->range = this->distance(x, y, agent->getPos().x, agent->getPos().y);
  lm->bearing = this->calculateBearing(x, y, agent);
  lm->robotPos = agent->getPos();
  ///@todo: Maybe throw and exception if no landmarks can be associated
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

/**
 * @todo: Same as getLandmark
 **/
Landmarks::Landmark *Landmarks::updateLandmark(bool matched, int id, double x_view, double y_view, IAgent const *agent)
{
  Landmarks::Landmark *lm;
  double x;
  double y;

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
    parametricConvert(agent, x_view, y_view, x, y);
    lm->pos.x = x;
    lm->pos.y = y;
    lm->range = this->distance(x, y, agent->getPos().x, agent->getPos().y);
    lm->bearing = this->calculateBearing(x, y, agent);
    lm->robotPos = agent->getPos();
    lm->id = this->addToDB(*lm);
  }
  return (lm);
}

// cannot be const, getassociation modify it
int Landmarks::updateLineLandmark(Landmark &lm)
{
  int id = this->getAssociation(lm);

  if (id == -1)
    id = this->addToDB(lm);
  return (id);
}

// Function use for debug: Add origin as landmark to check other landmark
Landmarks::Landmark *Landmarks::getOrigin()
{
  Landmarks::Landmark *lm = new Landmarks::Landmark();
  int id = -1;
  int totalTimesObserved = 0;

  // Already default landmark value
  // lm->pos.x = 0.0;
  // lm->pos.y = 0.0;
  // lm->range = -1;
  // lm->bearing = -1;

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
  lm->pos.x = x;
  lm->pos.y = y;
  // Already default value
  // lm->range = -1;
  // lm->bearing = -1;
  lm->a = a;
  lm->b = b;
  //associate landmark to closest landmark.
  this->getClosestAssociation(lm, id, totalTimesObserved);
  lm->id = id;

  return (lm);
}

Landmarks::Landmark *Landmarks::getLineLandmark(double a, double b, IAgent const *agent)
{
  //our goal is to calculate point on line closest to origin (0,0)
  //calculate line perpendicular to input line. a*ao = -1
  double ao = -1.0 / a;
  //landmark position
  double x = b / (ao - a);
  double y = (ao * b) / (ao - a);
  double range = this->distance(x, y, agent->getPos().x, agent->getPos().y);
  double bearing = this->calculateBearing(x, y, agent);
  //now do same calculation but get point on wall closest to robot instead
  //y = aox + bo => bo = y - aox
  double bo = agent->getPos().y - ao * agent->getPos().x;
  //get intersection between y = ax + b and y = aox + bo
  //so aox + bo = ax + b => aox - ax = b - bo => x = (b - bo)/(ao - a), y = ao*(b - bo)/(ao - a) + bo
  double px = (b - bo) / (ao - a);
  double py = ((ao * (b - bo)) / (ao - a)) + bo;
  double rangeError = this->distance(agent->getPos().x, agent->getPos().y, px, py);
  double bearingError = this->calculateBearing(px, py, agent);
  Landmarks::Landmark *lm = new Landmarks::Landmark();
  int id = 0;
  int totalTimesObserved = 0;

  //convert landmark to map coordinate
  lm->pos.x = x;
  lm->pos.y = y;
  lm->range = range;
  lm->bearing = bearing;
  lm->a = a;
  lm->b = b;
  lm->robotPos = agent->getPos();
  lm->rangeError = rangeError;
  lm->bearingError = bearingError;

  //associate landmark to closest landmark.
  this->getClosestAssociation(lm, id, totalTimesObserved);
  lm->id = id;
  lm->totalTimeObserved = totalTimesObserved;

  return (lm);
}

std::vector<Landmarks::Landmark *> Landmarks::extractSpikeLandmarks(pcl::PointCloud<pcl::PointXYZRGBA> const &cloud, IAgent const *agent)
{
  unsigned int sampleNumber = cloud.points.size();

  //have a large array to keep track of found landmarks
  std::vector<Landmarks::Landmark *> tempLandmarks;
  for (unsigned int i = 0; i < sampleNumber; ++i)
    tempLandmarks.push_back(new Landmarks::Landmark());

  double rangeBefore = this->distance(cloud.points[0].x, cloud.points[0].y, agent->getPos().x, agent->getPos().y);
  double range = this->distance(cloud.points[1].x, cloud.points[1].y, agent->getPos().x, agent->getPos().y);
  for (unsigned int i = 1; i < sampleNumber - 1; i++)
  {
    double rangeAfter = this->distance(cloud.points[i + 1].x, cloud.points[i + 1].y, agent->getPos().x, agent->getPos().y);
    //@TODO: Change to use true camera mesurement issue
    // Check for error measurement in laser data
    if (rangeBefore < agent->cameraProblem && rangeAfter < agent->cameraProblem)
    {
      if ((rangeBefore - range) + (rangeAfter - range) > Landmarks::MAX_DIFFERENCE)
        tempLandmarks[i] = this->getLandmark(cloud.points[i].x, cloud.points[i].y, agent);
      else if ((rangeBefore - range) > Landmarks::MIN_DIFFERENCE ||
               (rangeAfter - range) > Landmarks::MIN_DIFFERENCE)
        tempLandmarks[i] = this->getLandmark(cloud.points[i].x, cloud.points[i].y, agent);
    }
    rangeBefore = range;
    range = rangeAfter;
  }

  // copy landmarks into another vector
  std::vector<Landmarks::Landmark *> foundLandmarks;
  for (unsigned int i = 0; i < tempLandmarks.size(); ++i)
  {
    if (((int)tempLandmarks[i]->id) != -1)
      foundLandmarks.push_back(new Landmarks::Landmark(*tempLandmarks[i]));
    // delete (tempLandmarks[i]);
  }
  return (foundLandmarks);
}

std::vector<Landmarks::Landmark *> Landmarks::removeDouble(std::vector<Landmarks::Landmark *> const &extractedLandmarks, std::vector<Landmarks::Landmark *> &nonAssociatedLandmarks)
{
  int uniquelmrks = 0;
  double leastDistance = 99999;
  std::vector<int> doubleId;
  std::vector<Landmarks::Landmark *> uniqueLandmarks(extractedLandmarks.size(), NULL);

  for (unsigned int i = 0; i < extractedLandmarks.size(); ++i)
  {
    //remove landmarks that didn't get associated
    if (extractedLandmarks[i]->id != -1)
    {
      bool isDouble = false;
      for (size_t c = 0; c < doubleId.size(); ++c)
      {
        if (doubleId[c] == extractedLandmarks[i]->id)
        {
          isDouble = true;
          break;
        }
      }
      if (isDouble == true)
        continue;

      leastDistance = 99999;
      //remove doubles in extractedLandmarks
      //if two observations match same landmark, take closest landmark
      for (size_t j = i; j < extractedLandmarks.size(); ++j)
      {
        if (extractedLandmarks[i]->id == extractedLandmarks[j]->id)
        {
          doubleId.push_back(extractedLandmarks[i]->id);
          double temp = this->distance(*extractedLandmarks[j],
                                       *this->landmarkDB[extractedLandmarks[j]->id]);
          if (temp < leastDistance)
          {
            leastDistance = temp;
            uniqueLandmarks[uniquelmrks] = extractedLandmarks[j];
          }
        }
      }
      if (leastDistance != 99999)
        ++uniquelmrks;
    }
    else
      nonAssociatedLandmarks.push_back(extractedLandmarks[i]);
  }
  // Obliger de faire ça sinon la taille du tableau de sortie n'est pas la bonne
  // resize landmarks into an array of correct dimensions

  uniqueLandmarks.resize(uniquelmrks);
  return (uniqueLandmarks);
}

void Landmarks::alignLandmarkData(std::vector<Landmark *> const &extractedLandmarks,
                                  bool *&matched, int *&id, double *&ranges,
                                  double *&bearings,
                                  std::vector<pcl::PointXYZ> &lmrks,
                                  std::vector<pcl::PointXYZ> &exlmrks)
{
  std::vector<Landmarks::Landmark *> uniqueLandmarks(extractedLandmarks.size(), NULL);

  //Remove doubles form extracted, can't use removeDouble function because it's usiing the getAssociation function
  size_t  uniqueSize = 0;
  unsigned int  leastDistance = 99999; // A big enough distance

  for (size_t i = 0; i < extractedLandmarks.size(); ++i)
  {
    if (extractedLandmarks[i]->id != -1)
    {
      leastDistance = 99999;
      //remove doubles in extractedLandmarks
      //if two observations match same landmark, take closest landmark
      for (size_t j = i; j < extractedLandmarks.size(); ++j)
      {
        if (extractedLandmarks[i]->id == extractedLandmarks[j]->id)
        {
          double temp = this->distance(*extractedLandmarks[j],
                                       *this->landmarkDB[extractedLandmarks[j]->id]);
          if (temp < leastDistance)
          {
            leastDistance = temp;
            uniqueLandmarks[uniqueSize] = extractedLandmarks[j];
          }
        }
      }
      if (leastDistance != 99999)
        ++uniqueSize;
    }
  }
  uniqueLandmarks.resize(uniqueSize);

  matched = new bool[uniqueLandmarks.size()];
  id = new int[uniqueLandmarks.size()];
  ranges = new double[uniqueLandmarks.size()];
  bearings = new double[uniqueLandmarks.size()];
  lmrks = std::vector<pcl::PointXYZ>(uniqueLandmarks.size());
  exlmrks = std::vector<pcl::PointXYZ>(uniqueLandmarks.size());

  for (unsigned int i = 0; i < uniqueLandmarks.size(); ++i)
  {
    matched[i] = true;
    id[i] = uniqueLandmarks[i]->id;
    ranges[i] = uniqueLandmarks[i]->range;
    bearings[i] = uniqueLandmarks[i]->bearing;
    lmrks[i].x = landmarkDB[uniqueLandmarks[i]->id]->pos.x;
    lmrks[i].y = landmarkDB[uniqueLandmarks[i]->id]->pos.y;
    lmrks[i].z = landmarkDB[uniqueLandmarks[i]->id]->pos.z;
    exlmrks[i].x = uniqueLandmarks[i]->pos.x;
    exlmrks[i].y = uniqueLandmarks[i]->pos.y;
    exlmrks[i].z = uniqueLandmarks[i]->pos.z;
  }
}


std::vector<Landmarks::Landmark *> Landmarks::extractLineLandmarks(pcl::PointCloud<pcl::PointXYZRGBA> const &cloud, IAgent const *agent)
{
  // lines found
  std::vector<double> la;
  std::vector<double> lb;
  int totalLines = 0;
  unsigned int numberSample = cloud.points.size();
  int *consensusPoints;

#ifdef DEBUG
  std::vector<Landmarks::Landmark *> tempLandmarks;
#endif

  // linepoints is all the points linked to the lines
  unsigned int totalLinepoints = numberSample - 1;

  // BEGIN RANSAC ALGORITHM
  unsigned int noTrials = 0;
  
  // MINLINEPOINTS : if less than x points left, stop trying to find a consensus (stop algorithm)
  // MAXTRIAL : max times to run algorithm
  while (noTrials < Landmarks::MAXTRIALS && totalLinepoints > Landmarks::MINLINEPOINTS)
  {
    int *rndSelectedPoints = new int[Landmarks::MAXSAMPLE];
    int temp = 0;
    bool newpoint = false;

    int centerPoint = rand() % (totalLinepoints - 1) + Landmarks::MAXSAMPLE;
    rndSelectedPoints[0] = centerPoint;

    //std::cerr << "before search random points" << std::endl;
    // on cherche des points random afin de créer un modèle
    for (unsigned int i = 1; i < Landmarks::MAXSAMPLE; ++i)
    {
      newpoint = false;
      while (!newpoint)
      {
        temp = centerPoint + (rand() % 2 - 1) * rand() % Landmarks::MAXSAMPLE;
        for (unsigned int j = 0; j < i; ++j)
        {
          if (rndSelectedPoints[j] == temp)
            break; //point has already been selected
          if (j >= i - 1)
            newpoint = true; //point has not already been selected
        }
      }
      rndSelectedPoints[i] = temp; // nouveau point trouvé
    }

    double a = 0;
    double b = 0;
    //std::cerr << "before least square line estimate" << std::endl;
    // ax + b => ligne
    // cette fonction modifie les valeurs de 'a' et 'b'
    this->leastSquaresLineEstimate(cloud, agent, rndSelectedPoints, Landmarks::MAXSAMPLE, a, b);
    delete rndSelectedPoints;
    //std::cerr << "after least square line estimate" << std::endl;
    
    //    int *newLinePoints;
    try {
      //– Determine the consensus set S1* of points is P
       consensusPoints = new int[numberSample]; // points closed to the line
    } catch (...) {
      std::cerr << "error consensus points create " << numberSample << std::endl;
      std::vector<Landmarks::Landmark *> ret;
      return (ret);
    }
    unsigned int totalConsensusPoints = 0;
    // try {
    //    newLinePoints = new int[numberSample]; // points far to the line
    // } catch (...) {
    //   std::cerr << "error newline points create " << numberSample << std::endl;
    //   std::vector<Landmarks::Landmark *> ret;
    //   return (ret);
    // }
    unsigned int totalNewLinePoints = 0;
    double x = 0;
    double y = 0;
    double d = 0;
    //std::cerr << "before get consensus points" << std::endl;
    for (unsigned int i = 0; i < totalLinepoints; ++i) // totalLinepoint = numberSample - 1
    {
      // convert ranges and bearing to coordinates
      parametricConvert(agent, cloud.points[i].x, cloud.points[i].y, x, y);
      d = this->distanceToLine(x, y, a, b);
      if (d < Landmarks::RANSAC_TOLERANCE)
      {
        // points prets de la ligne
        consensusPoints[totalConsensusPoints] = i;
        ++totalConsensusPoints;
      }
      else
      {
        // points loin de la ligne
        //newLinePoints[totalNewLinePoints] = i;
        ++totalNewLinePoints;
      }
    }
    //std::cerr << "before get line from consensus points" << std::endl;
    if (totalConsensusPoints > Landmarks::RANSAC_CONSENSUS)
    {
      // cette fonction modifie les valeurs de 'a' et 'b'
      this->leastSquaresLineEstimate(cloud, agent, consensusPoints, totalConsensusPoints, a, b);
      totalLinepoints = totalNewLinePoints;

#ifdef DEBUG
      //for now add points associated to line as landmarks to see results
      for (unsigned int i = 0; i < totalConsensusPoints; ++i)
      {
        //Remove points that have now been associated to this line
        tempLandmarks[consensusPoints[i]] = this->getLandmark(cloud.points[consensusPoints[i]].x, cloud.points[consensusPoints[i]].y, agent);
      }
#endif
      delete consensusPoints;
      //delete newLinePoints;
      
      // ajout des lignes trouvées
      la.push_back(a);
      lb.push_back(b);
      ++totalLines;
      noTrials = 0;
    }
    else
      ++noTrials;
  }
  // END OF RANSAC ALGORITHM

  // pour debug, ajouter origin comme un landmark
#ifdef DEBUG
  tempLandmarks.push_back(this->getOrigin());
#endif

  std::vector<Landmarks::Landmark *> foundLandmarks(totalLines);
  try {
    for (int i = 0; i < totalLines; ++i)
      {
	foundLandmarks[i] = this->getLineLandmark(la[i], lb[i], agent);
      }
  } catch (...) {
    std::cerr << "Error when adding landmark in found landmark" << std::endl;
  }
  return foundLandmarks;
}

void Landmarks::removeBadLandmarks(pcl::PointCloud<pcl::PointXYZRGBA> const &cloud, IAgent const *agent)
{
  double maxrange = 0;
  double rangeBefore = this->distance(cloud.points[0].x, cloud.points[0].y, agent->getPos().x, agent->getPos().y);
  double range = this->distance(cloud.points[1].x, cloud.points[1].y, agent->getPos().x, agent->getPos().y);

  for (unsigned int i = 1; i < cloud.points.size() - 1; ++i)
  {
    double rangeAfter = this->distance(cloud.points[i + 1].x, cloud.points[i + 1].y, agent->getPos().x, agent->getPos().y);
    /// @todo: check real camera max
    // we get the camera data with max range
    if (rangeBefore < agent->cameraProblem
        && rangeAfter < agent->cameraProblem
        && range > maxrange)
      maxrange = range;
    rangeBefore = range;
    range = rangeAfter;
  }

  double *Xbounds = new double[4];
  double *Ybounds = new double[4];

  //get bounds of rectangular box to remove bad landmarks from
  Xbounds[0] = cos((this->degreePerScan * Landmarks::CONVERSION) + (agent->getYaw() * Landmarks::CONVERSION)) * maxrange + agent->getPos().x;
  Ybounds[0] = sin((this->degreePerScan * Landmarks::CONVERSION) + (agent->getYaw() * Landmarks::CONVERSION)) * maxrange + agent->getPos().y;
  Xbounds[1] = Xbounds[0] + cos((180 * this->degreePerScan * Landmarks::CONVERSION) + (agent->getYaw() * Landmarks::CONVERSION)) * maxrange;
  Ybounds[1] = Ybounds[0] + sin((180 * this->degreePerScan * Landmarks::CONVERSION) + (agent->getYaw() * Landmarks::CONVERSION)) * maxrange;
  Xbounds[2] = cos((359 * this->degreePerScan * Landmarks::CONVERSION) + (agent->getYaw() * Landmarks::CONVERSION)) * maxrange + agent->getPos().x;
  Ybounds[2] = sin((359 * this->degreePerScan * Landmarks::CONVERSION) + (agent->getYaw() * Landmarks::CONVERSION)) * maxrange + agent->getPos().y;
  Xbounds[3] = Xbounds[2] + cos((180 * this->degreePerScan * Landmarks::CONVERSION) + (agent->getYaw() * Landmarks::CONVERSION)) * maxrange;
  Ybounds[3] = Ybounds[2] + sin((180 * this->degreePerScan * Landmarks::CONVERSION) + (agent->getYaw() * Landmarks::CONVERSION)) * maxrange;

  //now check DB for landmarks that are within this box
  //decrease life of all landmarks in box. If the life reaches zero, remove landmark

  double pntx;
  double pnty;

  for (int k = 0; k < DBSize; ++k)
  {
    pntx = this->landmarkDB[k]->pos.x;
    pnty = this->landmarkDB[k]->pos.y;
    int i = 0;
    int j = 0;
    bool inRectangle = (agent->getPos().x < 0 || agent->getPos().y < 0 ? false : true);
    for (i = 0; i < 4; ++i)
    {
      if ((((Ybounds[i] <= pnty) && (pnty < Ybounds[j])) || ((Ybounds[j] <= pnty) && (pnty < Ybounds[i]))) &&
          (pntx < (Xbounds[j] - Xbounds[i]) * (pnty - Ybounds[i]) / (Ybounds[j] - Ybounds[i]) + Xbounds[i]))
      {
        if (inRectangle == false)
          inRectangle = true;
      }
      j = i++;
    }
    if (inRectangle)
    {
      //in rectangle so decrease life and maybe remove
      if ((--(this->landmarkDB[k]->life)) <= 0)
      {
        //@TODO: Change id of all landmarks to be continued: id is DBSize so we'll have more than one id
        delete (this->landmarkDB[k]);
        this->landmarkDB.erase(this->landmarkDB.begin() + k);
        --DBSize;
      }
    }
  }
}

std::vector<Landmarks::Landmark *> Landmarks::updateAndAddLineLandmarks(std::vector<Landmarks::Landmark *> extractedLandmarks)
{
  std::vector<Landmarks::Landmark *> res(extractedLandmarks.size());
  for (unsigned int i = 0; i < extractedLandmarks.size(); ++i)
    res[i] = this->updateLandmark(extractedLandmarks[i]);
  return (res);
}

std::vector<Landmarks::Landmark *> Landmarks::updateAndAddLandmarkUsingEKFResults(bool matched[],
    unsigned int numberMatched, int id[], std::vector<pcl::PointXYZ> const &pos, IAgent const *agent)
{
  std::vector<Landmarks::Landmark *> res(numberMatched);
  for (unsigned int i = 0; i < numberMatched; ++i)
    res[i] = this->updateLandmark(matched[i], id[i], pos[i].x, pos[i].y, agent);
  return (res);
}
