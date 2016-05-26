#include <cstdlib>
#include <algorithm>
#include "Landmarks.hh"

const double Landmarks::CONVERSION = (M_PI / 180.0); // Convert to radians
const unsigned int Landmarks::MAXLANDMARKS = 3000; // Max number of landmarks
const double Landmarks::MAXERROR = 0.01; // If a landmarks is within this distance of another landmarks, its the same landmarks
const unsigned int Landmarks::MINOBSERVATIONS = 15; // Number of times a landmark must be observed to be recongnized as a landmark
const unsigned int Landmarks::LIFE = 15; // Use to reset life counter (counter use to determine whether to discard a landmark or not)
const unsigned int Landmarks::MAXTRIALS = 200; // RANSAC: max times to run algorithm
const unsigned int Landmarks::MAXSAMPLE = 250; // RANSAC: randomly select x points
const unsigned int Landmarks::MINLINEPOINTS = 50; // RANSAC: if less than x points left, don't bother trying to find a consensus (stop algorithm)
const double  Landmarks::RANSAC_TOLERANCE = 8.0; // RANSAC: if point is within x distance of line, its part of the line
const unsigned int Landmarks::RANSAC_CONSENSUS = 10; // RANSAC: at leat x votes required to determine if its a line
const double Landmarks::MAX_DIFFERENCE = 0.5; // meter
const double Landmarks::MIN_DIFFERENCE = 0.3; // meter

Landmarks::Detect::Detect()
{
  _detector = cv::xfeatures2d::SURF::create(100, 4, 3, true, false);
  _frame = NULL;
  _lastFrame = NULL;
}

Landmarks::Detect::~Detect()
{
}

std::vector<cv::KeyPoint> Landmarks::Detect::kp_extract(cv::Ptr<cv::xfeatures2d::SURF> det,
  cv::Mat img)
{
  std::vector<cv::KeyPoint> tmp;
  
  det->detect(img, tmp);
  
  return(tmp);
}

cv::Mat Landmarks::Detect::descriptor_compute(cv::Ptr<cv::xfeatures2d::SURF> det,
  cv::Mat img, std::vector<cv::KeyPoint> kp)
{
  cv::Mat tmp;
  
  det->compute(img, kp, tmp);
  
  return(tmp);
}

std::vector<cv::DMatch> Landmarks::Detect::img_match(cv::BFMatcher matcher, cv::Mat last,
  cv::Mat recent)
{
  std::vector<cv::DMatch> tmp, goodMatches;
  
  matcher.match(last, recent, tmp);
  this->getGoodMatches(tmp, last ,recent, goodMatches);
  
  return(goodMatches);
}

double Landmarks::Detect::normofTransform(cv::Mat nmt_rvec, cv::Mat nmt_tvec)
{
  return (std::fabs(std::min(cv::norm(nmt_rvec),
    2 * M_PI - cv::norm(nmt_rvec))) + std::fabs(cv::norm(nmt_tvec)));
}

bool Landmarks::Detect::getGoodMatches(std::vector<cv::DMatch>& in_matches, cv::Mat& in_lastKeypoints,
  cv::Mat& in_keypoints, std::vector<cv::DMatch>& in_goodMatches)
{
  double goodMatchMinValue = Landmarks::MAXERROR;
  double goodMatchDistanceTimes = 3;

  //Calcmm_lastkp_lastkpulate closest match
  double minMatchDis = 9999;
  //size_t minMatchIndex = 0;

  if(in_matches.size() == 0)
  {
    std::cerr << "in_matches is empty" << std::endl;
    return false;
  }

  for (size_t i = 0; i < in_matches.size(); ++i)
  {
    if (in_matches[i].distance < minMatchDis)
    {
      minMatchDis = in_matches[i].distance;
      //minMatchIndex = i;
    }
  }

  double maxDistance = std::max(goodMatchDistanceTimes * minMatchDis, goodMatchMinValue);
  for (size_t i=0; i < in_matches.size(); i++)
  {
    if(in_matches[i].distance <= maxDistance)
    {
      in_goodMatches.push_back(in_matches[i]);
    }
  }

  if(in_goodMatches.size() == 0)
  {
    std::cerr << "in_goodMatches is empty" << std::endl;
    return false;
  }

  return true;
}

bool Landmarks::Detect::match_process()
{
  _frame->keyPoints = this->kp_extract(_detector, _frame->rgbMat);
  _frame->descriptions = this->descriptor_compute(_detector, _frame->rgbMat, _frame->keyPoints);
  
  _matchpoint = this->img_match(_matcher, _lastFrame->descriptions, _frame->descriptions);
  
  //surf_show(m_lastFrame->rgbImage, m_frame->rgbImage, m_lastFrame->keyPoints, m_frame->keyPoints, m_matchpoint);
  std::cerr << "MatchPoints size == " << _matchpoint.size() << std::endl;
  if(_matchpoint.size() > Landmarks::RANSAC_CONSENSUS)
  {
    std::cerr << "_lastFrame == " << _lastFrame << " -- frame == " << _frame << std::endl;
    return ransac_detect(_matchpoint, _lastFrame, _frame);
  }
  std::cerr << "Not enough point to do ransac" << std::endl;
  return false;
}

unsigned int    getScale(double nb)
{
  unsigned int  scale = 1;

  while (nb * scale < 1)
    scale *= 10;
  return (scale);
}

bool Landmarks::Detect::ransac_detect(std::vector<cv::DMatch>& in_match, ICapture::DATA* in_frame,
              ICapture::DATA* in_lastFrame)
{
  double norm;
  std::vector<cv::Point2f> pointCloud;
  std::vector<cv::Point3f> pointCloud2;
  std::vector<cv::Point3f> lastPointCloud;

  cv::Point2f tmpPoint2d;
  double pointDepth;
  cv::Point3f tmpLastPoint3d, tmpPoint2d2;

  ushort tmpx, tmpy, tmpDepth;

  double cx = _cameraMatrix.at<double>(0, 2);
  double cy = _cameraMatrix.at<double>(1, 2);
  double fx = _cameraMatrix.at<double>(0, 0);
  double fy = _cameraMatrix.at<double>(1, 1);
  double scale = getScale(in_frame->focal);

  std::cout << "SCALE IS == " << scale << std::endl;
  for (uint16_t i = 0; i < in_match.size(); ++i)
  {
    tmpx = in_frame->keyPoints[in_match[i].queryIdx].pt.x;
    tmpy = in_frame->keyPoints[in_match[i].queryIdx].pt.y;
    tmpDepth = in_frame->depthMat.ptr<ushort>(tmpy)[tmpx];

    pointDepth = double(tmpDepth) / scale;
    tmpPoint2d.x  = tmpx; //= (tmpx - cx) * pointDepth / fx;
    tmpPoint2d.y =  tmpy; //(tmpy - cy) * pointDepth / fy;

    pointCloud.push_back(tmpPoint2d);
  }

  for (uint16_t i = 0; i < in_match.size(); ++i)
  {
    tmpx = in_lastFrame->keyPoints[in_match[i].trainIdx].pt.x;
    tmpy = in_lastFrame->keyPoints[in_match[i].trainIdx].pt.y;
    tmpDepth = in_lastFrame->depthMat.ptr<ushort>(tmpy)[tmpx];

    tmpLastPoint3d.z = double(tmpDepth) / scale;
    tmpLastPoint3d.x = (tmpx - cx) * tmpLastPoint3d.z / fx;
    tmpLastPoint3d.y = (tmpy - cy) * tmpLastPoint3d.z / fy;
    tmpPoint2d2.x = tmpx;
    tmpPoint2d2.y = tmpy;
    tmpPoint2d2.z = tmpLastPoint3d.z;

    pointCloud2.push_back(tmpPoint2d2);
    lastPointCloud.push_back(tmpLastPoint3d);
  }

  //std::cout << "Sovling ransac with pointcloud == " << pointCloud << " -- test pointCloud2 == " << pointCloud2 << " -- _lastPointcloud == " << lastPointCloud << std::endl;
  //Ransacpnp
  solvePnPRansac(lastPointCloud, pointCloud, _cameraMatrix,
    in_frame->distCoefDepth, rvec, tvec, false, Landmarks::MAXTRIALS, Landmarks::RANSAC_TOLERANCE,
    0.99, inliners);

  if (inliners.rows < 5)
  {
    std::cerr << "Not enough point to Match: " << inliners.rows << std::endl;
    return false;
  }

  norm = this->normofTransform(rvec, tvec);

  std::cout << "norm: " << norm << std::endl;
  std::cout << "rvec: " << rvec << std::endl;
  std::cout << "tvec: " << tvec << std::endl;

  // if (norm >= 0.3) //0.3
  // {
  //   std::cerr << "Too Far Away: " << norm  << std::endl;
  //   return false;
  // }

  if(norm >= 5) //5
  {
    std::cerr << "Loop Too Far Away: " << norm << std::endl;
    return false;
  }

  if (norm <= Landmarks::MAXERROR)
  {
    std::cerr << "Too Close: " << norm << std::endl;
    return false;
  }
  return true;
}

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
  //std::vector<Landmarks::Landmark *> landmarkCpy(Landmarks::MAXLANDMARKS);
  this->DBSize = 0;
  this->lastID = 0;
  this->degreePerScan = degrees;
  this->IDtoID = idCpy;
  //this->landmarkDB = landmarkCpy;
}

Landmarks::~Landmarks()
{
  for (std::vector<Landmarks::Landmark *>::iterator it = landmarkDB.begin(); it != landmarkDB.end(); ++it)
  {
    delete (*it);
  }
  landmarkDB.clear();
}

unsigned int Landmarks::getDBSize() const
{
  return (this->DBSize);
}

int Landmarks::getSLamId(int id) const
{
  for (std::vector<std::pair<int, int> >::const_iterator it = IDtoID.begin(); it != IDtoID.end(); ++it)
  {
    if (it->first == id)
      return (it->second);
  }
  return (-1);
}

void Landmarks::addSlamId(int landmarkID, int slamID)
{
  std::pair<int, int> newSlamID;

  newSlamID.first = landmarkID;
  newSlamID.second = slamID;
  this->IDtoID.push_back(newSlamID);
}

///@todo: Should add innovation to getAssociation one day (after matrice implementation)
int Landmarks::getAssociation(Landmark &lm)
{
  Landmarks::Landmark *closestLandmark;
  double temp;
  double leastDistance = 99999; // 100k meter, big distance
 
  closestLandmark = getLandmark(lm.id);
  if (closestLandmark)
    leastDistance = this->distance(lm, *closestLandmark);
  if (lm.id == -1 || closestLandmark == NULL || leastDistance > Landmarks::MAXERROR) {
    for (std::vector<Landmarks::Landmark *>::iterator it = landmarkDB.begin(); it != landmarkDB.end(); ++it)
    {
      temp = this->distance(lm, (**it));
      if (temp < leastDistance && (*it)->id != -1)
      {
        leastDistance = temp;
        closestLandmark = (*it);
      }
    }
  }

  if (closestLandmark != NULL && leastDistance < Landmarks::MAXERROR)
  {
     closestLandmark->life = Landmarks::LIFE;
      ++closestLandmark->totalTimeObserved;
      closestLandmark->bearing = lm.bearing;
      closestLandmark->pos.x = lm.pos.x;
      closestLandmark->pos.y = lm.pos.y;
      closestLandmark->range = lm.range;
      closestLandmark->robotPos = lm.robotPos;
      lm.id = closestLandmark->id;
      return (closestLandmark->id);
  }
  return (-1);
}

double Landmarks::distance(double x1, double y1, double x2, double y2) const
{
  return (sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2)));
}

double Landmarks::distance(const Landmark &lm1, const Landmark &lm2) const
{
  return (sqrt(pow(lm1.pos.x - lm2.pos.x, 2) + pow(lm1.pos.y - lm2.pos.y, 2) + pow(lm1.pos.z - lm2.pos.z, 2)));
}

int Landmarks::addToDB(const Landmark &lm)
{
  Landmarks::Landmark *new_elem;

  //To limit DBSIZE, not usefull for now
  //if (static_cast<unsigned int>(DBSize + 1) < this->landmarkDB.size())
  //{
    new_elem = new Landmarks::Landmark();

    new_elem->pos.x = lm.pos.x;
    new_elem->pos.y = lm.pos.y;
    new_elem->pos.z = lm.pos.z;
    new_elem->life = Landmarks::LIFE;
    new_elem->id = lastID;
    new_elem->totalTimeObserved = 1;
    new_elem->bearing = lm.bearing;
    new_elem->range = lm.range;
    new_elem->robotPos = lm.robotPos;
    new_elem->a = lm.a;
    new_elem->b = lm.b;

    landmarkDB.push_back(new_elem);
    ++DBSize;
    ++lastID;
    return (lastID - 1);
  //}
  //return (-1);
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

  for (std::vector<Landmarks::Landmark *>::const_iterator it = landmarkDB.begin(); it != landmarkDB.end(); ++it)
    res.push_back(new Landmarks::Landmark(*(*it)));
  return (res);
}

void Landmarks::getClosestAssociation(Landmark *lm, int &id, int &totalTimeObserved)
{
  Landmarks::Landmark *closestLandmark = NULL;
  double temp;
  double leastDistance = 99999; // 100k meter, big distance

  for (std::vector<Landmarks::Landmark *>::iterator it = landmarkDB.begin(); it != landmarkDB.end(); ++it)
  {
    if (static_cast<unsigned int>((*it)->totalTimeObserved) > Landmarks::MINOBSERVATIONS)
    {
      temp = this->distance(*lm, *(*it));
      if (temp < leastDistance)
      {
        leastDistance = temp;
        closestLandmark = (*it);
      }
    }
  }
  if (closestLandmark == NULL)
    id = -1;
  else
  {
    id = closestLandmark->id;
    totalTimeObserved = closestLandmark->totalTimeObserved;
  }
}

// Parametric convert equation:
// x = (x_view * cos(bearing) - y_view * sin(bearing)) + agentx
// y = (x_view * sin(bearing) + y_view * cos(bearing)) + agenty
static void parametricConvert(IAgent const *agent, double x_view, double y_view, double &x, double &y, double z_view, double &z)
{
  x = (cos(agent->getYaw()) * x_view - sin(agent->getYaw()) * y_view) + agent->getPos().x;
  y = (sin(agent->getYaw()) * x_view + cos(agent->getYaw()) * y_view) + agent->getPos().y;
  z = x * sin(agent->getPitch()) + z_view * cos(agent->getPitch());
}

double Landmarks::calculateBearing(double x, double y, IAgent const *agent) const
{
  return (atan((y - agent->getPos().y) / (x - agent->getPos().x)) - agent->getYaw());
}

void Landmarks::leastSquaresLineEstimate(pcl::PointCloud<pcl::PointXYZRGBA> const &cloud, IAgent const *agent, std::vector<int> &selectPoints, int arraySize, double &a, double &b, double &c)
{
  double y; //y coordinate
  double x; //x coordinate
  double z;
  double sumY = 0; //sum of y coordinates
  double sumYY = 0; //sum of y^2 for each coordinate
  double sumX = 0; //sum of x coordinates
  double sumXX = 0; //sum of x^2 for each coordinate
  double sumYX = 0; //sum of y*x for each point
  double sumZ = 0;
  double sumZZ = 0;

  for (int i = 0; i < arraySize; ++i)
  {
    //convert ranges and bearing to coordinates
    parametricConvert(agent, cloud.points[selectPoints[i]].x, cloud.points[selectPoints[i]].y, x, y, cloud.points[selectPoints[i]].z, z);
    sumY += y;
    sumYY += pow(y, 2);
    sumX += x;
    sumXX += pow(x, 2);
    sumYX += y * x;
    sumZ += z;
    sumZZ += pow(z, 2);
  }
  b = (sumY * sumXX - sumX * sumYX) / (arraySize * sumXX - pow(sumX, 2));
  a = (arraySize * sumYX - sumX * sumY) / (arraySize * sumXX - pow(sumX, 2));
  ///@todo: trully calculate z
  c = sumZ / arraySize;
}

Landmarks::Landmark *Landmarks::getLandmark(unsigned int landmark_id) const {
  for (std::vector<Landmarks::Landmark *>::const_iterator it = landmarkDB.begin(); it != landmarkDB.end(); ++it) {
    if (static_cast<unsigned int>((*it)->id) == landmark_id)
      return (*it);
  }
  return (NULL);
}
/**
 * @todo Do not use range only, use point (maybe)
 */
Landmarks::Landmark *Landmarks::getLandmark(double x_view, double y_view, double z_view, IAgent const *agent)
{
  Landmarks::Landmark *lm = new Landmarks::Landmark();
  int id = -1;
  int totalTimeObserved = 0;
  double x;
  double y;
  double z;

  parametricConvert(agent, x_view, y_view, x, y, z_view, z);
  lm->pos.x = x;
  lm->pos.y = y;
  lm->pos.z = z;
  lm->range = this->distance(x, y, agent->getPos().x, agent->getPos().y);
  lm->bearing = this->calculateBearing(x, y, agent);
  lm->robotPos = agent->getPos();
  ///@todo: Maybe throw and exception if no landmarks can be associated
  this->getClosestAssociation(lm, id, totalTimeObserved);
  lm->id = id;
  lm->totalTimeObserved = totalTimeObserved;
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
Landmarks::Landmark *Landmarks::updateLandmark(bool matched, int id, double x_view, double y_view, double z_view, IAgent const *agent)
{
  Landmarks::Landmark *lm = this->getLandmark(id);;
  double x;
  double y;
  double z;

  if (matched && lm != NULL)
  {
    // it exists in the DB
    ++lm->totalTimeObserved;
  }
  else
  {
    // doesn't exist in the DB/fail to matched, so that, we've to add this sample
    lm = new Landmarks::Landmark();
    parametricConvert(agent, x_view, y_view, x, y, z_view, z);
    lm->pos.x = x;
    lm->pos.y = y;
    lm->pos.z = z;
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

Landmarks::Landmark *Landmarks::getLineLandmark(double a, double b, double c, IAgent const *agent)
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
  int id = -1;
  int totalTimesObserved = 0;

  //convert landmark to map coordinate
  lm->pos.x = x;
  lm->pos.y = y;
  lm->pos.z = c;
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
        tempLandmarks[i] = this->getLandmark(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z, agent);
      else if ((rangeBefore - range) > Landmarks::MIN_DIFFERENCE ||
               (rangeAfter - range) > Landmarks::MIN_DIFFERENCE)
        tempLandmarks[i] = this->getLandmark(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z, agent);
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

  for (std::vector<Landmarks::Landmark *>::const_iterator it = extractedLandmarks.begin(); it != extractedLandmarks.end(); ++it)
  {
    //remove landmarks that didn't get associated
    if ((*it)->id != -1)
    {

      bool isDouble = false;
      for (size_t c = 0; c < doubleId.size(); ++c)
      {
        if (doubleId[c] == (*it)->id)
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
      for (std::vector<Landmarks::Landmark *>::const_iterator jt = it; jt != extractedLandmarks.end(); ++jt)
      {
        if ((*it)->id == (*jt)->id)
        {
          doubleId.push_back((*it)->id);
          double temp = this->distance(*(*jt), *this->getLandmark((*jt)->id));
          if (temp < leastDistance)
          {
            leastDistance = temp;
            uniqueLandmarks[uniquelmrks] = (*jt);
          }
        }
      }
      if (leastDistance != 99999)
        ++uniquelmrks;
    }
    else
      nonAssociatedLandmarks.push_back((*it));
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

  for (std::vector<Landmarks::Landmark *>::const_iterator it = extractedLandmarks.begin(); it != extractedLandmarks.end(); ++it)
  {
    if ((*it)->id != -1)
    {
      leastDistance = 99999;
      //remove doubles in extractedLandmarks
      //if two observations match same landmark, take closest landmark
      for (std::vector<Landmarks::Landmark *>::const_iterator jt = it; jt != extractedLandmarks.end(); ++jt)
      {
        if ((*it)->id == (*jt)->id)
        {
          double temp = this->distance(*(*jt), *this->getLandmark((*jt)->id));
          if (temp < leastDistance)
          {
            leastDistance = temp;
            uniqueLandmarks[uniqueSize] = (*jt);
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
  std::vector<double> lc;
  int totalLines = 0;
  unsigned int numberSample = cloud.points.size();
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cpy(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGBA>);
  boost::shared_ptr<std::vector<int> > consensusPoints(new std::vector<int>);
  //int *newLinePoints;

#ifdef DEBUG
  std::vector<Landmarks::Landmark *> tempLandmarks;
#endif

  // linepoints is all the points linked to the lines
  unsigned int totalLinepoints = numberSample - 1;

  // BEGIN RANSAC ALGORITHM
  bool noTrials = false;
  
  //Create copy of cloud
  pcl::copyPointCloud(cloud, *cloud_cpy);
  // std::cerr << "extractLineLandmarks start" << std::endl;
  // MINLINEPOINTS : if less than x points left, stop trying to find a consensus (stop algorithm)
  // MAXTRIAL : max times to run algorithm
  while (noTrials != true && totalLinepoints > Landmarks::MINLINEPOINTS)
  {
    
    pcl::SampleConsensusModelPlane<pcl::PointXYZRGBA>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZRGBA> (cloud_cpy));
    pcl::RandomSampleConsensus<pcl::PointXYZRGBA> ransac (model_p);
    ransac.setDistanceThreshold (Landmarks::RANSAC_TOLERANCE);
    ransac.setMaxIterations(Landmarks::MAXTRIALS);
    ransac.computeModel();
    ransac.getInliers(*consensusPoints);
    


    // int *rndSelectedPoints = new int[Landmarks::MAXSAMPLE];
    // int temp = 0;
    // bool newpoint = false;

    // //Point index between 300 and totalLinepoints - 1 + 300
    // int centerPoint = rand() % (totalLinepoints - 1) + Landmarks::MAXSAMPLE;
    // // std::cerr << "Center point is == " << centerPoint << std::endl;
    // // std::cerr << "totalLinepoints are == " << totalLinepoints << std::endl;
    // rndSelectedPoints[0] = centerPoint;

    // //std::cerr << "before search random points" << std::endl;
    // // on cherche des points random afin de créer un modèle
    // for (unsigned int i = 1; i < Landmarks::MAXSAMPLE; ++i)
    // {
    //   newpoint = false;
    //   while (!newpoint)
    //   {
    //     //Point index between -300 and 300 from center point
    //     temp = centerPoint + (rand() % 2 - 1) * (rand() % Landmarks::MAXSAMPLE);
    //     for (unsigned int j = 0; j < i; ++j)
    //     {
    //       if (rndSelectedPoints[j] == temp)
    //         break; //point has already been selected
    //       if (j >= i - 1)
    //         newpoint = true; //point has not already been selected
    //     }
    //   }
    //   rndSelectedPoints[i] = temp; // nouveau point trouvé
    // }

    double a = 0;
    double b = 0;
    double c = 0;
    // //std::cerr << "before least square line estimate" << std::endl;
    // // ax + b => ligne
    // // cette fonction modifie les valeurs de 'a' et 'b'
    // this->leastSquaresLineEstimate(cloud_cpy, agent, rndSelectedPoints, Landmarks::MAXSAMPLE, a, b);
    // delete rndSelectedPoints;
    // //std::cerr << "after least square line estimate" << std::endl;
    
    // try {
    //   //– Determine the consensus set S1* of points is P
    //    consensusPoints = new int[numberSample]; // points closed to the line
    // } catch (...) {
    //   std::cerr << "error consensus points create " << numberSample << std::endl;
    //   std::vector<Landmarks::Landmark *> ret;
    //   return (ret);
    // }
    // unsigned int totalConsensusPoints = 0;
    // try {
    //   newLinePoints = new int[numberSample]; // points far to the line
    // } catch (...) {
    //   std::cerr << "error newline points create " << numberSample << std::endl;
    //   std::vector<Landmarks::Landmark *> ret;
    //   return (ret);
    // }
    // unsigned int totalNewLinePoints = 0;
    // double x = 0;
    // double y = 0;
    // double d = 0;
    // //std::cerr << "before get consensus points" << std::endl;
    // for (unsigned int i = 0; i < totalLinepoints; ++i) // totalLinepoint = numberSample - 1
    // {
    //   // convert ranges and bearing to coordinates
    //   parametricConvert(agent, cloud_cpy.points[i].x, cloud_cpy.points[i].y, x, y);
    //   d = this->distanceToLine(x, y, a, b);
    //   if (d < Landmarks::RANSAC_TOLERANCE)
    //   {
    //     // points prets de la ligne
    //     consensusPoints[totalConsensusPoints] = i;
    //     ++totalConsensusPoints;
    //   }
    //   else
    //   {
    //     // points loin de la ligne
    //     newLinePoints[totalNewLinePoints] = i;
    //     ++totalNewLinePoints;
    //   }
    // }
    //std::cerr << "before get line from consensus points " << consensusPoints->size() << std::endl;
    if (consensusPoints->size() > Landmarks::RANSAC_CONSENSUS)
    {
      // cette fonction modifie les valeurs de 'a' et 'b'
      this->leastSquaresLineEstimate(*cloud_cpy, agent, *consensusPoints, consensusPoints->size(), a, b, c);
      totalLinepoints = cloud_cpy->size() - consensusPoints->size();
      //std::cerr << "totalLinepoints == " << totalLinepoints << std::endl;
#ifdef DEBUG
      //for now add points associated to line as landmarks to see results
      for (unsigned int i = 0; i < consensusPoints->size(); ++i)
      {
        //Remove points that have now been associated to this line
        tempLandmarks[(*consensusPoints)[i]] = this->getLandmark(cloud_cpy->points[(*consensusPoints)[i]].x, cloud_cpy->points[(*consensusPoints)[i]].y, cloud_cpy->points[(*consensusPoints)[i]].z, agent);
      }
#endif
      //std::cerr << "before remove already use points" << std::endl;
      //Remove already used points from cloud
      pcl::copyPointCloud(*cloud_cpy, *tmp);
      cloud_cpy->clear();
      pcl::ExtractIndices<pcl::PointXYZRGBA> eifilter (true); // Initializing with true will allow us to extract the removed indices
      eifilter.setInputCloud (tmp);
      eifilter.setIndices (consensusPoints);
      // The indices_rem array indexes all points of cloud_in that are not indexed by indices_in
      eifilter.setNegative (true);
      eifilter.filter (*cloud_cpy);
      // for (pcl::PointCloud<pcl::PointXYZRGBA>::iterator it = tmp.begin(); it != tmp.end(); ++it) {
      //   if (std::find(consensusPoints.begin(), consensusPoints.end(), i) != consensusPoints.end()) {
      //     cloud_cpy->push_back(*it);
      //   }
      //   ++i;
      // }
      //std::cerr << "Big size == " << tmp->size() << " -- reduce size == " << cloud_cpy->size() << " -- consensusPoints size == " << consensusPoints->size() << " noTrials == " << noTrials <<std::endl;
      // for (unsigned int i = 0; i < totalLinepoints; ++i) {
      //   cloud_cpy.push_back(tmp.points[newLinePoints[i]]);
      // }
      tmp->clear();
      consensusPoints->clear();
      
      // ajout des lignes trouvées
      la.push_back(a);
      lb.push_back(b);
      lc.push_back(c);
      ++totalLines;
      noTrials = false;
    }
    else
      noTrials = true;
    //std::cerr << "before deleting stuff" << std::endl;

    //delete consensusPoints;
    //delete newLinePoints;
    // std::cerr << "END RANSAC round" << std::endl;

  }
  // END OF RANSAC ALGORITHM
  std::cerr << "END RANSAC with " << noTrials << " -- totalLinepoints == " << totalLinepoints << std::endl;
  // pour debug, ajouter origin comme un landmark
#ifdef DEBUG
  tempLandmarks.push_back(this->getOrigin());
#endif
  std::vector<Landmarks::Landmark *> foundLandmarks(totalLines);
  try {
    for (int i = 0; i < totalLines; ++i)
      {
	foundLandmarks[i] = this->getLineLandmark(la[i], lb[i], lc[i], agent);
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

  for (std::vector<Landmarks::Landmark *>::const_iterator it = this->landmarkDB.begin(); it != this->landmarkDB.end(); ++it)
  {
    pntx = (*it)->pos.x;
    pnty = (*it)->pos.y;
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
      if ((--((*it)->life)) <= 0)
      {
        //@TODO: Change id of all landmarks to be continued: id is DBSize so we'll have more than one id
        delete ((*it));
        this->landmarkDB.erase(it);
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
    res[i] = this->updateLandmark(matched[i], id[i], pos[i].x, pos[i].y, pos[i].z, agent);
  return (res);
}
