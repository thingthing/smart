#ifndef		_LANDMARKS_HH_
# define	_LANDMARKS_HH_

#include <vector>
#include <map>
#include <cmath>
#include <pcl/common/common.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/common/projection_matrix.h>
#include "Agent.hh"

class Landmarks
{
public:
static const double CONVERSION; // Convert to radians
static const unsigned int MAXLANDMARKS; // Max number of landmarks
static const double MAXERROR; // If a landmarks is within this distance of another landmarks, its the same landmarks
static const unsigned int MINOBSERVATIONS; // Number of times a landmark must be observed to be recongnized as a landmark
static const unsigned int LIFE; // Use to reset life counter (counter use to determine whether to discard a landmark or not)
static const unsigned int MAXTRIALS; // RANSAC: max times to run algorithm
static const unsigned int MAXSAMPLE; // RANSAC: randomly select x points
static const unsigned int MINLINEPOINTS; // RANSAC: if less than x points left, don't bother trying to find a consensus (stop algorithm)
static const double RANSAC_TOLERANCE; // RANSAC: if point is within x distance of line, its part of the line
static const unsigned int RANSAC_CONSENSUS; // RANSAC: at leat x votes required to determine if its a line
static const double MAX_DIFFERENCE; // meter
static const double MIN_DIFFERENCE; // meter

  class  Landmark
  {
  public:
    pcl::PointXY pos; // landmarks (x, y) position relative to map
    //double pos[2]; // landmarks (x, y) position relative to map
    int id; // lanndmarks unique ID
    int life; // a life counter to determine whether to discard a landmarl
    int totalTimeObserved; // the number of times we have seen the landmark
    double range; // last observed range to landmark
    double bearing; // last observed bearing to landmark
    pcl::PointXYZ robotPos;

    // RANSAC : Store equation of a line to be reused
    double a;
    double b;
    double rangeError; // distance from robot position to the wall we are using as a landmark (to calculate error)
    double bearingError; // bearing from robot position to the wall we are using as a landmark (to calculate error)

  public:
    Landmark();
    ~Landmark();
  };


public:
  ~Landmarks();
  Landmarks(double degreePerScan = Agent::DEGREESPERSCAN);

  // Getters
  int getSLamId(int id) const;
  int getDBSize() const;
  std::vector<Landmark *> getLandmarkDB() const;

  // Setters
  int addSlamId(int landmarkId, int slamId);
  int addToDB(const Landmark &lm);

  //Remove
  int removeBadLandmarks(pcl::PointCloud<pcl::PointXYZ> const &cloud, Agent const &agent);
  std::vector<Landmark *> removeDouble(std::vector<Landmark *> const &extractedLandmarks, std::vector<Landmark *> &nonAssociatedLandmarks);

  //Update
  std::vector<Landmark *> updateAndAddLineLandmarks(std::vector<Landmark *> extractedLandmarks); // bad return value
  std::vector<Landmark *> updateAndAddLandmarkUsingEKFResults(bool matched[], unsigned int numberMatched, int id[], std::vector<pcl::PointXYZ> const &pos, Agent const &agent);
  int updateLineLandmark(Landmark &lm);

  //Extract
  std::vector<Landmark *> extractLineLandmarks(pcl::PointCloud<pcl::PointXYZ> const &cloud, Agent const &agent);


  //Other
  // matched is an array of boolean
  // id is an arary of int
  // id is an arary of int
  // ranges is an array of double
  // bearings is an array of double
  void alignLandmarkData(std::vector<Landmark *> &extractedLandmarks, bool *&matched, int *&id,
			   double *&ranges, double *&bearings, std::vector<pcl::PointXY> &lmrks, std::vector<pcl::PointXY> &exlmrks);

  int getAssociation(Landmark &lm);
  void getClosestAssociation(Landmark *lm, int &id, int &totalTimeObserved);


private:
#ifdef UNITTEST
public: // ONLY FOR UNIT TESTS
#endif

  //Getters
  Landmark *getLandmark(double x_view, double y_view, Agent const &agent);
  Landmark *getLineLandmark(double a, double b, Agent const &agent);
  Landmark *getLine(double a, double b);
  Landmark *getOrigin();

  //Update
  Landmark *updateLandmark(bool matched, int id, double x_view, double y_view, Agent const &agent);
  Landmark *updateLandmark(Landmark *lm);

  //Extract
  std::vector<Landmark *> extractSpikeLandmarks(pcl::PointCloud<pcl::PointXYZ> const &cloud,
						Agent const &agent);

  //Other
  void leastSquaresLineEstimate(pcl::PointCloud<pcl::PointXYZ> const &cloud, Agent const &agent, int selectPoints[], int arraySize, double &a, double &b);
  double distanceToLine(double x, double y, double a, double b);
  double distance(double x1, double y1, double x2, double y2) const;
  double distance(const Landmark &lm1, const Landmark &lm2) const;
  double calculateBearing(double x, double y, Agent const &agent) const;

private: // PRIVATE OTHER CASES
#ifdef UNITTEST
public: // ONLY FOR UNIT TESTS
#endif
  double degreePerScan;
  std::vector<Landmark *> landmarkDB;
  int DBSize;
  std::vector<std::pair<int, int> > IDtoID;
  int EKFLandmarks;
};

#endif
