#ifndef		_LANDMARKS_HH_
# define	_LANDMARKS_HH_

#include <vector>
#include <map>
#include <cmath>

static const double CONVERSION = (M_PI / 180.0); // Convert to radians
static const unsigned int MAXLANDMARKS = 3000; // Max number of landmarks
static const double MAXERROR = 0.5; // If a landmarks is within this distance of another landmarks, its the same landmarks
static const unsigned int MINOBSERVATIONS = 15; // Number of times a landmark must be observed to be recongnized as a landmark
static const unsigned int LIFE = 40; // Use to reset life counter (counter use to determine whether to discard a landmark or not)
static const float MAX_RANGE = 1.0;
static const unsigned int MAXTRIALS = 1000; // RANSAC: max times to run algorithm
static const unsigned int MAXSAMPLE = 10; // RANSAC: randomly select x points
static const unsigned int MINLINEPOINTS = 30; // RANSAC: if less than x points left, don't bother trying to find a consensus (stop algorithm)
static const double RANSAC_TOLERANCE = 0.05; // RANSAC: if point is within x distance of line, its part of the line
static const unsigned int RANSAC_CONSENSUS = 30; // RANSAC: at leat x votes required to determine if its a line
static const double DEGREESPERSCAN = 0.5;

class Landmarks
{
public:
  class  Landmark
  {
  public:
    double pos[2]; // landmarks (x, y) position relative to map
    int id; // lanndmarks unique ID
    int life; // a life counter to determine whether to discard a landmarl
    int totalTimeObserved; // the number of times we have seen the landmark
    double range; // last observed range to landmark
    double bearing; // last observed bearing to landmark

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
  Landmarks(double degreePerScan = DEGREESPERSCAN);

  int getSLamId(int id) const;
  int addSlamId(int landmarkId, int slamId);
  int removeBadLandmarks(double cameradata[], double robotPosition[]); // Possibly change array to vector ? Depends of the robot
  int removeBadLandmarks(const std::vector<double> & cameradata, const std::vector<double> & robotPosition); // both to be sure

  std::vector<Landmark *> updateAndAddLineLandmarks(std::vector<Landmark *> extractedLandmarks); // bad return value
  std::vector<Landmark *> updateAndAddLandmarkUsingEKFResults(bool matched[], int id[], double ranges[], double bearings[], double robotPosition[]);
  int updateLineLandmark(Landmark &lm);
  std::vector<Landmark *> extractLineLandmarks(double cameradata[], double robotPosition[]);

  // matched is an array of boolean
  // id is an arary of int
  // id is an arary of int
  // ranges is an array of double
  // bearings is an array of double
  int alignLandmarkData(std::vector<Landmark *> &extractedLandmarks, bool *matched, int *id,
			double *ranges, double *bearings, std::vector<std::pair<double, double> > &lmrks, std::vector<std::pair<double, double> > &exlmrks);
  int addToDB(const Landmark &lm);

  int getDBSize() const;
  std::vector<Landmark *> getLandmarkDB() const;

private:
  Landmark *updateLandmark(bool matched, int id, double distance, double readingNo, double robotPosition[]);
  Landmark *udpdateLandmark(Landmark *lm);

  void leastSquaresLineEstimate(double cameradata[], double robotPosition[], int selectPoints[], int arraySize, double &a, double &b);
  double distanceToLine(double x, double y, double a, double b);
  std::vector<Landmark *> extractSpikeLandmarks(double cameradata[], double robotPosition[]);
  Landmark *getLandmark(double range, int readingNo, double robotPosition[]);
  Landmark *getLineLandmark(double a, double b, double robotPosition[]);
  Landmark *getLine(double a, double b);
  Landmark *getOrigin();
  void getClosestAssociation(Landmark *lm, int &id, int &totalTimeObserved);
  int getAssociation(Landmark &lm);
  std::vector<Landmark *> removeDouble(std::vector<Landmark *> extractedLandmarks);

  double distance(double x1, double y1, double x2, double y2) const;
  double distance(const Landmark &lm1, const Landmark &lm2) const;

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
