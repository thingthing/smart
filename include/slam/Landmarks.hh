/**
 * @class Landmarks
 *
 * @ingroup SLAM
 *
 * @brief Define and manage Landamrks
 *
 * This class is used to extract landmarks,
 * define landmark structure and all other functions associated
 * with the landmarks
 *
 * @author Nicolas
 *
 * @version 1.0
 *
 * @date 03/05/2015
 *
 */

#ifndef   _LANDMARKS_HH_
# define  _LANDMARKS_HH_

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

   ///Convert from degree to radians
  static const double CONVERSION;
   ///Max number of landmarks
  static const unsigned int MAXLANDMARKS;
  ///If a landmarks is within this distance of another landmarks, its the same landmarks
  static const double MAXERROR;
  ///Number of times a landmark must be observed to be recongnized as a landmark
  static const unsigned int MINOBSERVATIONS;
  /**
  * @brief Use to reset life counter
  * (counter use to determine whether to discard a landmark or not)
  */
  static const unsigned int LIFE;
  ///RANSAC: max times to run algorithm
  static const unsigned int MAXTRIALS;
  ///RANSAC: randomly select x points
  static const unsigned int MAXSAMPLE; //
  /**
  * @brief RANSAC: if less than x points left,
  * don't bother trying to find a consensus (stop algorithm)
  */
  static const unsigned int MINLINEPOINTS;
  ///RANSAC: if point is within x distance of line, its part of the line
  static const double RANSAC_TOLERANCE;
  ///RANSAC: at leat x votes required to determine if its a line
  static const unsigned int RANSAC_CONSENSUS;
  ///Maximum meters range from wich a spike landmark is consider
  static const double MAX_DIFFERENCE; //
  ///Minimum meters range from wich a spike landmark is consider
  static const double MIN_DIFFERENCE;

  class  Landmark
  {
  public:
    ///Landmark position relative to map
    pcl::PointXY pos;
    ///Landmark unique ID
    int id;
    ///A life counter to determine whether to discard a landmark
    int life;
    ///The number of times we have seen the landmark
    int totalTimeObserved;
    ///Last observed range from agent to landmark
    double range;
    ///Last observed bearing from agent to landmark
    double bearing;
    ///Position of agent when landmark was spotted
    pcl::PointXYZ robotPos;

    /// RANSAC : Store equation of a line to be reused, first point of line
    double a;
    /// RANSAC : Store equation of a line to be reused, second point of line
    double b;
    /// Distance from robot position to the wall we are using as a landmark (to calculate error)
    double rangeError;
    /// Bearing from robot position to the wall we are using as a landmark (to calculate error)
    double bearingError;

  public:
    Landmark();
    ~Landmark();
  };


public:
  ~Landmarks();
  Landmarks(double degreePerScan = Agent::DEGREESPERSCAN);

  // Getters

  /**
   * @brief Get Slam matrice Id assoicated with landmark id
   *
   * @param id Id of the landmark we want to get the matrice id
   * @return Matrice id for the landmark or -1 if landmark not associated
   */
  int getSLamId(int id) const;
  /**
   * @brief Get Landmarks Database size
   * @return Size of landmarkDB
   */
  unsigned int getDBSize() const;
  /**
   * @brief Get landmarks database
   * @return Landmarks database
   */
  std::vector<Landmark *> getLandmarkDB() const;

  // Setters

  /**
   * @brief Associate matrice id with landmark id
   *
   * @param landmarkId Id of the landmark to be associated
   * @param slamId Id of the landmark in the SLAM matrices
   */
  void addSlamId(int landmarkId, int slamId);
  /**
   * @brief Add landmark to database
   * @details Add a copy of lm to the database with default life and timeObserved
   *
   * @param lm Landmark to be copied and add to the database
   * @return Id of the new landmark in the database or -1 if an error occured
   */
  int addToDB(const Landmark &lm);

  //Remove

  /**
   * @brief Remove bad landmarks from db
   * @details Check if landmark should be seen in capture data.
   * If it should be seen, decrease life one time
   * (life is reset to LIFE whenever landmark is seen again)
   * If life is 0, it means that the landmark should have been seen at least LIFE
   * time but wasn't, so it's not a good landmark, so we delete it.
   *
   * @param cloud Capture data from agent
   * @param agent Agent information
   */
  void removeBadLandmarks(pcl::PointCloud<pcl::PointXYZ> const &cloud,
                          Agent const &agent);
  /**
   * @brief Remove duplicated landmarks
   * @details If two landmarks are too close from each other, there are surely
   * the same. We should then remove all of them but one
   *
   * @param extractedLandmarks Landmarks extracted from capture data this cycle
   * @param[out] nonAssociatedLandmarks Landmarks that weren't found in db
   * @return Vector of unique landmarks
   */
  std::vector<Landmark *> removeDouble(std::vector<Landmark *> const &extractedLandmarks,
                                       std::vector<Landmark *> &nonAssociatedLandmarks);

  //Update

  /**
   * @brief Update landmarks with extracted data and add them to database if
   * their are new
   *
   * @param extractedLandmarks New landmarks data
   * @return Extracted landmarks with updated data (LIFE, id and time observed)
   */
  std::vector<Landmark *> updateAndAddLineLandmarks(std::vector<Landmark *> extractedLandmarks);
  /**
   * @brief Update landmarks using EKF results
   *
   * @param matched Array of boolean: true if landmarks is in db, false otherwise
   * @param numberMatched Number of extracted landmark that are already in db
   * @param id Id of the landmarks (-1 if not in db)
   * @param pos Position of the landmarks
   * @param agent Agent information
   *
   * @return Extracted landmarks with updated data (LIFE, id and time observed)
   */
  std::vector<Landmark *> updateAndAddLandmarkUsingEKFResults(bool matched[],
      unsigned int numberMatched,
      int id[],
      std::vector<pcl::PointXYZ> const &pos,
      Agent const &agent);
  /**
   * @brief Update landmark information in db or add landmark to db
   *
   * @param lm Landmarks with information updated
   * @return New id of landmark in db
   */
  int updateLineLandmark(Landmark &lm);


  //Extract

  /**
   * @brief Extract landmarks from capture data
   * @details Use RANSAC algortihm to find line patterns in capture data
   * and interpret them as landmarks
   *
   * @param cloud Capture data this cycle
   * @param agent Agent information
   *
   * @return Extracted landmarks
   */
  std::vector<Landmark *> extractLineLandmarks(pcl::PointCloud<pcl::PointXYZ> const &cloud,
      Agent const &agent);


  //Other

  /**
   * @brief Set the data for updateAndAddLandmarkUsingEKFResults from extracted
   * landmarks
   *
   * @param exlmrks Extracted landmarks this cycle
   * @param[out] matched Array of boolean (true if landmark is in db)
   * @param[out] id Ids of landmark (-1 if landmark not in db)
   * @param[out] ranges Ranges from agent to landmarks
   * @param[out] bearings Bearings from agent ot landmarks
   * @param[out] lmrks Position of landmarks as saved in database
   * @param[out] exlmrks Position of landmarks as observed this cycle
   */
  void alignLandmarkData(std::vector<Landmark *> const &extractedLandmarks,
                         bool *&matched,
                         int *&id, double *&ranges, double *&bearings,
                         std::vector<pcl::PointXY> &lmrks,
                         std::vector<pcl::PointXY> &exlmrks);
  /**
   * @brief Get associated landmark in database
   * @details Search for a landmark close enough in database to be the same
   * If such a landmark is found, we update its information and increment its
   * totalTimeObserved.
   * 
   * @param lm Landmark we want to associate in database
   * @return New landmark id or -1 if no association found
   */
  int getAssociation(Landmark &lm);
  /**
   * @brief Get closest associated valid landmark in database
   * @details A valid landmark is a landmark that has been observed at least
   * MINOBSERVATION time. This function set the id and totaltimeobserved of the
   * closest valid landmark from lm in database
   * 
   * @param lm Landmark we want to associate
   * @param[out] id Id of associated landmark in database (or -1 if not found)
   * @param[out] totalTimeObserved Total time the associated landmark in database
   * has been observed
   */
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
