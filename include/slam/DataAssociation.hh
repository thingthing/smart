/**
 * @class DataAssociation
 *
 * @ingroup SLAM
 *
 * @brief Associate new Landmarks with Landmarks in DB
 *
 * @author Nicolas
 *
 * @version 1.0
 *
 * @date 03/05/2015
 *
 */

#ifndef   _DATA_ASSOCIATION_HH_
# define  _DATA_ASSOCIATION_HH_

#include "Landmarks.hh"
#include "IAgent.hh"

class DataAssociation
{
public:
  DataAssociation();
  DataAssociation(Landmarks *landmarkDb);
  ~DataAssociation();

  /**
  * @brief Get landmarks from mapping
  * @details Extract landmarks from mapping then find wich of them are new and
  * which are already in the database.
  * It will be called after each data gathering.
  *
  * @param cloud Result of current mapping
  * @param agent Agent wich is linked to the current mapping
  * @param[out] resultLandmarks New landmark extracted from current mapping
  * @param[out] reobservedLandmarks Reobserved landmark extracted from current mapping
  */
  void validationGate(ICapture::DATA &data, IAgent const *agent, std::vector<Landmarks::Landmark *> &resultLandmarks, std::vector<Landmarks::Landmark *> &reobservedLandmarks);
  /**
   * @brief Set id and timeObserved for landamark base on closer landmark in db
   * 
   * @param toAssociate New Landmark we want to associate with db
   * @return Boolean true if an association is found, false otherwise
   */
  bool  associateLandmarks(Landmarks::Landmark *toAssociate) const;

  /**
  * @brief Get landmark database
  * @details Get the landmark database associated with this SLAM algorithm
  * @return Landmark database
  */
  Landmarks *getLandmarkDb() const;

private:
#ifdef UNITTEST
public: // ONLY FOR UNIT TESTS
#endif
  Landmarks *_landmarkDb;

};

#endif    /* !_DATA_ASSOCIATION_HH_ */
