#include "igloo/igloo_alt.h"
#include "Landmarks.hh"
#include "test_data.hh"

using namespace igloo;

/**
 * Unit test for Landmark constructor
 **/
When(creating_a_landmark)
{
  Then(it_should_have_default_value)
    {
      Assert::That(lm.id, Is().EqualTo(::Landmark_Result::id));
      Assert::That(lm.life, Is().EqualTo(::Landmark_Result::life));
      Assert::That(lm.totalTimeObserved, Is().EqualTo(::Landmark_Result::totalTimesObserved));
      Assert::That(lm.range, Is().EqualTo(::Landmark_Result::range));
      Assert::That(lm.bearing, Is().EqualTo(::Landmark_Result::bearing));
      Assert::That(lm.pos[0], Is().EqualTo(::Landmark_Result::pos[0]));
      Assert::That(lm.pos[1], Is().EqualTo(::Landmark_Result::pos[1]));
    }

  ::Landmarks::Landmark	lm;
};

/**
 * Unit test for default Landmarks constructor
 **/
When(creating_Landmarks)
{
  Then(it_should_have_default_landmarks_value)
  {
    Assert::That(lms.DBSize, Is().EqualTo(::Landmarks_Result::DBSize));
    Assert::That(lms.EKFLandmarks, Is().EqualTo(::Landmarks_Result::EKFLandmarks));
    Assert::That(lms.degreePerScan, Is().EqualTo(::Landmarks_Result::defaultdegreePerScan));
    Assert::That(lms.IDtoID.size(), Is().EqualTo(::Landmarks_Result::sizeIDtoID));
    Assert::That(lms.landmarkDB.size(), Is().EqualTo(::Landmarks_Result::sizelandmarkDB));
  }

  Landmarks lms;
};

/**
 * Unit test for specifi Landmarks constructor : Landmarks(double)
 **/
When(creating_Landmarks_with_a_specific_degree_value_different_from_default)
{
  void SetUp()
  {
    lms = new ::Landmarks(::Landmarks_Result::degreePerScan);
  }

  Then(it_should_not_have_default_degree_value)
  {
    Assert::That(lms->degreePerScan, Is().Not().EqualTo(::Landmarks_Result::defaultdegreePerScan));
  }
  
  Then(it_should_have_this_specific_degree_value)
  {
    Assert::That(lms->degreePerScan, Is().EqualTo(::Landmarks_Result::degreePerScan));
  }

  void	TearDown()
  {
    delete lms;
  }

  Landmarks *lms;
};

/**
 * Unit test for getDbSize() before any change
 */
When(calling_getDBSize_after_construct)
{

  Then(it_should_be_equal_to_DBSize_value)
  {
    Assert::That(lms.getDBSize(), Is().EqualTo(lms.DBSize));
  }
  
  Landmarks lms;
};

/**
 * Unit test for getDbSize() after change
 **/
When(calling_getDBSize_after_DBSize_changed)
{

  void	SetUp()
  {
    lms.DBSize += 1;
  }

  Then(it_should_be_equal_to_DBSize_value)
  {
    Assert::That(lms.getDBSize(), Is().EqualTo(lms.DBSize));
  }
  
  Landmarks lms;
};

/**
 * Unit test for addSlamId()
 **/
When(adding_one_Slam_Id)
{
  
  void	SetUp()
  {
    previousLandmarksNumber = lms.EKFLandmarks;
    lms.addSlamId(::Landmarks_Result::goodSlamId.first, ::Landmarks_Result::goodSlamId.second);
  }
  
  Then(it_should_increase_EKFLandmarks_by_one)
  {
    Assert::That(lms.EKFLandmarks, Is().EqualTo(previousLandmarksNumber + 1));
  }

  Then(it_should_be_the_good_id)
  {
    Assert::That(lms.IDtoID[previousLandmarksNumber].first, Is().EqualTo(::Landmarks_Result::goodSlamId.first));
    Assert::That(lms.IDtoID[previousLandmarksNumber].second, Is().EqualTo(::Landmarks_Result::goodSlamId.second));
  }

  Landmarks lms;
  int	previousLandmarksNumber;
};

/**
 * Unit test for getSlamId()
 **/
When(getting_Slam_Id)
{
  
  void	SetUp()
  {
    lms.addSlamId(::Landmarks_Result::wrongSlamId.first, ::Landmarks_Result::wrongSlamId.second);
    lms.addSlamId(::Landmarks_Result::goodSlamId.first, ::Landmarks_Result::goodSlamId.second);
  }
  
  Then(it_should_not_return_the_bad_id)
  {
    Assert::That(lms.getSLamId(::Landmarks_Result::goodSlamId.first),
		 Is().Not().EqualTo(::Landmarks_Result::wrongSlamId.second));
  }

  Then(it_should_return_the_good_id)
  {
    Assert::That(lms.getSLamId(::Landmarks_Result::goodSlamId.first),
		 Is().EqualTo(::Landmarks_Result::goodSlamId.second));
  }

  Landmarks lms;
};

/**
 * Unit test for addToDB()
 **/
When(adding_landmark_to_db)
{
  
  void	SetUp()
  {
    lm.pos[0] = ::Landmark_Result::pos[0] + 1;
    lm.pos[1] = ::Landmark_Result::pos[1] + 1;
    lm.life = LIFE - 1;
    lm.bearing = ::Landmark_Result::bearing + 1;
    lm.range = ::Landmark_Result::range + 1;
    lm.a = 3.12;
    lm.b = 4.52;

    previousDBSize = lms.DBSize;
    idLandmark = lms.addToDB(lm);
  }
  
  Then(DBSize_should_be_increased_by_one)
  {
    Assert::That(lms.DBSize, Is().EqualTo(previousDBSize + 1));
  }

  Then(it_have_the_good_landmark)
  {
    Assert::That(lms.landmarkDB[idLandmark]->pos[0], Is().EqualTo(lm.pos[0]));
    Assert::That(lms.landmarkDB[idLandmark]->pos[1], Is().EqualTo(lm.pos[1]));
    Assert::That(lms.landmarkDB[idLandmark]->bearing, Is().EqualTo(lm.bearing));
    Assert::That(lms.landmarkDB[idLandmark]->range, Is().EqualTo(lm.range));
    Assert::That(lms.landmarkDB[idLandmark]->a, Is().EqualTo(lm.a));
    Assert::That(lms.landmarkDB[idLandmark]->b, Is().EqualTo(lm.b));
    Assert::That(lms.landmarkDB[idLandmark]->life, Is().EqualTo(LIFE));
    Assert::That(lms.landmarkDB[idLandmark]->id, Is().EqualTo(idLandmark));
    Assert::That(lms.landmarkDB[idLandmark]->totalTimeObserved, Is().EqualTo(1));
  }

  Landmarks::Landmark	lm;
  Landmarks		lms;
  int			previousDBSize;
  int			idLandmark;
};

