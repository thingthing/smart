#include "igloo/igloo_alt.h"
#include "Landmarks.hh"
#include "test_data.hh"
#include "CustomConstraint.hh"

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

/**
 * Unit test for getLandmarkDb()
 **/
When(getting_LandmarkDB)
{
  
  void	SetUp()
  {
    id1 = lms.addToDB(lm1);
    id2 = lms.addToDB(lm2);
    db = lms.getLandmarkDB();
  }
  
  Then(it_should_hae_the_same_size)
  {
    Assert::That(db.size(), Is().EqualTo(lms.DBSize));
  }

  Then(it_should_have_the_same_elements)
  {
    for(int i = 0; i < lms.DBSize; ++i)
      {
	Assert::That(db[i], Fulfills(IsSameLandmark(lms.landmarkDB[i])));
      }
  }

  Then(it_should_be_a_copy)
  {
    double oldPos[2] = {lms.landmarkDB[id1]->pos[0], lms.landmarkDB[id1]->pos[1]};
    double oldRange = lms.landmarkDB[id1]->range;

    lms.landmarkDB[id1]->pos[0] = oldPos[0] + 1;
    lms.landmarkDB[id1]->range = oldRange + 1;

    Assert::That(db[id1], Is().Not().Fulfilling(IsSameLandmark(lms.landmarkDB[id1])));

    Assert::That(db[id1]->pos[0], Is().EqualTo(oldPos[0]));
    Assert::That(db[id1]->range, Is().EqualTo(oldRange));
  }

  Landmarks lms;
  Landmarks::Landmark lm1;
  Landmarks::Landmark lm2;
  std::vector<Landmarks::Landmark *> db;
  int	id1;
  int	id2;
};

/**
 * Unit test for getAssociation()
 **/
When(getting_association)
{
   
  void	SetUp()
  {
    lm1.pos[0] = 4.2;
    lm1.pos[1] = 2.4;
    id1 = lms.addToDB(lm1);
    id2 = lms.addToDB(lm2);
  }

  When(it_is_a_already_seen_landmark)
  {
    void	SetUp()
    {
      Root().oldTimeObserved =  Root().lms.landmarkDB[Root().id1]->totalTimeObserved;
      idGot = Root().lms.getAssociation(Root().lm1);
    }
    
    Then(it_should_update_good_landmark)
    {
      Assert::That(idGot, Is().EqualTo(Root().id1));
    }
    
    Then(it_should_increase_time_observed)
    {
      Assert::That(Root().lms.landmarkDB[Root().id1]->totalTimeObserved,
		   Is().EqualTo(Root().oldTimeObserved + 1));
    }
    
    Then(it_should_reset_life_counter)
    {
      Assert::That(Root().lms.landmarkDB[Root().id1]->life, Is().EqualTo(LIFE));
    }
    int	idGot;
  };

  When(it_is_a_never_seen_landmark)
  {
    void	SetUp()
    {
      Root().lm2.pos[0] = 10;
      Root().lm2.pos[1] = 8;
      Root().oldTimeObserved = Root().lms.landmarkDB[Root().id2]->totalTimeObserved;
    }
    
    Then(it_should_not_update)
    {
      Assert::That(Root().lms.getAssociation(Root().lm2), Is().EqualTo(-1));
      Assert::That(Root().lms.landmarkDB[Root().id1]->totalTimeObserved,
		   Is().EqualTo(Root().oldTimeObserved));
    }
  };

  Landmarks lms;
  Landmarks::Landmark lm1;
  Landmarks::Landmark lm2;
  int	id1;
  int	id2;
  int	oldTimeObserved;
};

/**
 * Unit test for getClosestAssociation()
 **/
When(getting_closest_association_landmark)
{
  void	SetUp()
  {
    lm1.pos[0] = 24;
    lm1.pos[1] = 42;
    lm2.pos[0] = 10;
    lm2.pos[1] = 8;
    lm3.pos[0] = 12;
    lm3.pos[1] = 8;
    timeObservedResult = 0;
  }

  When(there_are_no_landmark)
  {
    void	SetUp()
    {
      Root().oldTimeObserved = Root().timeObservedResult;
      Root().lms.getClosestAssociation(&(Root().lm1), Root().idResult, Root().timeObservedResult);
    }

    Then(it_should_set_id_to_minus_one)
    {
      Assert::That(Root().idResult, Is().EqualTo(-1));
    }

    Then(it_should_not_change_total_time_observed)
    {
      Assert::That(Root().timeObservedResult, Is().EqualTo(Root().oldTimeObserved));
    }
  };

  Landmarks		lms;
  Landmarks::Landmark	lm1;
  Landmarks::Landmark	lm2;
  Landmarks::Landmark	lm3;
  int		id1;
  int		id2; 
  int		idResult;
  int		timeObservedResult;
  int		oldTimeObserved;
};
