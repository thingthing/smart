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

  When(there_is_one_landmark_with_not_enough_observation)
  {
    void	SetUp()
    {
      Root().id1 = Root().lms.addToDB(Root().lm1);
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
  
  When(there_is_one_landmark_with_enough_observation)
  {
    void	SetUp()
    {
      Root().id1 = Root().lms.addToDB(Root().lm1);
      Root().lms.landmarkDB[Root().id1]->totalTimeObserved = MINOBSERVATIONS + 1; 
      Root().lms.getClosestAssociation(&(Root().lm1), Root().idResult, Root().timeObservedResult);
    }

    Then(it_should_set_id_to_landmark_id)
    {
      Assert::That(Root().idResult, Is().EqualTo(Root().id1));
    }

    Then(it_should_change_total_time_observed)
    {
      Assert::That(Root().timeObservedResult, Is().EqualTo(Root().lms.landmarkDB[Root().id1]->totalTimeObserved));
    }
  };

  When(there_is_more_than_one_landmark_without_enough_observation)
  {
    void	SetUp()
    {
      Root().id1 = Root().lms.addToDB(Root().lm1);
      Root().id2 = Root().lms.addToDB(Root().lm2);
      Root().oldTimeObserved = Root().timeObservedResult;
      Root().lms.getClosestAssociation(&(Root().lm2), Root().idResult, Root().timeObservedResult);
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

  When(there_is_more_than_one_landmark_and_one_with_enough_observation)
  {
    void	SetUp()
    {
      Root().id1 = Root().lms.addToDB(Root().lm1);
      Root().id2 = Root().lms.addToDB(Root().lm2);
      Root().lms.landmarkDB[Root().id2]->totalTimeObserved = MINOBSERVATIONS + 1; 
    }
    
    When(trying_with_the_one_that_has_enough_observation)
    {
      void	SetUp()
      {
	Root().lms.getClosestAssociation(&(Root().lm2), Root().idResult, Root().timeObservedResult);
      }

      Then(it_should_set_id_to_good_landmarkId)
      {
	Assert::That(Root().idResult, Is().EqualTo(Root().id2));
      }
      
      Then(it_should_change_total_time_observed)
      {
	Assert::That(Root().timeObservedResult, Is().EqualTo(Root().lms.landmarkDB[Root().id2]->totalTimeObserved));
      }
    };

    When(trying_with_the_one_that_has_not_enough_observation)
    {
      void	SetUp()
      {
	Root().lms.getClosestAssociation(&(Root().lm1), Root().idResult, Root().timeObservedResult);
      }

      Then(it_should_set_id_to_good_landmarkId)
      {
	Assert::That(Root().idResult, Is().EqualTo(Root().id2));
      }
      
      Then(it_should_change_total_time_observed)
      {
	Assert::That(Root().timeObservedResult, Is().EqualTo(Root().lms.landmarkDB[Root().id2]->totalTimeObserved));
      }
    };
  };

  When(there_is_more_than_one_landmark_and_all_with_enough_observation)
  {
    void	SetUp()
    {
      Root().id1 = Root().lms.addToDB(Root().lm1);
      Root().id2 = Root().lms.addToDB(Root().lm2);
      Root().lms.landmarkDB[Root().id2]->totalTimeObserved = MINOBSERVATIONS + 1; 
      Root().lms.landmarkDB[Root().id1]->totalTimeObserved = MINOBSERVATIONS + 1; 
    }

    When(trying_with_one)
    {
      void	SetUp()
      {
	Root().lms.getClosestAssociation(&(Root().lm1), Root().idResult, Root().timeObservedResult);
      }

      Then(it_should_set_id_to_good_landmarkId)
      {
	Assert::That(Root().idResult, Is().EqualTo(Root().id1));
      }
      
      Then(it_should_change_total_time_observed)
      {
	Assert::That(Root().timeObservedResult, Is().EqualTo(Root().lms.landmarkDB[Root().id1]->totalTimeObserved));
      }
    };

    When(trying_with_the_other)
    {
      void	SetUp()
      {
	Root().lms.getClosestAssociation(&(Root().lm2), Root().idResult, Root().timeObservedResult);
      }

      Then(it_should_set_id_to_good_landmarkId)
      {
	Assert::That(Root().idResult, Is().EqualTo(Root().id2));
      }
      
      Then(it_should_change_total_time_observed)
      {
	Assert::That(Root().timeObservedResult, Is().EqualTo(Root().lms.landmarkDB[Root().id2]->totalTimeObserved));
      }
    };

    When(trying_with_one_that_is_not_in_db)
    {
      void	SetUp()
      {
	Root().lms.getClosestAssociation(&(Root().lm3), Root().idResult, Root().timeObservedResult);
      }

      Then(it_should_set_id_to_the_closest_landmarkId)
      {
	Assert::That(Root().idResult, Is().EqualTo(Root().id2));
      }
      
      Then(it_should_change_total_time_observed)
      {
	Assert::That(Root().timeObservedResult, Is().EqualTo(Root().lms.landmarkDB[Root().id2]->totalTimeObserved));
      }
    };
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

/**
 * Unit test for getLandmark()
 * @TODO: add test when there is no good landmark in the database
 **/
When(getting_landmark)
{
  void	SetUp()
  {
    double	robotPosition[3] = {42, 22, 1};

    lm1.pos[0] = 42;
    lm1.pos[1] = 24;
    lm2.pos[0] = 42;
    lm2.pos[1] = 44;
    range = 4.5;
    bearing = 3;
    id1 = lms.addToDB(lm1);
    id2 = lms.addToDB(lm2);
    lms.landmarkDB[id2]->totalTimeObserved = MINOBSERVATIONS + 1; 
    lms.landmarkDB[id1]->totalTimeObserved = MINOBSERVATIONS + 1; 
    lm3 = lms.getLandmark(range, bearing, robotPosition);
  }

  Then(it_should_return_one_of_the_db_landmark)
  {
    Assert::That(lm3->id, Is().EqualTo(id1));
  }

  Then(it_should_set_range)
  {
    Assert::That(lm3->range, Is().EqualTo(range));
  }

  Then(it_should_set_bearing)
  {
    Assert::That(lm3->bearing, Is().EqualTo(bearing));
  }

  Landmarks	lms;
  Landmarks::Landmark	lm1;
  Landmarks::Landmark	lm2;
  Landmarks::Landmark	*lm3;
  int		id1;
  int		id2;
  double	range;
  double	bearing;
};

/**
 * Unit Test for updateLandmark(Landmark *)
 **/
When(updating_landmarks_with_a_landmark)
{
  void		SetUp()
  {
    lm1.pos[0] = 42;
    lm1.pos[1] = 24;
    lm2.pos[0] = 12;
    lm2.pos[1] = 5;

    id1 = lms.addToDB(lm1);
  }
  
  When(landmark_is_not_in_db)
  {
    void	SetUp()
    {
      Root().oldDBSize = Root().lms.DBSize;
      Root().oldId = Root().lm2.id;
      Root().lm3 = Root().lms.updateLandmark(&(Root().lm2));
    }

    Then(it_should_add_the_landmark_to_the_db)
    {
      Assert::That(Root().lms.DBSize, Is().EqualTo(Root().oldDBSize + 1));
    }

    Then(it_should_set_landmark_id_to_the_db_id)
    {
      Assert::That(Root().lm3->id, Is().Not().EqualTo(Root().oldId));
      Assert::That(Root().lm3->id, Is().EqualTo(Root().lms.DBSize - 1));
    }
  };
  
  When(landmark_is_in_db)
  {
    void	SetUp()
    {
      Root().oldDBSize = Root().lms.DBSize;
      Root().oldId = Root().lm1.id;
      Root().lm3 = Root().lms.updateLandmark(&(Root().lm1));
    }

    Then(it_should_not_add_the_landmark_to_the_db)
    {
      Assert::That(Root().lms.DBSize, Is().EqualTo(Root().oldDBSize));
    }

    Then(it_should_set_landmark_id_to_the_db_id)
    {
      Assert::That(Root().lm3->id, Is().Not().EqualTo(Root().oldId));
      Assert::That(Root().lm3->id, Is().EqualTo(Root().id1));
    }
  };

  Landmarks	lms;
  Landmarks::Landmark lm1;
  Landmarks::Landmark lm2;
  Landmarks::Landmark *lm3;
  int		id1;
  int		oldDBSize;
  int		oldId;
};

/**
 * Unit Test for updateLandmark(bool, int, double, double, bearing, robotPos)
 **/
When(updating_landmarks_with_parameters)
{
  void		SetUp()
  {
    robotPosition[0] = 42.0;
    robotPosition[1] = 24.0;
    robotPosition[2] = 1.0;
    distance = 2.5;
    bearing = 0.5;
    oldDBSize = lms.DBSize;
    lm1 = lms.updateLandmark(false, 0, distance, bearing, robotPosition);
    id1 = lm1->id;
  }
 

  When(landmark_is_not_in_db)
  {
    Then(it_should_add_the_landmark_to_the_db)
    {
      Assert::That(Root().lms.DBSize, Is().EqualTo(Root().oldDBSize + 1));
    }

    Then(it_should_return_the_new_landmark)
    {
      Assert::That(Root().lm1->id, Is().Not().EqualTo(-1));
      Assert::That(Root().lm1->bearing, Is().EqualTo(Root().bearing));
      Assert::That(Root().lm1->range, Is().EqualTo(Root().distance));
      Assert::That(Root().lm1->pos[0], Is().Not().EqualTo(0));
      Assert::That(Root().lm1->pos[1], Is().Not().EqualTo(0));
    }
  };
  
  When(landmark_is_in_db)
  {
    void	SetUp()
    {
      Root().oldDBSize = Root().lms.DBSize;
      Root().oldTimeObserved = Root().lms.landmarkDB[Root().id1]->totalTimeObserved;
      Root().lm2 = Root().lms.updateLandmark(true, Root().id1, Root().distance,
					     Root().bearing, Root().robotPosition);
    }

    Then(it_should_not_add_the_landmark_to_the_db)
    {
      Assert::That(Root().lms.DBSize, Is().EqualTo(Root().oldDBSize));
    }

    Then(it_should_return_the_old_landmark)
    {
      Assert::That(Root().lm2->id, Is().EqualTo(Root().id1));
    }

    Then(it_should_add_one_to_timeobserved)
    {
      Assert::That(Root().lm2->totalTimeObserved, Is().EqualTo(Root().oldTimeObserved + 1));
      Assert::That(Root().lms.landmarkDB[Root().id1]->totalTimeObserved, Is().EqualTo(Root().oldTimeObserved + 1));
    }
  };

  Landmarks	lms;
  Landmarks::Landmark *lm1;
  Landmarks::Landmark *lm2;
  double	distance;
  double	bearing;
  double	robotPosition[3];
  int		id1;
  int		oldDBSize;
  int		oldTimeObserved;
};

/**
 * Unit test for updateLineLandmark()
 **/
When(updating_line_landmark)
{
  void	SetUp()
  {
    lm1.pos[0] = 42;
    lm1.pos[1] = 24;
    lm2.pos[0] = 12;
    lm2.pos[1] = 5;

    id1 = lms.addToDB(lm1);
  }

  When(landmark_is_not_in_db)
  {
    void	SetUp()
    {
      Root().oldDBSize = Root().lms.DBSize;
      Root().idResult = Root().lms.updateLineLandmark(Root().lm2);
    }

    Then(it_should_add_landmark_to_db)
    {
      Assert::That(Root().lms.DBSize, Is().EqualTo(Root().oldDBSize + 1));
    }

    Then(it_should_return_new_landmark_id)
    { 
      Assert::That(Root().idResult, Is().Not().EqualTo(-1));
      Assert::That(Root().idResult, Is().EqualTo(Root().lms.DBSize - 1));
    }
  };

  When(landmark_is_in_db)
  {
    void	SetUp()
    {
      Root().lms.addToDB(Root().lm2);
      Root().oldDBSize = Root().lms.DBSize;
      Root().idResult = Root().lms.updateLineLandmark(Root().lm1);
    }

    Then(it_should_not_add_landmark_to_db)
    {
      Assert::That(Root().lms.DBSize, Is().EqualTo(Root().oldDBSize));
    }

    Then(it_should_return_db_landmark_id)
    { 
      Assert::That(Root().idResult, Is().EqualTo(Root().id1));
    }
  };

  Landmarks	lms;
  Landmarks::Landmark lm1;
  Landmarks::Landmark lm2;
  int		id1;
  int		idResult;
  int		oldDBSize;
};

When(getting_landmark_origin)
{
  void	SetUp()
  {
    lm1.pos[0] = 2.5;
    lm1.pos[1] = 3.5;
    lm2.pos[0] = 1.5;
    lm2.pos[1] = 2.5;
    id1 = lms.addToDB(lm1);
    id2 = lms.addToDB(lm2);
    lms.landmarkDB[id1]->totalTimeObserved = MINOBSERVATIONS + 1;
    lms.landmarkDB[id2]->totalTimeObserved = MINOBSERVATIONS + 1;
    lm3 = lms.getOrigin();
  }

  Then(it_should_have_id_of_landmark_closest_from_origin)
  {
    Assert::That(lm3->id, Is().Not().EqualTo(-1));
    Assert::That(lm3->id, Is().EqualTo(id2));
  }

  Then(it_should_have_default_value)
  {
    Assert::That(lm3->pos[0], Is().EqualTo(0));
    Assert::That(lm3->pos[1], Is().EqualTo(0));
    Assert::That(lm3->range, Is().EqualTo(-1));
    Assert::That(lm3->bearing, Is().EqualTo(-1));
  }

  Landmarks	lms;
  Landmarks::Landmark	lm1;
  Landmarks::Landmark	lm2;
  Landmarks::Landmark	*lm3;
  int		id1;
  int		id2;
};

When(getting_landmarks_nearest_to_line)
{
  void	SetUp()
  {
    a = 35.5;
    b = 26.3;
    x = b / ((-1.0 / a) - a);
    y = ((-1.0 / a) * b) / ((-1.0 / a) - a);
    lm1.pos[0] = 42.5;
    lm1.pos[1] = 23.5;
    lm2.pos[0] = x + 0.12;
    lm2.pos[1] = y - 0.02;
    id1 = lms.addToDB(lm1);
    id2 = lms.addToDB(lm2);
    lms.landmarkDB[id1]->totalTimeObserved = MINOBSERVATIONS + 1;
    lms.landmarkDB[id2]->totalTimeObserved = MINOBSERVATIONS + 1;
    lm3 = lms.getLine(a, b);
  }

  Then(it_should_have_id_of_landmark_closest_from_origin)
  {
    Assert::That(lm3->id, Is().Not().EqualTo(-1));
    Assert::That(lm3->id, Is().EqualTo(id2));
  }

  Then(it_should_have_default_value)
  {
    Assert::That(lm3->pos[0], Is().EqualTo(x));
    Assert::That(lm3->pos[1], Is().EqualTo(y));
    Assert::That(lm3->a, Is().EqualTo(a));
    Assert::That(lm3->b, Is().EqualTo(b));
    Assert::That(lm3->range, Is().EqualTo(-1));
    Assert::That(lm3->bearing, Is().EqualTo(-1));
  }

  Landmarks	lms;
  Landmarks::Landmark	lm1;
  Landmarks::Landmark	lm2;
  Landmarks::Landmark	*lm3;
  int		id1;
  int		id2;
  double	a;
  double	b;
  double	x;
  double	y;
};

When(getting_landmarks_nearest_to_line_with_robot_pos)
{
  void	SetUp()
  {
    a = 35.5;
    b = 26.3;
    robotPosition[0] = 5.2;
    robotPosition[1] = 4.2;
    robotPosition[2] = 0.1;

    double ao = -1.0 / a;
    x = b / (ao - a);
    y = (ao * b) / (ao - a);
    range = sqrt(pow(x - robotPosition[0], 2) + pow(y - robotPosition[1], 2));
    bearing = atan((y - robotPosition[1]) / (x - robotPosition[0])) - robotPosition[2];
    
    double bo = robotPosition[1] - ao * robotPosition[0];
    double px = (b - bo) / (ao - a);
    double py = ((ao * (b - bo)) / (ao - a)) + bo;
    
    rangeError = lms.distance(robotPosition[0], robotPosition[1], px, py);
    bearingError = atan((py - robotPosition[1]) / (px - robotPosition[0])) - robotPosition[2];

    lm1.pos[0] = 42.5;
    lm1.pos[1] = 23.5;
    lm2.pos[0] = x + 0.12;
    lm2.pos[1] = y - 0.02;
    id1 = lms.addToDB(lm1);
    id2 = lms.addToDB(lm2);
    lms.landmarkDB[id1]->totalTimeObserved = MINOBSERVATIONS + 1;
    lms.landmarkDB[id2]->totalTimeObserved = MINOBSERVATIONS + 2;
    lm3 = lms.getLineLandmark(a, b, robotPosition);
  }

  Then(it_should_have_id_of_landmark_closest_from_origin)
  {
    Assert::That(lm3->id, Is().Not().EqualTo(-1));
    Assert::That(lm3->id, Is().EqualTo(id2));
  }

  Then(it_should_have_default_value)
  {
    Assert::That(lm3->pos[0], Is().EqualTo(x));
    Assert::That(lm3->pos[1], Is().EqualTo(y));
    Assert::That(lm3->a, Is().EqualTo(a));
    Assert::That(lm3->b, Is().EqualTo(b));
    Assert::That(lm3->range, Is().EqualTo(range));
    Assert::That(lm3->bearing, Is().EqualTo(bearing));
    Assert::That(lm3->rangeError, Is().EqualTo(rangeError));
    Assert::That(lm3->bearingError, Is().EqualTo(bearingError));
  }

  Landmarks	lms;
  Landmarks::Landmark	lm1;
  Landmarks::Landmark	lm2;
  Landmarks::Landmark	*lm3;
  int		id1;
  int		id2;
  double	a;
  double	b;
  double	x;
  double	y;
  double	robotPosition[3];
  double	range;
  double	bearing;
  double	rangeError;
  double	bearingError;
  int		totalTimesObserved;
};
