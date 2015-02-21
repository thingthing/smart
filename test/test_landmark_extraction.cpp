#include <cstdlib> //Rand
#include "igloo/igloo_alt.h"
#include "Landmarks.hh"
#include "CustomConstraint.hh"
#include "test_slam_common.hh"

using namespace igloo;

/**
 * Unit test for Landmark constructor
 **/
When(creating_a_landmark)
{
  Then(it_should_have_default_value)
    {
      AssertThatDetail(lm.id, Is().EqualTo(::Landmark_Result::id));
      AssertThatDetail(lm.life, Is().EqualTo(::Landmark_Result::life));
      AssertThatDetail(lm.totalTimeObserved, Is().EqualTo(::Landmark_Result::totalTimesObserved));
      AssertThatDetail(lm.range, Is().EqualTo(::Landmark_Result::range));
      AssertThatDetail(lm.bearing, Is().EqualTo(::Landmark_Result::bearing));
      AssertThatDetail(lm.pos.x, Is().EqualTo(::Landmark_Result::pos[0]));
      AssertThatDetail(lm.pos.y, Is().EqualTo(::Landmark_Result::pos[1]));
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
    AssertThatDetail(lms.DBSize, Is().EqualTo(::Landmarks_Result::DBSize));
    AssertThatDetail(lms.EKFLandmarks, Is().EqualTo(::Landmarks_Result::EKFLandmarks));
    AssertThatDetail(lms.degreePerScan, Is().EqualTo(::Landmarks_Result::defaultdegreePerScan));
    AssertThatDetail(lms.IDtoID.size(), Is().EqualTo(::Landmarks_Result::sizeIDtoID));
    AssertThatDetail(lms.landmarkDB.size(), Is().EqualTo(::Landmarks_Result::sizelandmarkDB));
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
    AssertThatDetail(lms->degreePerScan, Is().Not().EqualTo(::Landmarks_Result::defaultdegreePerScan));
  }

  Then(it_should_have_this_specific_degree_value)
  {
    AssertThatDetail(lms->degreePerScan, Is().EqualTo(::Landmarks_Result::degreePerScan));
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
    AssertThatDetail(lms.getDBSize(), Is().EqualTo(lms.DBSize));
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
    AssertThatDetail(lms.getDBSize(), Is().EqualTo(lms.DBSize));
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
    AssertThatDetail(lms.EKFLandmarks, Is().EqualTo(previousLandmarksNumber + 1));
  }

  Then(it_should_be_the_good_id)
  {
    AssertThatDetail(lms.IDtoID[previousLandmarksNumber].first, Is().EqualTo(::Landmarks_Result::goodSlamId.first));
    AssertThatDetail(lms.IDtoID[previousLandmarksNumber].second, Is().EqualTo(::Landmarks_Result::goodSlamId.second));
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
    AssertThatDetail(lms.getSLamId(::Landmarks_Result::goodSlamId.first),
		 Is().Not().EqualTo(::Landmarks_Result::wrongSlamId.second));
  }

  Then(it_should_return_the_good_id)
  {
    AssertThatDetail(lms.getSLamId(::Landmarks_Result::goodSlamId.first),
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
    lm.pos.x = ::Landmark_Result::pos[0] + 1;
    lm.pos.y = ::Landmark_Result::pos[1] + 1;
    lm.life = Landmarks::LIFE - 1;
    lm.bearing = ::Landmark_Result::bearing + 1;
    lm.range = ::Landmark_Result::range + 1;
    lm.a = 3.12;
    lm.b = 4.52;

    previousDBSize = lms.DBSize;
    idLandmark = lms.addToDB(lm);
  }

  Then(DBSize_should_be_increased_by_one)
  {
    AssertThatDetail(lms.DBSize, Is().EqualTo(previousDBSize + 1));
  }

  Then(it_have_the_good_landmark)
  {
    AssertThatDetail(lms.landmarkDB[idLandmark]->pos.x, Is().EqualTo(lm.pos.x));
    AssertThatDetail(lms.landmarkDB[idLandmark]->pos.y, Is().EqualTo(lm.pos.y));
    AssertThatDetail(lms.landmarkDB[idLandmark]->bearing, Is().EqualTo(lm.bearing));
    AssertThatDetail(lms.landmarkDB[idLandmark]->range, Is().EqualTo(lm.range));
    AssertThatDetail(lms.landmarkDB[idLandmark]->a, Is().EqualTo(lm.a));
    AssertThatDetail(lms.landmarkDB[idLandmark]->b, Is().EqualTo(lm.b));
    AssertThatDetail(lms.landmarkDB[idLandmark]->life, Is().EqualTo(Landmarks::LIFE));
    AssertThatDetail(lms.landmarkDB[idLandmark]->id, Is().EqualTo(idLandmark));
    AssertThatDetail(lms.landmarkDB[idLandmark]->totalTimeObserved, Is().EqualTo(1));
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
    AssertThatDetail(db.size(), Is().EqualTo(lms.DBSize));
  }

  Then(it_should_have_the_same_elements)
  {
    for(int i = 0; i < lms.DBSize; ++i)
      {
	AssertThatDetail(db[i], Fulfills(IsSameLandmark(lms.landmarkDB[i])));
      }
  }

  Then(it_should_be_a_copy)
  {
    double oldPos[2] = {lms.landmarkDB[id1]->pos.x, lms.landmarkDB[id1]->pos.y};
    double oldRange = lms.landmarkDB[id1]->range;

    lms.landmarkDB[id1]->pos.x = oldPos[0] + 1;
    lms.landmarkDB[id1]->range = oldRange + 1;

    AssertThatDetail(db[id1], Is().Not().Fulfilling(IsSameLandmark(lms.landmarkDB[id1])));

    AssertThatDetail(db[id1]->pos.x, Is().EqualTo(oldPos[0]));
    AssertThatDetail(db[id1]->range, Is().EqualTo(oldRange));
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
  ScenarioAttribute("hasChild", "true")

  void	SetUp()
  {
    lm1.pos.x = 4.2;
    lm1.pos.y = 2.4;
    id1 = lms.addToDB(lm1);
    id2 = lms.addToDB(lm2);
  }

  When(it_is_a_already_seen_landmark)
  {
    ScenarioAttribute("hasParent", "\t")

    void	SetUp()
    {
      Root().oldTimeObserved =  Root().lms.landmarkDB[Root().id1]->totalTimeObserved;
      idGot = Root().lms.getAssociation(Root().lm1);
    }

    Then(it_should_update_good_landmark)
    {
      AssertThatDetail(idGot, Is().EqualTo(Root().id1));
    }

    Then(it_should_increase_time_observed)
    {
      AssertThatDetail(Root().lms.landmarkDB[Root().id1]->totalTimeObserved,
		   Is().EqualTo(Root().oldTimeObserved + 1));
    }

    Then(it_should_reset_life_counter)
    {
      AssertThatDetail(Root().lms.landmarkDB[Root().id1]->life, Is().EqualTo(Landmarks::LIFE));
    }
    int	idGot;
  };

  When(it_is_a_never_seen_landmark)
  {
    ScenarioAttribute("hasParent", "\t")

    void	SetUp()
    {
      Root().lm2.pos.x = 10;
      Root().lm2.pos.y = 8;
      Root().oldTimeObserved = Root().lms.landmarkDB[Root().id2]->totalTimeObserved;
    }

    Then(it_should_not_update)
    {
      AssertThatDetail(Root().lms.getAssociation(Root().lm2), Is().EqualTo(-1));
      AssertThatDetail(Root().lms.landmarkDB[Root().id1]->totalTimeObserved,
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
  ScenarioAttribute("hasChild", "true")

  void	SetUp()
  {
    lm1.pos.x = 24;
    lm1.pos.y = 42;
    lm2.pos.x = 10;
    lm2.pos.y = 8;
    lm3.pos.x = 12;
    lm3.pos.y = 8;
    timeObservedResult = 0;
  }

  When(there_are_no_landmark)
  {
    ScenarioAttribute("hasParent", "\t")

    void	SetUp()
    {
      Root().oldTimeObserved = Root().timeObservedResult;
      Root().lms.getClosestAssociation(&(Root().lm1), Root().idResult, Root().timeObservedResult);
    }

    Then(it_should_set_id_to_minus_one)
    {
      AssertThatDetail(Root().idResult, Is().EqualTo(-1));
    }

    Then(it_should_not_change_total_time_observed)
    {
      AssertThatDetail(Root().timeObservedResult, Is().EqualTo(Root().oldTimeObserved));
    }
  };

  When(there_is_one_landmark_with_not_enough_observation)
  {
    ScenarioAttribute("hasParent", "\t")

    void	SetUp()
    {
      Root().id1 = Root().lms.addToDB(Root().lm1);
      Root().oldTimeObserved = Root().timeObservedResult;
      Root().lms.getClosestAssociation(&(Root().lm1), Root().idResult, Root().timeObservedResult);
    }

    Then(it_should_set_id_to_minus_one)
    {
      AssertThatDetail(Root().idResult, Is().EqualTo(-1));
    }

    Then(it_should_not_change_total_time_observed)
    {
      AssertThatDetail(Root().timeObservedResult, Is().EqualTo(Root().oldTimeObserved));
    }
  };

  When(there_is_one_landmark_with_enough_observation)
  {
    ScenarioAttribute("hasParent", "\t")

    void	SetUp()
    {
      Root().id1 = Root().lms.addToDB(Root().lm1);
      Root().lms.landmarkDB[Root().id1]->totalTimeObserved = Landmarks::MINOBSERVATIONS + 1;
      Root().lms.getClosestAssociation(&(Root().lm1), Root().idResult, Root().timeObservedResult);
    }

    Then(it_should_set_id_to_landmark_id)
    {
      AssertThatDetail(Root().idResult, Is().EqualTo(Root().id1));
    }

    Then(it_should_change_total_time_observed)
    {
      AssertThatDetail(Root().timeObservedResult, Is().EqualTo(Root().lms.landmarkDB[Root().id1]->totalTimeObserved));
    }
  };

  When(there_is_more_than_one_landmark_without_enough_observation)
  {
    ScenarioAttribute("hasParent", "\t");

    void	SetUp()
    {
      Root().id1 = Root().lms.addToDB(Root().lm1);
      Root().id2 = Root().lms.addToDB(Root().lm2);
      Root().oldTimeObserved = Root().timeObservedResult;
      Root().lms.getClosestAssociation(&(Root().lm2), Root().idResult, Root().timeObservedResult);
    }

    Then(it_should_set_id_to_minus_one)
    {
      AssertThatDetail(Root().idResult, Is().EqualTo(-1));
    }

    Then(it_should_not_change_total_time_observed)
    {
      AssertThatDetail(Root().timeObservedResult, Is().EqualTo(Root().oldTimeObserved));
    }
  };

  When(there_is_more_than_one_landmark_and_one_with_enough_observation)
  {
    ScenarioAttribute("hasParent", "\t")

    void	SetUp()
    {
      Root().id1 = Root().lms.addToDB(Root().lm1);
      Root().id2 = Root().lms.addToDB(Root().lm2);
      Root().lms.landmarkDB[Root().id2]->totalTimeObserved = Landmarks::MINOBSERVATIONS + 1;
    }

    When(trying_with_the_one_that_has_enough_observation)
    {
      ScenarioAttribute("hasParent", "\t\t")

      void	SetUp()
      {
	Root().lms.getClosestAssociation(&(Root().lm2), Root().idResult, Root().timeObservedResult);
      }

      Then(it_should_set_id_to_good_landmarkId)
      {
	AssertThatDetail(Root().idResult, Is().EqualTo(Root().id2));
      }

      Then(it_should_change_total_time_observed)
      {
	AssertThatDetail(Root().timeObservedResult, Is().EqualTo(Root().lms.landmarkDB[Root().id2]->totalTimeObserved));
      }
    };

    When(trying_with_the_one_that_has_not_enough_observation)
    {
      ScenarioAttribute("hasParent", "\t\t")

      void	SetUp()
      {
	Root().lms.getClosestAssociation(&(Root().lm1), Root().idResult, Root().timeObservedResult);
      }

      Then(it_should_set_id_to_good_landmarkId)
      {
	AssertThatDetail(Root().idResult, Is().EqualTo(Root().id2));
      }

      Then(it_should_change_total_time_observed)
      {
	AssertThatDetail(Root().timeObservedResult, Is().EqualTo(Root().lms.landmarkDB[Root().id2]->totalTimeObserved));
      }
    };
  };

  When(there_is_more_than_one_landmark_and_all_with_enough_observation)
  {
    ScenarioAttribute("hasParent", "\t")

    void	SetUp()
    {
      Root().id1 = Root().lms.addToDB(Root().lm1);
      Root().id2 = Root().lms.addToDB(Root().lm2);
      Root().lms.landmarkDB[Root().id2]->totalTimeObserved = Landmarks::MINOBSERVATIONS + 1;
      Root().lms.landmarkDB[Root().id1]->totalTimeObserved = Landmarks::MINOBSERVATIONS + 1;
    }

    When(trying_with_one)
    {
      ScenarioAttribute("hasParent", "\t\t")

      void	SetUp()
      {
	Root().lms.getClosestAssociation(&(Root().lm1), Root().idResult, Root().timeObservedResult);
      }

      Then(it_should_set_id_to_good_landmarkId)
      {
	AssertThatDetail(Root().idResult, Is().EqualTo(Root().id1));
      }

      Then(it_should_change_total_time_observed)
      {
	AssertThatDetail(Root().timeObservedResult, Is().EqualTo(Root().lms.landmarkDB[Root().id1]->totalTimeObserved));
      }
    };

    When(trying_with_the_other)
    {
      ScenarioAttribute("hasParent", "\t\t")

      void	SetUp()
      {
	Root().lms.getClosestAssociation(&(Root().lm2), Root().idResult, Root().timeObservedResult);
      }

      Then(it_should_set_id_to_good_landmarkId)
      {
	AssertThatDetail(Root().idResult, Is().EqualTo(Root().id2));
      }

      Then(it_should_change_total_time_observed)
      {
	AssertThatDetail(Root().timeObservedResult, Is().EqualTo(Root().lms.landmarkDB[Root().id2]->totalTimeObserved));
      }
    };

    When(trying_with_one_that_is_not_in_db)
    {
      ScenarioAttribute("hasParent", "\t\t")

      void	SetUp()
      {
	Root().lms.getClosestAssociation(&(Root().lm3), Root().idResult, Root().timeObservedResult);
      }

      Then(it_should_set_id_to_the_closest_landmarkId)
      {
	AssertThatDetail(Root().idResult, Is().EqualTo(Root().id2));
      }

      Then(it_should_change_total_time_observed)
      {
	AssertThatDetail(Root().timeObservedResult, Is().EqualTo(Root().lms.landmarkDB[Root().id2]->totalTimeObserved));
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
    Agent	*agent = new Agent();
    double	x_view = 12.5;
    double	y_view = 36.2;

    agent->setPos(42.0, 22.0, 0.0);
    agent->setBearing(1.0);

    lm1.pos.x = (cos(agent->getBearing() * Landmarks::CONVERSION) * x_view - sin(agent->getBearing() * Landmarks::CONVERSION) * y_view) + agent->getPos().x;
    lm1.pos.y = (sin(agent->getBearing() * Landmarks::CONVERSION) * x_view + cos(agent->getBearing() * Landmarks::CONVERSION) * y_view) + agent->getPos().y;
;
    lm2.pos.x = 42;
    lm2.pos.y = 44;
    id1 = lms.addToDB(lm1);
    id2 = lms.addToDB(lm2);
    lms.landmarkDB[id2]->totalTimeObserved = Landmarks::MINOBSERVATIONS + 1;
    lms.landmarkDB[id1]->totalTimeObserved = Landmarks::MINOBSERVATIONS + 1;
    range = lms.distance(lm1.pos.x, lm1.pos.y, agent->getPos().x, agent->getPos().y);
    bearing = atan((lm1.pos.y - agent->getPos().y) / (lm1.pos.x - agent->getPos().x)) - agent->getBearing();
    lm3 = lms.getLandmark(x_view, y_view, *agent);
  }

  Then(it_should_return_one_of_the_db_landmark)
  {
    AssertThatDetail(lm3->id, Is().EqualTo(id1));
  }

  Then(it_should_set_range)
  {
    AssertThatDetail(lm3->range, Is().EqualToWithDelta(range, 0.0001));
  }

  Then(it_should_set_bearing)
  {
    AssertThatDetail(lm3->bearing, Is().EqualToWithDelta(bearing, 0.0001));
  }

  Landmarks	lms;
  Landmarks::Landmark	lm1;
  Landmarks::Landmark	lm2;
  Landmarks::Landmark	*lm3;
  double	range;
  double	bearing;
  int		id1;
  int		id2;
};

/**
 * Unit Test for updateLandmark(Landmark *)
 **/
When(updating_landmarks_with_a_landmark)
{
  ScenarioAttribute("hasChild", "true")

  void		SetUp()
  {
    lm1.pos.x = 42;
    lm1.pos.y = 24;
    lm2.pos.x = 12;
    lm2.pos.y = 5;

    id1 = lms.addToDB(lm1);
  }

  When(landmark_is_not_in_db)
  {
    ScenarioAttribute("hasParent", "\t")

    void	SetUp()
    {
      Root().oldDBSize = Root().lms.DBSize;
      Root().oldId = Root().lm2.id;
      Root().lm3 = Root().lms.updateLandmark(&(Root().lm2));
    }

    Then(it_should_add_the_landmark_to_the_db)
    {
      AssertThatDetail(Root().lms.DBSize, Is().EqualTo(Root().oldDBSize + 1));
    }

    Then(it_should_set_landmark_id_to_the_db_id)
    {
      AssertThatDetail(Root().lm3->id, Is().Not().EqualTo(Root().oldId));
      AssertThatDetail(Root().lm3->id, Is().EqualTo(Root().lms.DBSize - 1));
    }
  };

  When(landmark_is_in_db)
  {
    ScenarioAttribute("hasParent", "\t")

    void	SetUp()
    {
      Root().oldDBSize = Root().lms.DBSize;
      Root().oldId = Root().lm1.id;
      Root().lm3 = Root().lms.updateLandmark(&(Root().lm1));
    }

    Then(it_should_not_add_the_landmark_to_the_db)
    {
      AssertThatDetail(Root().lms.DBSize, Is().EqualTo(Root().oldDBSize));
    }

    Then(it_should_set_landmark_id_to_the_db_id)
    {
      AssertThatDetail(Root().lm3->id, Is().Not().EqualTo(Root().oldId));
      AssertThatDetail(Root().lm3->id, Is().EqualTo(Root().id1));
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
  ScenarioAttribute("hasChild", "true")

  void		SetUp()
  {
    double x_view = 2.6;
    double y_view = 15.24;

    agent = new Agent();
    agent->setPos(42.0, 24.0, 0.0);
    agent->setBearing(1.0);

    oldDBSize = lms.DBSize;
    lm1 = lms.updateLandmark(false, 0, x_view, y_view, *agent);
    distance = lms.distance(lm1->pos.x, lm1->pos.y, agent->getPos().x, agent->getPos().y);
    bearing = lms.calculateBearing(lm1->pos.x, lm1->pos.y, *agent);
    id1 = lm1->id;
  }


  When(landmark_is_not_in_db)
  {
    ScenarioAttribute("hasParent", "\t")

    Then(it_should_add_the_landmark_to_the_db)
    {
      AssertThatDetail(Root().lms.DBSize, Is().EqualTo(Root().oldDBSize + 1));
    }

    Then(it_should_return_the_new_landmark)
    {
      AssertThatDetail(Root().lm1->id, Is().Not().EqualTo(-1));
      AssertThatDetail(Root().lm1->bearing, Is().EqualToWithDelta(Root().bearing, 0.0001));
      AssertThatDetail(Root().lm1->range, Is().EqualToWithDelta(Root().distance, 0.0001));
      AssertThatDetail(Root().lm1->pos.x, Is().Not().EqualTo(0));
      AssertThatDetail(Root().lm1->pos.y, Is().Not().EqualTo(0));
    }
  };

  When(landmark_is_in_db)
  {
    ScenarioAttribute("hasParent", "\t")

    void	SetUp()
    {
      double x_view = 2.6;
      double y_view = 15.24;

      Root().oldDBSize = Root().lms.DBSize;
      Root().oldTimeObserved = Root().lms.landmarkDB[Root().id1]->totalTimeObserved;
      Root().lm2 = Root().lms.updateLandmark(true, Root().id1, x_view,
					     y_view, *Root().agent);
    }

    Then(it_should_not_add_the_landmark_to_the_db)
    {
      AssertThatDetail(Root().lms.DBSize, Is().EqualTo(Root().oldDBSize));
    }

    Then(it_should_return_the_old_landmark)
    {
      AssertThatDetail(Root().lm2->id, Is().EqualTo(Root().id1));
    }

    Then(it_should_add_one_to_timeobserved)
    {
      AssertThatDetail(Root().lm2->totalTimeObserved, Is().EqualTo(Root().oldTimeObserved + 1));
      AssertThatDetail(Root().lms.landmarkDB[Root().id1]->totalTimeObserved, Is().EqualTo(Root().oldTimeObserved + 1));
    }
  };

  Landmarks	lms;
  Landmarks::Landmark *lm1;
  Landmarks::Landmark *lm2;
  double	distance;
  double	bearing;
  Agent		*agent;
  int		id1;
  int		oldDBSize;
  int		oldTimeObserved;
};

/**
 * Unit test for updateLineLandmark()
 **/
When(updating_line_landmark)
{
  ScenarioAttribute("hasChild", "true")

  void	SetUp()
  {
    lm1.pos.x = 42;
    lm1.pos.y = 24;
    lm2.pos.x = 12;
    lm2.pos.y = 5;

    id1 = lms.addToDB(lm1);
  }

  When(landmark_is_not_in_db)
  {
    ScenarioAttribute("hasParent", "\t")

    void	SetUp()
    {
      Root().oldDBSize = Root().lms.DBSize;
      Root().idResult = Root().lms.updateLineLandmark(Root().lm2);
    }

    Then(it_should_add_landmark_to_db)
    {
      AssertThatDetail(Root().lms.DBSize, Is().EqualTo(Root().oldDBSize + 1));
    }

    Then(it_should_return_new_landmark_id)
    {
      AssertThatDetail(Root().idResult, Is().Not().EqualTo(-1));
      AssertThatDetail(Root().idResult, Is().EqualTo(Root().lms.DBSize - 1));
    }
  };

  When(landmark_is_in_db)
  {
    ScenarioAttribute("hasParent", "\t")

    void	SetUp()
    {
      Root().lms.addToDB(Root().lm2);
      Root().oldDBSize = Root().lms.DBSize;
      Root().idResult = Root().lms.updateLineLandmark(Root().lm1);
    }

    Then(it_should_not_add_landmark_to_db)
    {
      AssertThatDetail(Root().lms.DBSize, Is().EqualTo(Root().oldDBSize));
    }

    Then(it_should_return_db_landmark_id)
    {
      AssertThatDetail(Root().idResult, Is().EqualTo(Root().id1));
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
    lm1.pos.x = 2.5;
    lm1.pos.y = 3.5;
    lm2.pos.x = 1.5;
    lm2.pos.y = 2.5;
    id1 = lms.addToDB(lm1);
    id2 = lms.addToDB(lm2);
    lms.landmarkDB[id1]->totalTimeObserved = Landmarks::MINOBSERVATIONS + 1;
    lms.landmarkDB[id2]->totalTimeObserved = Landmarks::MINOBSERVATIONS + 1;
    lm3 = lms.getOrigin();
  }

  Then(it_should_have_id_of_landmark_closest_from_origin)
  {
    AssertThatDetail(lm3->id, Is().Not().EqualTo(-1));
    AssertThatDetail(lm3->id, Is().EqualTo(id2));
  }

  Then(it_should_have_default_value)
  {
    AssertThatDetail(lm3->pos.x, Is().EqualTo(0));
    AssertThatDetail(lm3->pos.y, Is().EqualTo(0));
    AssertThatDetail(lm3->range, Is().EqualTo(-1));
    AssertThatDetail(lm3->bearing, Is().EqualTo(-1));
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
    point.x = (double)(b / ((-1.0 / a) - a));
    point.y = (double)(((-1.0 / a) * b) / ((-1.0 / a) - a));
    lm1.pos.x = 42.5;
    lm1.pos.y = 23.5;
    lm2.pos.x = point.x + 0.12;
    lm2.pos.y = point.y - 0.02;
    id1 = lms.addToDB(lm1);
    id2 = lms.addToDB(lm2);
    lms.landmarkDB[id1]->totalTimeObserved = Landmarks::MINOBSERVATIONS + 1;
    lms.landmarkDB[id2]->totalTimeObserved = Landmarks::MINOBSERVATIONS + 1;
    lm3 = lms.getLine(a, b);
  }

  Then(it_should_have_id_of_landmark_closest_from_origin)
  {
    AssertThatDetail(lm3->id, Is().Not().EqualTo(-1));
    AssertThatDetail(lm3->id, Is().EqualTo(id2));
  }

  Then(it_should_have_default_value)
  {
    AssertThatDetail(lm3->pos.x, Is().EqualTo(point.x));
    AssertThatDetail(lm3->pos.y, Is().EqualTo(point.y));
    AssertThatDetail(lm3->a, Is().EqualTo(a));
    AssertThatDetail(lm3->b, Is().EqualTo(b));
    AssertThatDetail(lm3->range, Is().EqualTo(-1));
    AssertThatDetail(lm3->bearing, Is().EqualTo(-1));
  }

  Landmarks	lms;
  Landmarks::Landmark	lm1;
  Landmarks::Landmark	lm2;
  Landmarks::Landmark	*lm3;
  int		id1;
  int		id2;
  double	a;
  double	b;
  pcl::PointXY	point;
};

When(getting_landmarks_nearest_to_line_with_robot_pos)
{
  void	SetUp()
  {
    agent = new Agent();
    a = 35.5;
    b = 26.3;
    agent->setPos(5.2, 4.2, 0.0);
    agent->setBearing(0.1);

    double ao = -1.0 / a;
    point.x = b / (ao - a);
    point.y = (ao * b) / (ao - a);
    double x = b / (ao - a);
    double y = (ao * b) / (ao - a);

    range = sqrt(pow(x - agent->getPos().x, 2) + pow(y - agent->getPos().y, 2));
    bearing = atan((y - agent->getPos().y) / (x - agent->getPos().x)) - agent->getBearing();

    double bo = agent->getPos().y - ao * agent->getPos().x;
    double px = (b - bo) / (ao - a);
    double py = ((ao * (b - bo)) / (ao - a)) + bo;

    rangeError = lms.distance(agent->getPos().x, agent->getPos().y, px, py);
    bearingError = atan((py - agent->getPos().y) / (px - agent->getPos().x)) - agent->getBearing();

    lm1.pos.x = 42.5;
    lm1.pos.y = 23.5;
    lm2.pos.x = point.x + 0.12;
    lm2.pos.y = point.y - 0.02;
    id1 = lms.addToDB(lm1);
    id2 = lms.addToDB(lm2);
    lms.landmarkDB[id1]->totalTimeObserved = Landmarks::MINOBSERVATIONS + 1;
    lms.landmarkDB[id2]->totalTimeObserved = Landmarks::MINOBSERVATIONS + 2;
    lm3 = lms.getLineLandmark(a, b, *agent);
  }

  Then(it_should_have_id_of_landmark_closest_from_origin)
  {
    AssertThatDetail(lm3->id, Is().Not().EqualTo(-1));
    AssertThatDetail(lm3->id, Is().EqualTo(id2));
  }

  Then(it_should_have_default_value)
  {
    AssertThatDetail(lm3->pos.x, Is().EqualTo(point.x));
    AssertThatDetail(lm3->pos.y, Is().EqualTo(point.y));
    AssertThatDetail(lm3->a, Is().EqualTo(a));
    AssertThatDetail(lm3->b, Is().EqualTo(b));
    AssertThatDetail(lm3->range, Is().EqualTo(range));
    AssertThatDetail(lm3->bearing, Is().EqualTo(bearing));
    AssertThatDetail(lm3->rangeError, Is().EqualTo(rangeError));
    AssertThatDetail(lm3->bearingError, Is().EqualTo(bearingError));
  }

  Landmarks	lms;
  Landmarks::Landmark	lm1;
  Landmarks::Landmark	lm2;
  Landmarks::Landmark	*lm3;
  int		id1;
  int		id2;
  double	a;
  double	b;
  pcl::PointXY	point;
  Agent		*agent;
  double	range;
  double	bearing;
  double	rangeError;
  double	bearingError;
  int		totalTimesObserved;
};

When(extracting_spike_landmark)
{
  void	SetUp()
  {
    srand(42);
    TestSlamCommon::generateData(cloud, 500);

    agent = new Agent();
    agent->setPos(2.0, 4.0, 0.0);
    agent->setBearing(0.2);

    lm1.pos.x = 42.5;
    lm1.pos.y = 24.1;
    lm2.pos.x = 12.2;
    lm2.pos.y = 2.0;
    id1 = lms.addToDB(lm1);
    id2 = lms.addToDB(lm2);
    lms.landmarkDB[id1]->totalTimeObserved = Landmarks::MINOBSERVATIONS + 1;
    lms.landmarkDB[id2]->totalTimeObserved = Landmarks::MINOBSERVATIONS + 2;
    result = lms.extractSpikeLandmarks(cloud, *agent);
  }

  Then(it_should_have_some_landmarks)
  {
    AssertThatDetail(result.size(), Is().GreaterThan(0));
  }

  Then(each_landmark_should_have_a_good_id)
  {
    for (std::vector<Landmarks::Landmark *>::iterator it = result.begin(); it != result.end(); ++it)
      {
	AssertThatDetail((*it)->id, Is().Not().EqualTo(-1));
      }
  }

  Landmarks	lms;
  Landmarks::Landmark	lm1;
  Landmarks::Landmark	lm2;
  int		id1;
  int		id2;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  Agent		*agent;
  std::vector<Landmarks::Landmark *> result;
};

When(removing_double_landmarks)
{
  void	SetUp()
  {
    srand(42);
    TestSlamCommon::generateData(cloud, 500);

    agent = new Agent();
    agent->setPos(2.0, 4.0, 0.0);
    agent->setBearing(0.2);

    lm1.pos.x = (cos(agent->getBearing() * Landmarks::CONVERSION) * cloud.points[1].x - sin(agent->getBearing() * Landmarks::CONVERSION) * cloud.points[1].y) + agent->getPos().x;
    lm1.pos.y = (sin(agent->getBearing() * Landmarks::CONVERSION) * cloud.points[1].x + cos(agent->getBearing() * Landmarks::CONVERSION) * cloud.points[1].y) + agent->getPos().y;

    lm2.pos.x = (cos(agent->getBearing() * Landmarks::CONVERSION) * cloud.points[19].x - sin(agent->getBearing() * Landmarks::CONVERSION) * cloud.points[19].y) + agent->getPos().x;
    lm2.pos.y = (sin(agent->getBearing() * Landmarks::CONVERSION) * cloud.points[19].x + cos(agent->getBearing() * Landmarks::CONVERSION) * cloud.points[19].y) + agent->getPos().y;
;
    id1 = lms.addToDB(lm1);
    id2 = lms.addToDB(lm2);
    lms.landmarkDB[id1]->totalTimeObserved = Landmarks::MINOBSERVATIONS + 1;
    lms.landmarkDB[id2]->totalTimeObserved = Landmarks::MINOBSERVATIONS + 2;
    extracted = lms.extractSpikeLandmarks(cloud, *agent);
    result = lms.removeDouble(extracted);
  }

  Then(it_should_not_have_more_landmarks_than_db)
  {
    AssertThatDetail(result.size(), Is().Not().EqualTo(0));
    AssertThatDetail(result.size(), Is().LessThan(lms.DBSize + 1));
  }


  Landmarks	lms;
  Landmarks::Landmark	lm1;
  Landmarks::Landmark	lm2;
  int		id1;
  int		id2;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  Agent		*agent;
  std::vector<Landmarks::Landmark *> extracted;
  std::vector<Landmarks::Landmark *> result;
};


When(getting_aligned_landmark_data)
{
  void	SetUp()
  {
    srand(42);
    TestSlamCommon::generateData(cloud, 500);

    agent = new Agent();
    agent->setPos(2.0, 4.0, 0.0);
    agent->setBearing(0.2);

    lm1.pos.x = (cos((1 * lms.degreePerScan * Landmarks::CONVERSION) + (agent->getBearing() * Landmarks::CONVERSION)) * cloud.points[1].z)
      + agent->getPos().x;
    lm1.pos.y = (sin((1 * lms.degreePerScan * Landmarks::CONVERSION) + (agent->getBearing() * Landmarks::CONVERSION)) * cloud.points[1].z)
      + agent->getPos().y;
    lm2.pos.x = (cos((19 * lms.degreePerScan * Landmarks::CONVERSION) + (agent->getBearing() * Landmarks::CONVERSION)) * cloud.points[19].z)
      + agent->getPos().x;
    lm2.pos.y = (sin((19 * lms.degreePerScan * Landmarks::CONVERSION) + (agent->getBearing() * Landmarks::CONVERSION)) * cloud.points[19].z)
      + agent->getPos().y;
    id1 = lms.addToDB(lm1);
    id2 = lms.addToDB(lm2);
    lms.landmarkDB[id1]->totalTimeObserved = Landmarks::MINOBSERVATIONS + 1;
    lms.landmarkDB[id2]->totalTimeObserved = Landmarks::MINOBSERVATIONS + 2;
    extracted = lms.extractSpikeLandmarks(cloud, *agent);
    lms.alignLandmarkData(extracted, matched, id, ranges, bearings, lmrk, exlmrk);
  }

  Then(it_should_have_good_values)
  {
    int	i = 0;

    for(std::vector<pcl::PointXY>::iterator it = lmrk.begin(); it != lmrk.end(); ++it)
      {
	AssertThatDetail(id[i], Is().GreaterThan(-1));
	AssertThatDetail(matched[i], Is().EqualTo(true));
	AssertThatDetail(lmrk[i].x, Is().EqualTo(lms.landmarkDB[id[i]]->pos.x));
	AssertThatDetail(lmrk[i].y, Is().EqualTo(lms.landmarkDB[id[i]]->pos.y));
	++i;
      }
  }

  Landmarks	lms;
  Landmarks::Landmark	lm1;
  Landmarks::Landmark	lm2;
  int		id1;
  int		id2;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  Agent		*agent;
  std::vector<Landmarks::Landmark *> extracted;
  bool		*matched;
  int		*id;
  double	*ranges;
  double	*bearings;
  std::vector<pcl::PointXY> lmrk;
  std::vector<pcl::PointXY> exlmrk;
};


/**
 * Unit Test for updateAndAddLineLandmarks(std::vector<Landmarks::Landmark *>)
 **/
When(update_and_add_line_landmarks)
{
  ScenarioAttribute("hasChild", "true")

  void SetUp()
  {
    lmrk1.pos.x = 21;
    lmrk1.pos.y = 42;
    lmrk2.pos.x = 12;
    lmrk2.pos.y = 24;
  }

  When(Landmarks_are_not_in_db)
  {
    ScenarioAttribute("hasParent", "\t")
    void SetUp()
    {
      Root().lmrks.push_back(&(Root().lmrk1));
      Root().lmrks.push_back(&(Root().lmrk2));

      Root().oldIds.push_back(Root().lmrk1.id);
      Root().oldIds.push_back(Root().lmrk2.id);
      Root().oldDBsize = Root().lms.DBSize;
      Root().dbLmrks = Root().lms.updateAndAddLineLandmarks(Root().lmrks);
      for (unsigned int i = 0; i < Root().dbLmrks.size(); ++i)
	Root().ids.push_back(Root().dbLmrks[i]->id);
    }

    Then(it_should_add_landmarks_to_the_db)
    {
      AssertThatDetail(Root().lms.DBSize, Is().EqualTo(2));
    }

    Then(it_should_return_updated_landmarks_with_good_values)
    {
      AssertThatDetail(Root().dbLmrks.size(), Is().EqualTo(2));
      AssertThatDetail(Root().oldIds.size(), Is().EqualTo(Root().ids.size()));
      for (unsigned int i = 0; i < Root().oldIds.size(); ++i)
	AssertThatDetail(Root().oldIds[i], Is().Not().EqualTo(Root().ids[i]));
    }
  };

  When(Landmarks_are_in_db)
  {
    ScenarioAttribute("hasParent", "\t")

    void SetUp()
    {
      Root().dbLmrks.push_back(&(Root().lmrk1));
      Root().dbLmrks.push_back(&(Root().lmrk2));

      Root().lms.addToDB(Root().lmrk1);
      Root().lms.addToDB(Root().lmrk2);

      Root().oldDBsize = Root().lms.DBSize;
      for (unsigned int i = 0; i < Root().dbLmrks.size(); ++i)
	Root().oldIds.push_back(Root().dbLmrks[i]->id);
      Root().lmrks = Root().lms.updateAndAddLineLandmarks(Root().dbLmrks);
      for (unsigned int i = 0; i < Root().lmrks.size(); ++i)
	Root().ids.push_back(Root().lmrks[i]->id);
    }

    Then(it_should_not_add_landmarks_to_the_db)
    {
      AssertThatDetail(Root().lms.DBSize, Is().EqualTo(Root().oldDBsize));
    }

    Then(it_should_return_updated_landmarks_with_good_values)
    {
      AssertThatDetail(Root().lmrks.size(), Is().EqualTo(2));
      AssertThatDetail(Root().oldIds.size(), Is().EqualTo(Root().ids.size()));
      for (unsigned int i = 0; i < Root().oldIds.size(); ++i)
	AssertThatDetail(Root().oldIds[i], Is().Not().EqualTo(Root().ids[i]));
    }
  };

  Landmarks lms;
  Landmarks::Landmark lmrk1;
  Landmarks::Landmark lmrk2;
  std::vector<Landmarks::Landmark *> lmrks;
  std::vector<Landmarks::Landmark *> dbLmrks;
  unsigned int oldDBsize;
  std::vector<unsigned int> ids;
  std::vector<unsigned int> oldIds;
};

/**
 * Unit test for updateAndAddLandmarkUsingEKFResults(bool matched[], unsigned int numberMatched, int id[], std::vector<pcl::PointXYZ> const &pos, Agent const &agent)
 **/
When(Update_And_Add_Landmark_Using_EKF_Results)
{
  ScenarioAttribute("hasChild", "true")

    void SetUp()
    {
      agent = new Agent();
      agent->setPos(42.0, 24.0, 0.0);
      agent->setBearing(1.0);
      id[0] = 0;
      id[1] = 1;
      pos.push_back(pcl::PointXYZ(25.4, 31.1, 0.0));
      pos.push_back(pcl::PointXYZ(35.4, 11.1, 0.0));
    }

  When(Landmarks_are_not_in_db)
  {
    ScenarioAttribute("hasParent", "\t")
      void SetUp()
      {
	Root().matched[0] = false;
	Root().matched[1] = false;
	Root().ldmks = Root().lms.updateAndAddLandmarkUsingEKFResults(Root().matched, 2, Root().id, Root().pos, *Root().agent);
      }

    Then(it_should_add_landmarks_to_the_db)
    {
      AssertThatDetail(Root().lms.DBSize, Is().EqualTo(2));
    }

    Then(it_should_return_updated_landmarks_with_good_values)
    {
      AssertThatDetail(Root().ldmks.size(), Is().EqualTo(2));
      for (unsigned int i = 0; i < Root().ldmks.size(); ++i)
	{
	  AssertThatDetail(Root().ldmks[i]->id, Is().EqualTo(i));
	  // AssertThatDetail(Root().ldmks[i]->range, Is().EqualTo(1.4));
	  // AssertThatDetail(Root().ldmks[i]->bearing, Is().EqualTo(4));
	}
    }
  };

  When(Landmarks_are_in_db)
  {
    ScenarioAttribute("hasParent", "\t")

      void SetUp()
      {
	Root().matched[0] = true;
	Root().matched[1] = true;
	ldmk1.id = Root().id[0];
	ldmk2.id = Root().id[1];
	// ldmk1.range = Root().ranges[0];
	// ldmk2.range = Root().ranges[1];
	// ldmk1.bearing = Root().bearings[0];
	// ldmk2.bearing = Root().bearings[1];
	Root().lms.addToDB(ldmk1);
	Root().lms.addToDB(ldmk2);
	oldSize = Root().lms.DBSize;
	Root().ldmks = Root().lms.updateAndAddLandmarkUsingEKFResults(Root().matched, 2, Root().id, Root().pos, *Root().agent);
      }

    Then(it_should_not_add_landmarks_to_the_db)
    {
      AssertThatDetail(Root().lms.DBSize, Is().EqualTo(oldSize));
    }

    Then(it_should_return_updated_landmarks_with_good_values)
    {
      AssertThatDetail(Root().ldmks.size(), Is().EqualTo(2));
      for (unsigned int i = 0; i < Root().ldmks.size(); ++i)
	{
	  AssertThatDetail(Root().ldmks[i]->id, Is().EqualTo(i));
	  // AssertThatDetail(Root().ldmks[i]->range, Is().EqualTo(1.4));
	  // AssertThatDetail(Root().ldmks[i]->bearing, Is().EqualTo(4));
	  AssertThatDetail(Root().ldmks[i]->totalTimeObserved, Is().EqualTo(2));
	}
    }

    Landmarks::Landmark ldmk1;
    Landmarks::Landmark ldmk2;
    unsigned int oldSize;
  };

  Landmarks	lms;
  std::vector<Landmarks::Landmark *> ldmks;
  std::vector<pcl::PointXYZ> pos;
  bool		matched[2];
  int		id[2];
  double	ranges[2];
  double	bearings[2];
  Agent		*agent;
};

/*
** Unit test for removeBadLandmarks
*/
When(Remove_bad_landmarks)
{
  ScenarioAttribute("hasChild", "true")

  void	SetUp()
  {
    goodLandmark1.pos.x = 1.2;
    goodLandmark1.pos.y = 3.4;
    goodLandmark2.pos.x = 0.9;
    goodLandmark2.pos.y = 3.6;
    goodLandmark3.pos.x = 1.5;
    goodLandmark3.pos.y = 3.1;
    agent = new Agent();
  }

  When(All_Landmarks_are_in_rectangle)
  {
    ScenarioAttribute("hasParent", "\t")

    void SetUp()
    {
      pcl::PointCloud<pcl::PointXYZ> cloud;

      cloud.width    = 4;
      cloud.height   = 1;
      cloud.is_dense = false;
      cloud.points.resize(cloud.width * cloud.height);

      cloud.points[0].z = 2.2;
      cloud.points[1].z = 4.6;
      cloud.points[2].z = 1.8;
      cloud.points[3].z = 3.3;
      cloud.points[0].x = 0;
      cloud.points[1].x = 0;
      cloud.points[2].x = 0;
      cloud.points[3].x = 0;
      cloud.points[0].y = 0;
      cloud.points[1].y = 0;
      cloud.points[2].y = 0;
      cloud.points[3].y = 0;

      Root().badLandmark1.pos.x = 1.4;
      Root().badLandmark1.pos.y = 3.5;

      Root().agent->setPos(42.0, 3.0, 0.0);
      Root().agent->setBearing(60.2);

      Root().lms.addToDB(Root().goodLandmark1);
      Root().lms.addToDB(Root().goodLandmark2);
      Root().lms.addToDB(Root().goodLandmark3);
      Root().badLandmarkID = Root().lms.addToDB(Root().badLandmark1);

      Root().lms.landmarkDB[3]->life = 1;

      Root().oldDBSize = Root().lms.DBSize;
      Root().lms.removeBadLandmarks(cloud, *Root().agent);
    }

    Then(DBSize_should_be_smaller)
    {
      AssertThatDetail(Root().lms.DBSize, Is().EqualTo(Root().oldDBSize - 1));
    }

    Then(landmarkDB_should_not_contain_badlandmark)
    {
      for (int i = 0; i < Root().lms.DBSize; ++i)
	AssertThatDetail(Root().lms.landmarkDB[i]->id, Is().Not().EqualTo(Root().badLandmarkID));
    }
  };

  When(Badlandmark_is_not_in_rectangle)
  {
    ScenarioAttribute("hasParent", "\t")

    void SetUp()
    {
      pcl::PointCloud<pcl::PointXYZ> cloud;

      cloud.width    = 4;
      cloud.height   = 1;
      cloud.is_dense = false;
      cloud.points.resize(cloud.width * cloud.height);

      cloud.points[0].z = 2.2;
      cloud.points[1].z = 4.6;
      cloud.points[2].z = 1.8;
      cloud.points[3].z = 3.3;
      cloud.points[0].x = 0;
      cloud.points[1].x = 0;
      cloud.points[2].x = 0;
      cloud.points[3].x = 0;
      cloud.points[0].y = 0;
      cloud.points[1].y = 0;
      cloud.points[2].y = 0;
      cloud.points[3].y = 0;

      Root().badLandmark1.pos.x = 75.4;
      Root().badLandmark1.pos.y = 90.8;

      Root().agent->setPos(25.7, -33.2, 0.0);
      Root().agent->setBearing(160.4);

      Root().lms.addToDB(Root().goodLandmark1);
      Root().lms.addToDB(Root().goodLandmark2);
      Root().lms.addToDB(Root().goodLandmark3);
      Root().lms.addToDB(Root().badLandmark1);

      Root().lms.landmarkDB[3]->life = 1;

      Root().oldDBSize =  Root().lms.DBSize;
      Root().lms.removeBadLandmarks(cloud, *Root().agent);
    }

    Then(DBSize_should_not_change)
    {
      AssertThatDetail(Root().lms.DBSize, Is().EqualTo(Root().oldDBSize));
    }

    Then(landmarkDB_contains_badlandmark)
    {
      AssertThatDetail(Root().lms.landmarkDB[3]->pos.x, Is().EqualTo(Root().badLandmark1.pos.x));
      AssertThatDetail(Root().lms.landmarkDB[3]->pos.y, Is().EqualTo(Root().badLandmark1.pos.y));
    }
  };

  Landmarks::Landmark goodLandmark1;
  Landmarks::Landmark goodLandmark2;
  Landmarks::Landmark goodLandmark3;
  Landmarks::Landmark goodLandmark4;
  Landmarks::Landmark badLandmark1;
  int badLandmarkID;
  unsigned int oldDBSize;
  Landmarks lms;
  Agent	*agent;
};

/*
** Unit test for extractLineLandmarks(double[], unsigned int, Agent const &)
*/
When(extracting_line_landmark)
{
  void	SetUp()
  {
    /*
    ** depend du Ransac consensus (static variable)
    ** Datasize = 150 = Landmarks::RANSAC_CONSENSUS * 5
    */
    srand(42);
    TestSlamCommon::generateData(cloud, 500);

    agent = new Agent();
    agent->setPos(2.0, 4.0, 0.0);
    agent->setBearing(0.2);

    lm1.pos.x = 42.5;
    lm1.pos.y = 24.1;
    lm2.pos.x = 12.2;
    lm2.pos.y = 2.0;
    id1 = lms.addToDB(lm1);
    id2 = lms.addToDB(lm2);
    lms.landmarkDB[id1]->totalTimeObserved = Landmarks::MINOBSERVATIONS + 1;
    lms.landmarkDB[id2]->totalTimeObserved = Landmarks::MINOBSERVATIONS + 2;
    result = lms.extractLineLandmarks(cloud, *agent);
  }

  Then(it_should_have_some_landmarks)
  {
    AssertThatDetail(result.size(), Is().GreaterThan(0));
  }

  Then(each_landmark_should_have_a_good_id)
  {
    for (std::vector<Landmarks::Landmark *>::iterator it = result.begin(); it != result.end(); ++it)
      {
	AssertThatDetail((*it)->id, Is().Not().EqualTo(-1));
      }
  }

  Landmarks	lms;
  Landmarks::Landmark	lm1;
  Landmarks::Landmark	lm2;
  int		id1;
  int		id2;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  std::vector<Landmarks::Landmark *> result;
  Agent	*agent;
};
