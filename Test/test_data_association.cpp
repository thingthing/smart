#include "igloo/igloo_alt.h"
#include "Landmarks.hh"
#include "DataAssociation.hh"
#include "CustomConstraint.hh"

using namespace igloo;

When(creating_data_association)
{
  void SetUp()
    {
      landmarks = new Landmarks();
      datas = new DataAssociation(landmarks);
    }

  Then(it_should_set_the_db)
    {
      AssertThatDetail(datas->_landmarkDb, Is().EqualTo(landmarks));
    }

  ::DataAssociation *datas;
  ::Landmarks *landmarks;
};

When(getting_db_from_data_association)
{
  void SetUp()
    {
      landmarks = new Landmarks();
      datas._landmarkDb = landmarks;
    }

  Then(it_should_return_the_db)
    {
      AssertThatDetail(landmarks, Is().EqualTo(datas.getLandmarkDb()));
    }

  ::DataAssociation datas;
  ::Landmarks *landmarks;
};

/*
 * Test landmark association
 */
When(associating_landmarks)
{
  ScenarioAttribute("hasChild", "true")

  void SetUp()
    {
      landmarks = new Landmarks();
      datas = new DataAssociation(landmarks);
      to_test = new Landmarks::Landmark();
      id = to_test->id;
      totalTimeObserved = to_test->totalTimeObserved;
    }

  When(landmarks_db_is_empty)
  {
    ScenarioAttribute("hasParent", "\t")

      void SetUp()
      {
	Root().result = Root().datas->associateLandmarks(Root().to_test);
      }

    Then(it_should_return_false)
    {
      AssertThatDetail(Root().result, Is().EqualTo(false));
    }

    Then(it_should_not_change_landmark_data)
    {
      AssertThatDetail(Root().to_test->id, Is().EqualTo(Root().id));
      AssertThatDetail(Root().to_test->totalTimeObserved,
		   Is().EqualTo(Root().totalTimeObserved));
    }
  };

  When(landmarks_db_is_not_empty)
  {
    ScenarioAttribute("hasParent", "\t")

      void SetUp()
      {
	Root().new_id = Root().landmarks->addToDB(*Root().to_test);
	Root().landmarks->landmarkDB[Root().new_id]->totalTimeObserved =
	  Landmarks::MINOBSERVATIONS + 1;
	Root().result = Root().datas->associateLandmarks(Root().to_test);
      }

    Then(it_should_return_true)
    {
      AssertThatDetail(Root().result, Is().EqualTo(true));
    }

    Then(it_should_change_landmark_data)
    {
      AssertThatDetail(Root().to_test->id, Is().Not().EqualTo(Root().id));
      AssertThatDetail(Root().to_test->totalTimeObserved,
		   Is().Not().EqualTo(Root().totalTimeObserved));
    }

    Then(it_should_set_good_landmark_data)
    {
      AssertThatDetail(Root().to_test->id, Is().EqualTo(Root().new_id));
      AssertThatDetail(Root().to_test->totalTimeObserved,
		   Is().EqualTo(Landmarks::MINOBSERVATIONS + 1));
    }
  };

  ::DataAssociation *datas;
  ::Landmarks *landmarks;
  ::Landmarks::Landmark *to_test;
  bool	result;
  int	id;
  int	totalTimeObserved;
  int	new_id;
};
