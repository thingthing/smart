#include "igloo/igloo_alt.h"
#include "Landmarks.hh"
#include "DataAssociation.hh"
#include "CustomConstraint.hh"
#include "test_slam_common.hh"

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

When(testing_validation_gate)
{
  ScenarioAttribute("hasChild", "true")

    void	SetUp()
    {

      agent = new Agent();
      agent->setPos(2.0, 4.0, 0.0);
      agent->setBearing(0.2);

      int	r = 0;
      int	size = 0;
      ::Landmarks	*lms = new Landmarks();

      while (size == 0)
	{
	  srand(r);

	  TestSlamCommon::generateData(data, numberSample);

	  landmarksTest = lms->extractLineLandmarks(data, numberSample, *agent);

	  size = landmarksTest.size();
	  ++r;
	}
      datas = new DataAssociation(lms);
    }

  When(no_landmarks_are_in_db)
  {
    ScenarioAttribute("hasParent", "\t")

      void	SetUp()
      {
	Root().result = Root().datas->validationGate(Root().data, Root().numberSample, *Root().agent);
      }

    Then(it_should_add_landmarks_to_result)
    {
      AssertThatDetail(Root().result.size(), Is().GreaterThan(0));
    }
  };

  When(landmarks_are_already_in_db_and_already_enough_observed)
  {
    ScenarioAttribute("hasParent", "\t")

      void	SetUp()
      {
	for(std::vector<Landmarks::Landmark *>::iterator it = Root().landmarksTest.begin(); it != Root().landmarksTest.end(); ++it)
	  {
	    ids.push_back(Root().datas->getLandmarkDb()->addToDB(**it));
	  }

	Root().oldDbSize = Root().datas->getLandmarkDb()->getDBSize();
	Root().result = Root().datas->validationGate(Root().data, Root().numberSample, *Root().agent);
      }

    Then(it_should_have_less_landmark_in_result_than_in_db)
    {
      AssertThatDetail(Root().result.size(), Is().LessThan(Root().landmarksTest.size()));
    }

    Then(it_shoud_add_observed_time_to_landmarks_observed)
    {
      bool	isObserved = false;

      for(std::vector<int>::iterator it = ids.begin(); it != ids.end(); ++it)
	{
	  if (Root().datas->getLandmarkDb()->landmarkDB[*it]->totalTimeObserved > 1)
	    {
	      isObserved = true;
	      break;
	    }
	}
      AssertThatDetail(isObserved, Is().EqualTo(true));
    }

    std::vector<int> ids;
  };

  static const int numberSample = 150;
  ::DataAssociation *datas;
  std::vector<Landmarks::Landmark *>	landmarksTest;
  pcl::PointXYZ	data[numberSample];
  Agent	*agent;
  std::vector<Landmarks::Landmark *> result;
  int		oldDbSize;
};
