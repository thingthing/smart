#include <igloo/igloo_alt.h>
#include <Landmarks.hh>

namespace	Landmark_Result
{
  int		id = -1;
  int		life = LIFE;
  int		totalTimesObserved = 0;
  double	range = -1;
  double	bearing = -1;
  double	pos[2] = {0.0, 0.0};
};

namespace	Landmarks_Result
{
  int		DBSize = 0;
  int		EKFLandmarks = 0;
  double	defaultdegreePerScan = DEGREESPERSCAN;
  double	degreePerScan = 0.42; 
  int		sizeIDtoID = MAXLANDMARKS;
  int		sizelandmarkDB = MAXLANDMARKS;
};

using namespace igloo;

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

When(creating_Landmarks)
{
  Then(it_should_have_default_landmarks_value)
  {
    Assert::That(lms.DBSize, Is().EqualTo(::Landmarks_Result::DBSize));
    Assert::That(lms.EKFLandmarks, Is().EqualTo(::Landmarks_Result::EKFLandmarks));
    Assert::That(lms.degreePerScan, Is().EqualTo(::Landmarks_Result::defaultdegreePerScan));
    Assert::That(lms.IDtoID.capacity(), Is().EqualTo(::Landmarks_Result::sizeIDtoID));
    Assert::That(lms.landmarkDB.capacity(), Is().EqualTo(::Landmarks_Result::sizelandmarkDB));
  }

  Landmarks lms;
};

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

