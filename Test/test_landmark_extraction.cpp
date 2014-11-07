#include <igloo/igloo_alt.h>
#include <Landmarks.hh>

// Add true landmarks class when ready
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
  double	defaultdegreePerScan = 0.5;
  double	degreePerScan = 4.2;
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
  Then(should_have_default_landmarks_value)
  {
    Assert::That(lms.getDBSize(), Is().EqualTo(::Landmarks_Result::DBSize));
    Assert::That(lms.degreePerScan, Is().EqualTo(::Landmarks_Result::defaultdegreePerScan));
  }

  Landmarks lms;
};
