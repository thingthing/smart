#include <igloo/igloo_alt.h>
#include <Landmarks.hh>

// Add true landmarks class when ready
namespace	Result
{
  int		id = -1;
  int		life = LIFE;
  int		totalTimesObserved = 0;
  double	range = -1;
  double	bearing = -1;
  double	pos[2] = {0, 0};
};

using namespace igloo;

When(creating_a_landmark)
{
  Then(it_should_have_default_value)
    {
      Assert::That(lm.id, Is().EqualTo(::Result::id));
      Assert::That(lm.life, Is().EqualTo(::Result::life));
      Assert::That(lm.totalTimeObserved, Is().EqualTo(::Result::totalTimesObserved));
      Assert::That(lm.range, Is().EqualTo(::Result::range));
      Assert::That(lm.bearing, Is().EqualTo(::Result::bearing));
      Assert::That(lm.pos[0], Is().EqualTo(::Result::pos[0]));
      Assert::That(lm.pos[1], Is().EqualTo(::Result::pos[1]));
    }

  ::Landmarks::Landmark	lm;
};
