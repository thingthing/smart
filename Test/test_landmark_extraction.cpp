#include <igloo/igloo_alt.h>
//#include <Landmarks.hh>

// Add true landmarks class when ready
class Landmarks
{
public:
  static const int LIFE = 42;
  class Landmark
  {
  public:
    Landmark() {
    }
    
  public:
    int	id;
    int totalTimesObserved;
    int life;
    int	a;
    int	b;
  };

};

namespace	Result
{
  int		id = -1;
  int		life = ::Landmarks::LIFE;
  int		totalTimesObserved = -1;
  double	a = -1;
  double	b = -1;
};

using namespace igloo;

When(creating_a_landmark)
{
  Then(it_should_have_default_value)
    {
      Assert::That(lm.id, Is().EqualTo(::Result::id));
      Assert::That(lm.life, Is().EqualTo(::Result::life));
      Assert::That(lm.totalTimesObserved, Is().EqualTo(::Result::totalTimesObserved));
      Assert::That(lm.a, Is().EqualTo(::Result::a));
      Assert::That(lm.b, Is().EqualTo(::Result::b));
    }
  
  ::Landmarks::Landmark	lm;
};
