#include "Landmarks.hh"
#include "CustomConstraint.hh"

IsSameLandmark::IsSameLandmark(Landmarks::Landmark *lm)
{
  _lm = lm;
}

IsSameLandmark::~IsSameLandmark()
{
}

bool	IsSameLandmark::Matches(const Landmarks::Landmark *lm) const
{
  return (lm->pos.x == _lm->pos.x && lm->pos.y == _lm->pos.y && lm->life == _lm->life &&
	  lm->id == _lm->id && lm->totalTimeObserved == _lm->totalTimeObserved && lm->bearing == _lm->bearing &&
	  lm->range == _lm->range && lm->a == _lm->a && lm->b == _lm->b);
}

std::ostream& operator<<(std::ostream& stm, __attribute__ ((unused))const IsSameLandmark&  cp)
{
  stm << "The same landmark";
  return (stm);
}
