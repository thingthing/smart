#include "Landmarks.hh"

Landmarks::Landmark::Landmark()
{
  pos[0] = 0;
  pos[1] = 0;
  id = -1;
  life = LIFE;
  totalTimeObserved = 0;
  range = -1;
  bearing = -1;
}

Landmarks::Landmark::~Landmark()
{}
