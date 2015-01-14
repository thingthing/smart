#ifndef		_CUSTOM_CONSTRAINT_HH_
# define	_CUSTOM_CONSTRAINT_HH_

#include <iostream>
#include "Landmarks.hh"

/**
 * Use to compare two landmark
 **/
class	IsSameLandmark
{
public:
  IsSameLandmark(Landmarks::Landmark *lm);
  ~IsSameLandmark();

  bool	Matches(const Landmarks::Landmark *lm) const;

  friend std::ostream& operator<<(std::ostream& stm, const IsSameLandmark& );
protected:
  IsSameLandmark();

private:
  Landmarks::Landmark	*_lm;
};

std::ostream& operator<<(std::ostream& stm, const IsSameLandmark& cp);


#endif		/*! _CUSTOM_CONSTRAINT_HH_*/
