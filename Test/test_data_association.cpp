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
