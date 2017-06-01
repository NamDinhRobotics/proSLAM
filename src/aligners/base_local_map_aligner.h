#pragma once
#include "relocalization/local_map_correspondence.h"
#include "base_aligner.h"

namespace proslam {

  //ds class that implements an aligner interface between two local maps
  class BaseLocalMapAligner: public BaseAligner {

  //ds object handling
  public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    BaseLocalMapAligner() {}
    virtual ~BaseLocalMapAligner() {};

  //ds functionality
  public:

    //ds pure virtual
    virtual void initialize(LocalMapCorrespondence* context_, const TransformMatrix3D& current_to_reference_ = TransformMatrix3D::Identity()) = 0;

  //ds setters/getters
  public:

    inline const TransformMatrix3D& currentToReference() const {return _current_to_reference;}
    
  protected:

    //ds context
    LocalMapCorrespondence* _context = 0;

    //ds objective
    TransformMatrix3D _current_to_reference = TransformMatrix3D::Identity();
  };
}
