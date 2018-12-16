#pragma once
#include "types/frame.h"
#include "base_aligner.h"

namespace proslam {

//ds implements an interface for frame-to-frame aligner to be used inside the tracker
class BaseFrameAligner: public BaseAligner {

//ds object handling
public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  BaseFrameAligner(AlignerParameters* parameters_): BaseAligner(parameters_) {}
  virtual ~BaseFrameAligner() {};

//ds functionality
public:

  //ds pure virtual
  virtual void initialize(const Frame* frame_previous_,
                          const Frame* frame_current_,
                          const TransformMatrix3D& previous_to_current_) = 0;

//ds setters/getters
public:

  inline const TransformMatrix3D& previousToCurrent() const {return _previous_to_current;}

//ds attributes
protected:

  //ds alignment context
  const Frame* _frame_current  = 0;
  const Frame* _frame_previous = 0;

  //ds objective
  TransformMatrix3D _previous_to_current = TransformMatrix3D::Identity();

  Count _number_of_rows_image = 0;
  Count _number_of_cols_image = 0;
};

typedef std::shared_ptr<BaseFrameAligner> BaseFrameAlignerPtr;

}
