#pragma once
#include "stereouv_aligner.h"
#include "xyz_aligner.h"

namespace proslam {

class AlignerFactory {
  public: EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //ds object handling
    public:

      //ds instantiation fully blocked
      AlignerFactory()  = delete;
      ~AlignerFactory() = delete;

    //ds factory methods
    public:

      //ds allocate aligner by enum
      static BaseAligner6_3* create(const AlignerType6_3& aligner_type_) {

        //ds switch through available products
        switch(aligner_type_) {

          //ds classical 3d icp
          case AlignerType6_3::xyz: {

            //ds allocate a new aligner
            BaseAligner6_3* aligner = new XYZAligner();

            //ds bookkeep construction for proper memory handling
            _aligners6_3.push_back(aligner);

            //ds return to user
            std::cerr << "AlignerFactory::create|deploying aligner of type: XYZ (convergence delta: " << aligner->errorDeltaForConvergence() << " kernel cutoff error: " << aligner->maximumErrorKernel() << ")" << std::endl;
            return aligner;
          }

          //ds unknown
          default: {

            //ds requested target could not be produced
            return 0;
          }
        }
      }

      //ds allocate aligner by enum
      static BaseAligner6_4* create(const AlignerType6_4& aligner_type_) {

        //ds switch through available products
        switch(aligner_type_) {

          //ds stereo uv aligner (used in SVI)
          case AlignerType6_4::stereouv: {

            //ds allocate a new aligner TODO check converge/speed problems!
            BaseAligner6_4* aligner = new StereoUVAligner();

            //ds bookkeep construction for proper memory handling
            _aligners6_4.push_back(aligner);

            //ds return to user
            std::cerr << "AlignerFactory::create|deploying aligner of type: Stereo UV (convergence delta: " << aligner->errorDeltaForConvergence() << " kernel cutoff error: " << aligner->maximumErrorKernel() << ")" << std::endl;;
            return aligner;
          }

          //ds unknown
          default: {

            //ds requested target could not be produced
            return 0;
          }
        }
      }

      //ds frees memory for all products
      static void clear() {
        for (const BaseAligner6_3* aligner: _aligners6_3) {
          delete aligner;
        }
        for (const BaseAligner6_4* aligner: _aligners6_4) {
          delete aligner;
        }
        _aligners6_3.clear();
        _aligners6_4.clear();
      }

    //ds memory control
    protected:

      static std::vector<BaseAligner6_3*> _aligners6_3;
      static std::vector<BaseAligner6_4*> _aligners6_4;

  }; //class AlignerFactory
} //namespace gslam
