#pragma once
#include "stereouv_aligner.h"
#include "uvd_aligner.h"
#include "xuvd_aligner.h"
#include "xyz_aligner.h"
#include "pxyz_aligner.h"

namespace gslam {

class AlignerFactory {

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

          //ds projection aligner
          case AlignerType6_3::uvd: {

            //ds allocate a new aligner
            BaseAligner6_3* aligner = new UVDAligner();

            //ds bookkeep construction for proper memory handling
            _aligners6_3.push_back(aligner);

            //ds return to user
            std::cerr << "AlignerFactory::create|deploying aligner of type: UVD (convergence delta: " << aligner->errorDeltaForConvergence() << " kernel cutoff error: " << aligner->maximumErrorKernel() << ")" << std::endl;
            return aligner;
          }

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

          //ds mocked uvd aligner running on odometry
          case AlignerType6_3::xuvd: {

            //ds allocate a new aligner
            BaseAligner6_3* aligner = new XUVDAligner();

            //ds bookkeep construction for proper memory handling
            _aligners6_3.push_back(aligner);

            //ds return to user
            std::cerr << "AlignerFactory::create|deploying aligner of type: Mocked UVD (convergence delta: " << aligner->errorDeltaForConvergence() << " kernel cutoff error: " << aligner->maximumErrorKernel() << ")" << std::endl;
            return aligner;
          }

          //ds far 3d icp
          case AlignerType6_3::pxyz: {

            //ds allocate a new aligner
            BaseAligner6_3* aligner = new PXYZAligner();

            //ds bookkeep construction for proper memory handling
            _aligners6_3.push_back(aligner);

            //ds return to user
            std::cerr << "AlignerFactory::create|deploying aligner of type: PXYZ (convergence delta: " << aligner->errorDeltaForConvergence() << " kernel cutoff error: " << aligner->maximumErrorKernel() << ")" << std::endl;
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
