#ifndef _LOOP_CLOSURE_CHECK_H_
#define _LOOP_CLOSURE_CHECK_H_

#include "g2o/core/optimizable_graph.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/edge_se2.h"

#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/edge_se3.h"

using namespace g2o;
using namespace std;

class LoopClosureChecker{
public:
  typedef std::map<OptimizableGraph::Edge*, double> EdgeDoubleMap;
  LoopClosureChecker();
  void init(OptimizableGraph::VertexIDMap& movableRegion, OptimizableGraph::EdgeSet& closingEdges, double inlierThreshold);
  inline int inliers() const {return _bestInliers;}
  inline EdgeDoubleMap& closures() {return _bestResult;}
  inline double chi2() const {return _bestChi2;}
  
  void check(const std::string& matchingType="3dPose");
protected:
  // this is gonna be unelegant
  void checkAndUpdateBest(OptimizableGraph::EdgeSet& eset, const std::string& matchingType="3dPose");

  void applyZeroErrorTransform(OptimizableGraph::EdgeSet& eset, 
			       const std::string& matchingType="3dPose");

  EdgeDoubleMap _bestResult;
  double        _bestChi2    = 0.0;
  int           _bestInliers = 0;

  EdgeDoubleMap _tempResult;
  //EdgeDoubleMap _tempChi2;
  OptimizableGraph::VertexIDMap _localVertices;
  OptimizableGraph::EdgeSet   _closuresToCheck;
  double _inlierThreshold = 0.0;
};

#endif
