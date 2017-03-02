#include "gt_closure_checker.h"

LoopClosureChecker::LoopClosureChecker(){}

void LoopClosureChecker::init(OptimizableGraph::VertexIDMap& movableRegion, OptimizableGraph::EdgeSet& closingEdges, double inlierThreshold) {
  _localVertices = movableRegion;
  _closuresToCheck = closingEdges;
  _inlierThreshold = inlierThreshold;
  _bestInliers = 0;
  _bestChi2 = std::numeric_limits<double>::max();
  _tempResult.clear();
  _bestResult.clear();
  for (OptimizableGraph::EdgeSet::iterator it=closingEdges.begin(); it!=closingEdges.end(); it++){
    OptimizableGraph::Edge* e=(OptimizableGraph::Edge*)(*it);
    _tempResult.insert(make_pair(e, std::numeric_limits<double>::max()));
    _bestResult.insert(make_pair(e, std::numeric_limits<double>::max()));
  }
}
  
void LoopClosureChecker::check(const std::string& matchingType){
  if (matchingType == "2dPose" || matchingType == "3dPose") {
    for (OptimizableGraph::EdgeSet::iterator it=_closuresToCheck.begin(); it!=_closuresToCheck.end(); it++){
      OptimizableGraph::Edge* e=(OptimizableGraph::Edge*)(*it);
      OptimizableGraph::EdgeSet eset;
      eset.insert(e);
      checkAndUpdateBest(eset, matchingType);
    }
  } else {
    assert (0 && "wrong matching strategy selected");
  }
}

// this is gonna be unelegant
void LoopClosureChecker::checkAndUpdateBest(OptimizableGraph::EdgeSet& eset, const std::string& matchingType) {
  applyZeroErrorTransform(eset, matchingType);
  // count the chi2 and the inliers;
  int inliers = 0;
  double totalChi2 = 0;
  for (EdgeDoubleMap::iterator it= _tempResult.begin(); it!= _tempResult.end(); it++){
    if (it->second < _inlierThreshold) {
      inliers ++;
      totalChi2 += it->second;
    }
  }
  if (inliers > _bestInliers || (inliers == _bestInliers && totalChi2 < _bestChi2) ){
    _bestInliers = inliers;
    _bestChi2 = totalChi2;
    _bestResult = _tempResult;
  } 
}


void LoopClosureChecker::applyZeroErrorTransform(OptimizableGraph::EdgeSet& eset, const std::string& matchingType){
  if (matchingType == "2dPose")
  {
    assert( false );
  }
  else if (matchingType == "3dPose"){
    assert(eset.size()==1);
    EdgeSE3* e=dynamic_cast<EdgeSE3*>(*eset.begin());
    assert (e && "Wrong edge type for 3d pose matching");

    // determine which of the two vertices is the one that moves
    VertexSE3* root = 0;
    VertexSE3* vfrom=dynamic_cast<VertexSE3*>(e->vertices()[0]);
    VertexSE3* vto=dynamic_cast<VertexSE3*>(e->vertices()[1]);
    OptimizableGraph::VertexIDMap::iterator it = _localVertices.find(vfrom->id());
    if (it != _localVertices.end())
      root = vfrom;

    it = _localVertices.find(vto->id());
    if (it != _localVertices.end())
      root = vto;
	
    assert (root && "the loop closure does not have any vertex in the floating part of the map");
      
    // determine the transformation that would result in a zero error of the constraint
    Eigen::Isometry3d  newRootPose = Eigen::Isometry3d::Identity();
    if (root == vfrom){
      newRootPose = vto->estimate()*e->measurement().inverse();
    } else {
      newRootPose = vfrom->estimate()*e->measurement();
    }
    Eigen::Isometry3d motion = newRootPose * root->estimate().inverse();

    typedef std::map<VertexSE3*, Eigen::Isometry3d, std::less<VertexSE3*>, Eigen::aligned_allocator<std::pair<const VertexSE3*, Eigen::Isometry3d> > > VertexSE3Map;
    VertexSE3Map oldPoses;
    // now push all the vertices in the moving map and apply the transform
    for (OptimizableGraph::VertexIDMap::iterator it = _localVertices.begin(); it!=_localVertices.end(); it++){
      VertexSE3* v = (VertexSE3*)it->second;
      oldPoses[v]=v->estimate();
      v->push();
      v->setEstimate(motion*v->estimate());
    }
    for (EdgeDoubleMap::iterator it= _tempResult.begin(); it!= _tempResult.end(); it++){
      OptimizableGraph::Edge* e = (OptimizableGraph::Edge*) (it->first);
      e->computeError();
      VertexSE3* v = 0;
      VertexSE3* vfrom=dynamic_cast<VertexSE3*>(e->vertices()[0]);
      VertexSE3* vto=dynamic_cast<VertexSE3*>(e->vertices()[1]);
      if (_localVertices.count(vfrom->id()))
	v=vfrom;
      else
	v=vto;
      Eigen::Isometry3d estBefore = oldPoses[v];
      Eigen::Isometry3d delta = estBefore.inverse()*v->estimate();
      double thetaDifference = Eigen::AngleAxisd(delta.linear()).angle();
      it->second = e->chi2() + 0 * thetaDifference * thetaDifference;
    }
    for (OptimizableGraph::VertexIDMap::iterator it = _localVertices.begin(); it!=_localVertices.end(); it++){
      VertexSE3* v=(VertexSE3*)(it->second);
      v->pop();
    }
  }
}
