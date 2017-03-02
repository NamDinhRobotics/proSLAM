#ifndef _CLOSURE_BUFFER_H_
#define _CLOSURE_BUFFER_H_

#include <list>

#include "g2o/core/optimizable_graph.h"


using namespace g2o;

struct VertexTime{
  int time;
  OptimizableGraph::Vertex* v;
};

struct VertexTimeComparator
{
  OptimizableGraph::Vertex* v;

VertexTimeComparator(OptimizableGraph::Vertex* v_)
: v(v_) {}
 
  bool operator()(const VertexTime& vt)
  {
    return vt.v->id() == v->id();
  }
};

struct ClosureBuffer {
  void addEdge(OptimizableGraph::Edge *e);
  void removeEdge(OptimizableGraph::Edge *e);

  void addEdgeSet(OptimizableGraph::EdgeSet& eset);
  void removeEdgeSet(OptimizableGraph::EdgeSet& eset);

  void addVertex(OptimizableGraph::Vertex *v);
  void removeVertex(OptimizableGraph::Vertex *v);

  OptimizableGraph::Vertex* findVertex(int idVertex);

  /* inline void setRobotId(int rid) { _robotId = rid;} */
  /* inline int robotId() const { return _robotId;} */

  inline OptimizableGraph::EdgeSet& edgeSet() {return _eset;}
  inline OptimizableGraph::VertexIDMap& vertices() {return _vmap;}

  inline std::list<VertexTime>& vertexList() {return _vertexList;}

  void updateList(int windowSize);
  bool checkList(int windowSize);

protected:
  std::list<VertexTime> _vertexList;
  OptimizableGraph::EdgeSet _eset;
  OptimizableGraph::VertexIDMap _vmap;
  //int _robotId;
};

#endif
