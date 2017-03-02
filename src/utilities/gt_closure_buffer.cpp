#include "gt_closure_buffer.h"

void ClosureBuffer::addEdge(OptimizableGraph::Edge *e){
    _eset.insert(e);
}

void ClosureBuffer::removeEdge(OptimizableGraph::Edge *e){
  OptimizableGraph::EdgeSet::iterator it = _eset.find(e);
  if (it != _eset.end())
    _eset.erase(it);
}

void ClosureBuffer::addEdgeSet(OptimizableGraph::EdgeSet& eset){
  for (OptimizableGraph::EdgeSet::iterator it=eset.begin(); it!=eset.end(); ++it){
    OptimizableGraph::Edge *e=(OptimizableGraph::Edge*)(*it);
    addEdge(e);
  }
}
  
void ClosureBuffer::removeEdgeSet(OptimizableGraph::EdgeSet& eset){
  for (OptimizableGraph::EdgeSet::iterator it=eset.begin(); it!=eset.end(); ++it){
    OptimizableGraph::Edge *e=(OptimizableGraph::Edge*)(*it);
    removeEdge(e);
  }
}

void ClosureBuffer::addVertex(OptimizableGraph::Vertex *v){
  _vmap.insert(std::make_pair(v->id(),v));
  VertexTime vt;
  vt.time = 0;
  vt.v = v;
  _vertexList.push_back(vt);
}

void ClosureBuffer::removeVertex(OptimizableGraph::Vertex *v){
  OptimizableGraph::VertexIDMap::iterator it=_vmap.find(v->id());
  if (it != _vmap.end()){
    _vmap.erase(it);
    
    OptimizableGraph::EdgeSet tmp = _eset;
    for (OptimizableGraph::EdgeSet::iterator it=tmp.begin(); it!=tmp.end(); ++it){
      OptimizableGraph::Edge *e=(OptimizableGraph::Edge*)(*it);
      
      for (size_t i=0; i<e->vertices().size(); i++){
	OptimizableGraph::Vertex* vedge=(OptimizableGraph::Vertex*)e->vertices()[i];
	if (vedge->id() == v->id())
	  removeEdge(e);
      }
    }
    _vertexList.remove_if(VertexTimeComparator(v));
  }
}

OptimizableGraph::Vertex* ClosureBuffer::findVertex(int idVertex){
  OptimizableGraph::VertexIDMap::iterator it=_vmap.find(idVertex);
  if (it == _vmap.end())
    return 0;

  OptimizableGraph::Vertex* v = dynamic_cast<OptimizableGraph::Vertex*>(it->second);
  return v;
}


void ClosureBuffer::updateList(int windowSize){
  for(std::list<VertexTime>::iterator it = _vertexList.begin(); it!= _vertexList.end(); it++){
    (*it).time++;
  }
     
  std::list<VertexTime> tmp(_vertexList);
  for(std::list<VertexTime>::iterator it = tmp.begin(); it!= tmp.end(); it++){
    if ((*it).time >= windowSize) 
      removeVertex((*it).v);
  }
}

bool ClosureBuffer::checkList(int windowSize){
  for(std::list<VertexTime>::iterator it = _vertexList.begin(); it!= _vertexList.end(); it++){
    if ((*it).time == (windowSize-1))
      return true;
  }
  return false;  
}
