#include <iostream>
#include "map_optimization/graph_optimizer.h"

G2O_USE_TYPE_GROUP(slam3d);

int32_t main (int32_t argc_, char** argv_) {

  std::cerr << "allocating proslam::GraphOptimizerParameters" << std::endl;
  proslam::GraphOptimizerParameters* parameters = new proslam::GraphOptimizerParameters(proslam::LoggingLevel::Debug);
  std::cerr << "allocated" << std::endl;

  std::cerr << "allocating proslam::GraphOptimizer" << std::endl;
  proslam::GraphOptimizer* graph_optimizer = new proslam::GraphOptimizer(parameters);
  std::cerr << "allocated" << std::endl;

  std::cerr << "configuring" << std::endl;
  graph_optimizer->configure();
  std::cerr << "configured" << std::endl;

  std::cerr << "deallocating proslam::GraphOptimizer" << std::endl;
  delete graph_optimizer;
  std::cerr << "deallocated" << std::endl;

  return 0;
}
