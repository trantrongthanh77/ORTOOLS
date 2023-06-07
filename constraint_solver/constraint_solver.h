#ifndef VRP_H
#define VRP_H
#include <algorithm>
#include <cstdint>
#include <sstream>
#include <vector>

#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"

namespace constraint_solver {
struct DataModel {
  std::vector<std::vector<double>> distance_matrix;
  int num_vehicles;
  operations_research::RoutingIndexManager::NodeIndex depot;
  std::vector<int64_t> vehicle_capacities;
};

class RoutingWrapper {
public:
  RoutingWrapper();
  void InitDataModel(std::vector<std::vector<double>> distance_matrix,
                     int num_vehicles, int depotIndex);

  // getters
  DataModel getData() { return data; }
  // operations_research::RoutingIndexManager getManager() { return *manager; }
  // operations_research::RoutingModel *getRouting() { return routing.get(); }
  // operations_research::RoutingSearchParameters getSearchParameters() { return searchParameters; }
  // const operations_research::Assignment *getSolution() const { return solution; }

  void CreateRoutingIndexManager(DataModel data);
  void CreateRoutingModel();
  int RegisterTransitCallback();

  bool AddDimension(int evaluator_index, int slack_max, int capacity,
                    bool fix_start_cumul_to_zero, const std::string &name);
  bool AddDimensionWithVehicleCapacity(int evaluator_index, int64_t slack_max,
                                       std::vector<int64_t> vehicle_capacities,
                                       bool fix_start_cumul_to_zero,
                                       const std::string &name);
  void CreateDefaultRoutingSearchParameters();
  void SetFirstSolutionStrategy(std::string strategy);
  void SolveWithCurrentParameters();
  void PrintSolution();

private:
  std::unique_ptr<operations_research::RoutingIndexManager> manager;
  std::unique_ptr<operations_research::RoutingModel> routing;
  DataModel data;
  operations_research::RoutingSearchParameters searchParameters;
  operations_research::FirstSolutionStrategy_Value firstSolutionStrategy;
  const operations_research::Assignment *solution;
  // Solver solver;
};
} // namespace constraint_solver

#endif
