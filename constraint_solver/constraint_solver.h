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
  std::vector<std::vector<int>> pickups_deliveries;
};

class RoutingWrapper {
public:
  RoutingWrapper();
  void InitDataModel(std::vector<std::vector<double>> distance_matrix,
                     int num_vehicles, int depotIndex);

  // getters
  DataModel getData() { return data; }

  void CreateRoutingIndexManager(DataModel data);
  void CreateRoutingModel();

  // Add constraint data
  void AddPickupsAndDeliveris(std::vector<std::vector<int>> pickups_deliveries);
  void AddVehicleCapacities(std::vector<int> vehicle_capacities);

  int RegisterTransitCallback();
  int RegisterDemandCallback(std::vector<int> demands);

  bool AddDimension(int evaluator_index, int slack_max, int capacity,
                    bool fix_start_cumul_to_zero, const std::string &name);
  bool AddDimensionWithVehicleCapacity(int evaluator_index, int slack_max,
                                       bool fix_start_cumul_to_zero,
                                       std::string name);

  void SetGlobalSpanCostCoefficient(std::string dimension_name, int coefficient);
  void AddPickupAndDeliveryConstraint(std::string dimension_name);

  void CreateDefaultRoutingSearchParameters();
  void SetFirstSolutionStrategy(std::string strategy);
  void SetLocalSearchMetaheuristic(std::string metaheuristic);
  void SetMutableTimeLimit(int seconds);

  void SolveWithCurrentParameters();

  void PrintSolution();

private:
  std::unique_ptr<operations_research::RoutingIndexManager> manager;
  std::unique_ptr<operations_research::RoutingModel> routing;
  DataModel data;
  operations_research::RoutingSearchParameters search_parameters;
  const operations_research::Assignment *solution;
  // Solver solver;
};
} // namespace constraint_solver

#endif
