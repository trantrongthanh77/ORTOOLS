#include "constraint_solver.h"
#include <algorithm>
#include <cstdint>
#include <memory>
#include <sstream>
#include <vector>

#include "InstanceCVRPLIB.h"
#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"

namespace constraint_solver {

RoutingWrapper::RoutingWrapper() {}

void RoutingWrapper::InitDataModel(
    std::vector<std::vector<double>> distance_matrix, int num_vehicles,
    int depotIndex) {
  data.distance_matrix = distance_matrix;
  data.num_vehicles = num_vehicles;
  operations_research::RoutingIndexManager::NodeIndex depot(depotIndex);
  data.depot = depot;
  // data.vehicle_capacities = std::vector<int64_t>(num_vehicles, 36);
}

void RoutingWrapper::CreateRoutingIndexManager(DataModel data) {
  manager = std::make_unique<operations_research::RoutingIndexManager>(
      data.distance_matrix.size(), data.num_vehicles, data.depot);
}

void RoutingWrapper::CreateRoutingModel() {
  routing = std::make_unique<operations_research::RoutingModel>(*manager);
}

int RoutingWrapper::RegisterTransitCallback() {
  // Define cost of each arc.
  const int transit_callback_index = routing->RegisterTransitCallback(
      [data = &this->data, manager = manager.get()](
          int64_t from_index, int64_t to_index) -> int64_t {
        // Convert from routing variable Index to distance matrix NodeIndex.
        auto from_node = manager->IndexToNode(from_index).value();
        auto to_node = manager->IndexToNode(to_index).value();
        return data->distance_matrix[from_node][to_node];
      });
  routing->SetArcCostEvaluatorOfAllVehicles(transit_callback_index);
  return transit_callback_index;
}

bool RoutingWrapper::AddDimension(int evaluator_index, int slack_max,
                                  int capacity, bool fix_start_cumul_to_zero,
                                  const std::string &name) {
  return routing->AddDimension(evaluator_index, slack_max, capacity,
                               fix_start_cumul_to_zero, name);
}

bool RoutingWrapper::AddDimensionWithVehicleCapacity(
    int evaluator_index, int64_t slack_max,
    std::vector<int64_t> vehicle_capacities, bool fix_start_cumul_to_zero,
    const std::string &name) {
  return routing->AddDimensionWithVehicleCapacity(
      evaluator_index, slack_max, vehicle_capacities, fix_start_cumul_to_zero,
      name);
}

void RoutingWrapper::CreateDefaultRoutingSearchParameters() {
  searchParameters = operations_research::DefaultRoutingSearchParameters();
}

void RoutingWrapper::SetFirstSolutionStrategy(std::string strategy) {
  std::unordered_map<std::string, operations_research::FirstSolutionStrategy_Value> strategyMap = {
      {"AUTOMATIC", operations_research::FirstSolutionStrategy::AUTOMATIC},
      {"PATH_CHEAPEST_ARC", operations_research::FirstSolutionStrategy::PATH_CHEAPEST_ARC},
      {"PATH_MOST_CONSTRAINED_ARC",
       operations_research::FirstSolutionStrategy::PATH_MOST_CONSTRAINED_ARC},
      {"EVALUATOR_STRATEGY", operations_research::FirstSolutionStrategy::EVALUATOR_STRATEGY},
      {"SAVINGS", operations_research::FirstSolutionStrategy::SAVINGS},
      {"SWEEP", operations_research::FirstSolutionStrategy::SWEEP},
      {"CHRISTOFIDES", operations_research::FirstSolutionStrategy::CHRISTOFIDES},
      {"ALL_UNPERFORMED", operations_research::FirstSolutionStrategy::ALL_UNPERFORMED},
      {"BEST_INSERTION", operations_research::FirstSolutionStrategy::BEST_INSERTION},
      {"PARALLEL_CHEAPEST_INSERTION",
       operations_research::FirstSolutionStrategy::PARALLEL_CHEAPEST_INSERTION},
      {"SEQUENTIAL_CHEAPEST_INSERTION",
       operations_research::FirstSolutionStrategy::SEQUENTIAL_CHEAPEST_INSERTION},
      {"LOCAL_CHEAPEST_INSERTION",
       operations_research::FirstSolutionStrategy::LOCAL_CHEAPEST_INSERTION},
      {"LOCAL_CHEAPEST_COST_INSERTION",
       operations_research::FirstSolutionStrategy::LOCAL_CHEAPEST_COST_INSERTION},
      {"GLOBAL_CHEAPEST_ARC", operations_research::FirstSolutionStrategy::GLOBAL_CHEAPEST_ARC},
      {"LOCAL_CHEAPEST_ARC", operations_research::FirstSolutionStrategy::LOCAL_CHEAPEST_ARC},
      {"FIRST_UNBOUND_MIN_VALUE",
       operations_research::FirstSolutionStrategy::FIRST_UNBOUND_MIN_VALUE}};

  auto it = strategyMap.find(strategy);
  if (it != strategyMap.end()) {
    firstSolutionStrategy = it->second;
  }
}

void RoutingWrapper::SolveWithCurrentParameters() {
  solution = routing->SolveWithParameters(searchParameters);
}
} // namespace constraint_solver

// int main(int /*argc*/, char * /*argv*/[]) {
//   constraint_solver::VrpGlobalSpan();
//   return EXIT_SUCCESS;
// }
