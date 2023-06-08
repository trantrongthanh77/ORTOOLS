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

// first solution strategies
std::unordered_map<std::string,
                   operations_research::FirstSolutionStrategy_Value>
    STRATEGY_MAP = {
        {"AUTOMATIC", operations_research::FirstSolutionStrategy::AUTOMATIC},
        {"PATH_CHEAPEST_ARC",
         operations_research::FirstSolutionStrategy::PATH_CHEAPEST_ARC},
        {"PATH_MOST_CONSTRAINED_ARC",
         operations_research::FirstSolutionStrategy::PATH_MOST_CONSTRAINED_ARC},
        {"EVALUATOR_STRATEGY",
         operations_research::FirstSolutionStrategy::EVALUATOR_STRATEGY},
        {"SAVINGS", operations_research::FirstSolutionStrategy::SAVINGS},
        {"SWEEP", operations_research::FirstSolutionStrategy::SWEEP},
        {"CHRISTOFIDES",
         operations_research::FirstSolutionStrategy::CHRISTOFIDES},
        {"ALL_UNPERFORMED",
         operations_research::FirstSolutionStrategy::ALL_UNPERFORMED},
        {"BEST_INSERTION",
         operations_research::FirstSolutionStrategy::BEST_INSERTION},
        {"PARALLEL_CHEAPEST_INSERTION",
         operations_research::FirstSolutionStrategy::
             PARALLEL_CHEAPEST_INSERTION},
        {"SEQUENTIAL_CHEAPEST_INSERTION",
         operations_research::FirstSolutionStrategy::
             SEQUENTIAL_CHEAPEST_INSERTION},
        {"LOCAL_CHEAPEST_INSERTION",
         operations_research::FirstSolutionStrategy::LOCAL_CHEAPEST_INSERTION},
        {"LOCAL_CHEAPEST_COST_INSERTION",
         operations_research::FirstSolutionStrategy::
             LOCAL_CHEAPEST_COST_INSERTION},
        {"GLOBAL_CHEAPEST_ARC",
         operations_research::FirstSolutionStrategy::GLOBAL_CHEAPEST_ARC},
        {"LOCAL_CHEAPEST_ARC",
         operations_research::FirstSolutionStrategy::LOCAL_CHEAPEST_ARC},
        {"FIRST_UNBOUND_MIN_VALUE",
         operations_research::FirstSolutionStrategy::FIRST_UNBOUND_MIN_VALUE}};

// local search metaheuristics
std::unordered_map<std::string,
                   operations_research::LocalSearchMetaheuristic_Value>
    LOCAL_SEARCH_METAHEURISTIC_MAP = {
        {"AUTOMATIC", operations_research::LocalSearchMetaheuristic::AUTOMATIC},
        {"GREEDY_DESCENT",
         operations_research::LocalSearchMetaheuristic::GREEDY_DESCENT},
        {"GUIDED_LOCAL_SEARCH",
         operations_research::LocalSearchMetaheuristic::GUIDED_LOCAL_SEARCH},
        {"SIMULATED_ANNEALING",
         operations_research::LocalSearchMetaheuristic::SIMULATED_ANNEALING},
        {"TABU_SEARCH",
         operations_research::LocalSearchMetaheuristic::TABU_SEARCH},
        {"GENERIC_TABU_SEARCH",
         operations_research::LocalSearchMetaheuristic::GENERIC_TABU_SEARCH}};

RoutingWrapper::RoutingWrapper() {}

void RoutingWrapper::InitDataModel(
    std::vector<std::vector<double>> distance_matrix, int num_vehicles,
    int depot_index) {
  data.distance_matrix = distance_matrix;
  data.num_vehicles = num_vehicles;
  operations_research::RoutingIndexManager::NodeIndex depot(depot_index);
  data.depot = depot;
}

void RoutingWrapper::AddPickupsAndDeliveris(
    std::vector<std::vector<int>> pickups_deliveries) {
  data.pickups_deliveries = pickups_deliveries;
}

void RoutingWrapper::AddVehicleCapacities(std::vector<int> vehicle_capacities) {
  data.vehicle_capacities = std::vector<int64_t>(vehicle_capacities.begin(),
                                                 vehicle_capacities.end());
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

int RoutingWrapper::RegisterDemandCallback(std::vector<int> demands) {
  const int demand_callback_index = routing->RegisterUnaryTransitCallback(
      [demands, manager = manager.get()](int64_t from_index) -> int64_t {
        // Convert from routing variable Index to demand NodeIndex.
        int from_node = manager->IndexToNode(from_index).value();
        return demands[from_node];
      });
  return demand_callback_index;
}

bool RoutingWrapper::AddDimension(int evaluator_index, int slack_max,
                                  int capacity, bool fix_start_cumul_to_zero,
                                  const std::string &name) {
  return routing->AddDimension(evaluator_index, slack_max, capacity,
                               fix_start_cumul_to_zero, name);
}

bool RoutingWrapper::AddDimensionWithVehicleCapacity(
    int evaluator_index, int slack_max, bool fix_start_cumul_to_zero,
    std::string name) {
  return routing->AddDimensionWithVehicleCapacity(
      evaluator_index, slack_max, data.vehicle_capacities,
      fix_start_cumul_to_zero, name);
}

void RoutingWrapper::SetGlobalSpanCostCoefficient(std::string dimension_name,
                                                  int coefficient) {
  operations_research::RoutingDimension *dimension =
      routing->GetMutableDimension(dimension_name);
  dimension->SetGlobalSpanCostCoefficient(coefficient);
}

void RoutingWrapper::AddPickupAndDeliveryConstraint(
    std::string dimension_name) {
  operations_research::Solver *const solver = routing->solver();
  operations_research::RoutingDimension *distance_dimension =
      routing->GetMutableDimension(dimension_name);
  for (const auto &request : data.pickups_deliveries) {
    int64_t pickup_index = request[0];
    int64_t delivery_index = request[1];

    routing->AddPickupAndDelivery(pickup_index, delivery_index);
    solver->AddConstraint(
        solver->MakeEquality(routing->VehicleVar(pickup_index),
                             routing->VehicleVar(delivery_index)));
    solver->AddConstraint(
        solver->MakeLessOrEqual(distance_dimension->CumulVar(pickup_index),
                                distance_dimension->CumulVar(delivery_index)));
  }
}

void RoutingWrapper::CreateDefaultRoutingSearchParameters() {
  search_parameters = operations_research::DefaultRoutingSearchParameters();
}

void RoutingWrapper::SetFirstSolutionStrategy(std::string strategy) {
  auto it = STRATEGY_MAP.find(strategy);
  if (it != STRATEGY_MAP.end()) {
    search_parameters.set_first_solution_strategy(it->second);
  }
}

void RoutingWrapper::SetLocalSearchMetaheuristic(std::string metaheuristic) {
  auto it = LOCAL_SEARCH_METAHEURISTIC_MAP.find(metaheuristic);
  if (it != LOCAL_SEARCH_METAHEURISTIC_MAP.end()) {
    search_parameters.set_local_search_metaheuristic(it->second);
  }
}

void RoutingWrapper::SetMutableTimeLimit(int seconds) {
  search_parameters.mutable_time_limit()->set_seconds(seconds);
}

void RoutingWrapper::SolveWithCurrentParameters() {
  solution = routing->SolveWithParameters(search_parameters);
}

} // namespace constraint_solver

// int main(int /*argc*/, char * /*argv*/[]) {
//   constraint_solver::VrpGlobalSpan();
//   return EXIT_SUCCESS;
// }
