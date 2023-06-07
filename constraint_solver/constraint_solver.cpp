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

void RoutingWrapper::PrintSolution() {
  int64_t total_distance = 0;
  for (int vehicle_id = 0; vehicle_id < data.num_vehicles; ++vehicle_id) {
    int64_t index = routing->Start(vehicle_id);
    std::cout << "Route #" << vehicle_id << ":";
    int64_t route_distance = 0;
    std::stringstream route;
    while (routing->IsEnd(index) == false) {
      route << manager.get()->IndexToNode(index).value() << " ";
      int64_t previous_index = index;
      index = solution->Value(routing->NextVar(index));
      route_distance +=
          routing->GetArcCostForVehicle(previous_index, index, vehicle_id);
    }
    // Kết thúc đường đi
    std::cout << route.str() << manager.get()->IndexToNode(index).value() << "\n";
    total_distance += route_distance;
  }
  std::cout << "Tổng khoảng cách của tất cả các đường đi: " << total_distance
            << "m\n";
}

void VrpGlobalSpan() {
  // init RoutingWrapper
  RoutingWrapper routingWrapper;
  const std::vector<std::vector<double>> distance_matrix{
      {0, 548, 776, 696, 582, 274, 502, 194, 308, 194, 536, 502, 388, 354, 468,
       776, 662},
      {548, 0, 684, 308, 194, 502, 730, 354, 696, 742, 1084, 594, 480, 674,
       1016, 868, 1210},
      {776, 684, 0, 992, 878, 502, 274, 810, 468, 742, 400, 1278, 1164, 1130,
       788, 1552, 754},
      {696, 308, 992, 0, 114, 650, 878, 502, 844, 890, 1232, 514, 628, 822,
       1164, 560, 1358},
      {582, 194, 878, 114, 0, 536, 764, 388, 730, 776, 1118, 400, 514, 708,
       1050, 674, 1244},
      {274, 502, 502, 650, 536, 0, 228, 308, 194, 240, 582, 776, 662, 628, 514,
       1050, 708},
      {502, 730, 274, 878, 764, 228, 0, 536, 194, 468, 354, 1004, 890, 856, 514,
       1278, 480},
      {194, 354, 810, 502, 388, 308, 536, 0, 342, 388, 730, 468, 354, 320, 662,
       742, 856},
      {308, 696, 468, 844, 730, 194, 194, 342, 0, 274, 388, 810, 696, 662, 320,
       1084, 514},
      {194, 742, 742, 890, 776, 240, 468, 388, 274, 0, 342, 536, 422, 388, 274,
       810, 468},
      {536, 1084, 400, 1232, 1118, 582, 354, 730, 388, 342, 0, 878, 764, 730,
       388, 1152, 354},
      {502, 594, 1278, 514, 400, 776, 1004, 468, 810, 536, 878, 0, 114, 308,
       650, 274, 844},
      {388, 480, 1164, 628, 514, 662, 890, 354, 696, 422, 764, 114, 0, 194, 536,
       388, 730},
      {354, 674, 1130, 822, 708, 628, 856, 320, 662, 388, 730, 308, 194, 0, 342,
       422, 536},
      {468, 1016, 788, 1164, 1050, 514, 514, 662, 320, 274, 388, 650, 536, 342,
       0, 764, 194},
      {776, 868, 1552, 560, 674, 1050, 1278, 742, 1084, 810, 1152, 274, 388,
       422, 764, 0, 798},
      {662, 1210, 754, 1358, 1244, 708, 480, 856, 514, 468, 354, 844, 730, 536,
       194, 798, 0},
  };
  routingWrapper.InitDataModel(distance_matrix, 10, 0);
  routingWrapper.CreateRoutingIndexManager(routingWrapper.getData());
  routingWrapper.CreateRoutingModel();
  int transit_callback_index = routingWrapper.RegisterTransitCallback();
  routingWrapper.AddDimension(transit_callback_index, 0, 3000, true,
                              "Distance");
  routingWrapper.CreateDefaultRoutingSearchParameters();
  routingWrapper.SetFirstSolutionStrategy("AUTOMATIC");
  // Solve the problem.
  routingWrapper.SolveWithCurrentParameters();

  // // Print solution on console.
  routingWrapper.PrintSolution();

  // PrintSolution(routingWrapper.getData(), routingWrapper.getManager(),
  //               routingWrapper.getRouting(), *routingWrapper.getSolution());
}

} // namespace constraint_solver

// int main(int /*argc*/, char * /*argv*/[]) {
//   constraint_solver::VrpGlobalSpan();
//   return EXIT_SUCCESS;
// }
