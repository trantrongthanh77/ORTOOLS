package main

import (
	"fmt"
	"math"
	constraintSolver "vrp/constraint_solver"
)

func main() {
	fmt.Println("Program started")
	routingWrapper := constraintSolver.NewRoutingWrapper()

	// prepare data
	x := []float64{769, 91, 108, 816, 725, 117, 766, 90, 255, 63, 35, 122, 85, 267, 937, 865, 863, 724, 148, 243, 871, 782, 77, 776, 166, 69, 972, 320, 91, 20, 81, 776, 830, 148, 160, 741, 877, 587, 808, 26, 58, 755, 817, 165, 216, 124, 96, 132, 163, 915, 753, 161, 37, 898, 770, 745, 695, 124, 11, 258, 72, 713, 866, 214, 213, 864, 98, 701, 884, 842, 125, 352, 206, 825, 93, 863, 98, 115, 71, 96, 65, 75, 140, 129, 129, 846, 109, 863, 139, 77, 154, 231, 115, 844, 98, 739, 179, 862, 764, 844, 174, 703, 151, 863, 72, 632, 823, 26, 47, 71, 754, 137, 80, 172, 28, 754, 531, 87, 866, 93, 942, 788, 824, 926, 182, 98, 104, 48, 952, 239, 138, 747, 136, 748, 43, 792, 95, 109, 67, 104, 817, 183, 922, 779, 863, 40, 66, 862, 383, 307, 233, 329, 623, 103, 736, 75, 174}
	y := []float64{259, 377, 338, 567, 520, 357, 559, 417, 430, 384, 276, 318, 322, 348, 546, 461, 547, 431, 144, 571, 512, 596, 272, 650, 371, 239, 607, 521, 434, 305, 271, 669, 553, 243, 202, 527, 561, 551, 435, 278, 394, 444, 557, 367, 510, 330, 587, 369, 234, 492, 657, 319, 227, 747, 524, 582, 469, 355, 283, 406, 335, 399, 626, 344, 349, 577, 440, 512, 559, 518, 379, 410, 228, 592, 252, 511, 347, 326, 392, 248, 404, 342, 300, 340, 273, 696, 463, 586, 356, 351, 322, 323, 394, 558, 454, 568, 443, 618, 764, 554, 319, 319, 453, 569, 473, 575, 578, 42, 381, 336, 484, 465, 355, 296, 310, 579, 662, 382, 481, 337, 567, 567, 451, 901, 500, 384, 370, 462, 629, 324, 516, 605, 460, 624, 268, 597, 352, 413, 224, 386, 672, 360, 652, 419, 721, 456, 395, 465, 357, 434, 421, 280, 554, 349, 688, 266, 406}

	dist := calculateDistanceMatrix(x, y)

	// init pickup and delivery (optional)
	pickupsDeliveries := [][]int{
		{1, 3},
		{2, 10},
	}
	pdVector := constraintSolver.NewPairIntVector()
	for i := 0; i < len(pickupsDeliveries); i++ {
		pair := constraintSolver.NewIntVector()
		for j := 0; j < len(pickupsDeliveries[i]); j++ {
			pair.Add(pickupsDeliveries[i][j])
		}
		pdVector.Add(pair)
	}

	// add vehicle capacity (optional)
	vehicleCapacities := []int{50, 50, 50, 50, 50}
	capacityVector := constraintSolver.NewIntVector()

	for i := 0; i < len(vehicleCapacities); i++ {
		capacityVector.Add(vehicleCapacities[i])
	}
	routingWrapper.AddVehicleCapacities(capacityVector)

	// add vehicle demand (require if vehicle capacity is set)
	demands := []int{0, 45, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
	demandVector := constraintSolver.NewIntVector()
	for i := 0; i < len(demands); i++ {
		demandVector.Add(demands[i])
	}

	// init data and model
	routingWrapper.InitDataModel(dist, 5, 0)
	routingWrapper.AddPickupsAndDeliveris(pdVector)
	routingWrapper.CreateRoutingIndexManager(routingWrapper.GetData())
	routingWrapper.CreateRoutingModel()

	// dimension constraint
	transitCallbackIndex := routingWrapper.RegisterTransitCallback()
	routingWrapper.AddDimension(transitCallbackIndex, 0, 3000, true,
		"Distance")
	routingWrapper.SetGlobalSpanCostCoefficient("Distance", 100)

	// Add capacity constraint (require if vehicle capacity is set)
	demandCallBackIndex := routingWrapper.RegisterDemandCallback(demandVector)
	routingWrapper.AddDimensionWithVehicleCapacity(demandCallBackIndex, 0, true,
		"Capacity")

	// Add pickup and delivery constraints. (require if pickup and delivery is set)
	routingWrapper.AddPickupAndDeliveryConstraint("Distance")

	// Setting search strategy
	routingWrapper.CreateDefaultRoutingSearchParameters()
	routingWrapper.SetFirstSolutionStrategy("AUTOMATIC")
	// routingWrapper.SetLocalSearchMetaheuristic("GUIDED_LOCAL_SEARCH") // optional
	// routingWrapper.SetMutableTimeLimit(5)                             // optional

	// Solve the problem.
	routingWrapper.SolveWithCurrentParameters()
	routingWrapper.PrintSolution()
}

func calculateDistanceMatrix(x []float64, y []float64) constraintSolver.DistanceMatrix {
	// calculate distance matrix
	dist := constraintSolver.NewDistanceMatrix()
	for i := 0; i < len(x); i++ {
		temp := constraintSolver.NewDoubleVector()
		for j := 0; j < len(x); j++ {
			temp.Add(math.Sqrt(math.Pow(x[i]-x[j], 2) + math.Pow(y[i]-y[j], 2)))
		}
		dist.Add(temp)
	}
	return dist
}
