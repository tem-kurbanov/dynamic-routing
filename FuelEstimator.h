//
// Created by Tim on 23.12.2024.
//

#ifndef FUELESTIMATOR_H
#define FUELESTIMATOR_H
#include <xgboost/c_api.h>

#include <string>
#include <vector>

#include "Edge.h"

class FuelEstimator {
   private:
    BoosterHandle mBooster;

    int mNumFeatures = 5;

   public:
    explicit FuelEstimator(const std::string& pathToModel);

    double predictFuel(const Edge& edge, double density, double speed);

    // Batch prediction for multiple edges
    std::vector<double> predictFuelBatch(std::vector<const Edge*>& edges, std::vector<double>& densities,
                                         std::vector<double>& speeds);
};

#endif  // FUELESTIMATOR_H
