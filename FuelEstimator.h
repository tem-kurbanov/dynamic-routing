//
// Created by Tim on 23.12.2024.
//

#ifndef FUELESTIMATOR_H
#define FUELESTIMATOR_H
#include "EstimatorSupport.h"

#include <string>
#include <vector>

#include "Edge.h"

/**
 * @class FuelEstimator
 * @brief Predicts fuel consumption on edges using an XGBoost model.
 */
class FuelEstimator {
   private:
    estimator::Booster mBooster;  ///< XGBoost booster handle

    int mNumFeatures = 5;  ///< Number of features expected by the model

   public:
    /** @brief Construct estimator by loading a model from path. */
    explicit FuelEstimator(const std::string& pathToModel);

    /** @brief Predict fuel consumption for an edge at given density and speed. */
    double predictFuel(const Edge& edge, double density, double speed);

    /** @brief Batch prediction for multiple edges. */
    std::vector<double> predictFuelBatch(std::vector<const Edge*>& edges, std::vector<double>& densities,
                                         std::vector<double>& speeds);
};

#endif  // FUELESTIMATOR_H
