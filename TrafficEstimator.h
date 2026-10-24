
#ifndef HIERARCHICALSYSTEM_TRAFFICESTIMATOR_H
#define HIERARCHICALSYSTEM_TRAFFICESTIMATOR_H

#include "EstimatorSupport.h"

#include <memory>
#include <unordered_map>
#include <vector>

#include "CoverEdge.h"
#include "Edge.h"

/**
 * @class TrafficEstimator
 * @brief Predicts traffic densities on edges using an XGBoost model.
 */
class TrafficEstimator {
   private:
    estimator::Booster mBooster;  ///< XGBoost booster handle

    int mNumFeatures = 8;  ///< Number of features expected by the model

    /** @brief Flatten 2D data into a contiguous vector (row-major). */
    std::vector<double> toContiguousVector(const std::vector<std::vector<double>>& data);

    /** @brief Compute and print classification metrics for diagnostics. */
    void calculateClassificationMetrics(const std::vector<double>& actual, const std::vector<int>& predicted);

   public:
    /** @brief Construct estimator by loading a model from path. */
    explicit TrafficEstimator(const std::string& pathToModel);

    /** @brief Batch-predict densities for edges given features per edge ID. */
    std::unordered_map<int, double> predictDensities(const std::unordered_map<int, std::vector<double>>& edgeData);

    /** @brief Predict density for a single edge from flows and current density. */
    double predictDensity(const Edge& edge, std::pair<int, int> flows, double currentDensity);
};

#endif  // HIERARCHICALSYSTEM_TRAFFICESTIMATOR_H
