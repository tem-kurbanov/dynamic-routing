//
// Created by Tim on 10.12.2024.
//

#ifndef SPEEDESTIMATOR_H
#define SPEEDESTIMATOR_H

#include <xgboost/c_api.h>

#include <memory>
#include <unordered_map>
#include <vector>

#include "CoverEdge.h"
#include "Edge.h"

/**
 * @class SpeedEstimator
 * @brief Predicts traffic speed on an edge using an XGBoost model.
 */
class SpeedEstimator {
   private:
    BoosterHandle mBooster;  ///< XGBoost booster handle

    int mNumFeatures = 6;  ///< Number of features expected by the model

   public:
    /** @brief Construct estimator by loading a model from path. */
    explicit SpeedEstimator(const std::string& pathToModel);

    /** @brief Predict speed on an edge given density. */
    double predictSpeed(const Edge& edge, double density);
};

#endif  // SPEEDESTIMATOR_H
