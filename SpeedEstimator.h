//
// Created by Tim on 10.12.2024.
//

#ifndef SPEEDESTIMATOR_H
#define SPEEDESTIMATOR_H

#include <xgboost/c_api.h>
#include <unordered_map>
#include <vector>
#include <memory>
#include "Edge.h"
#include "CoverEdge.h"

class SpeedEstimator {
private:
    BoosterHandle mBooster;

    int mNumFeatures = 6;

public:
    explicit SpeedEstimator(const std::string& pathToModel);

    double predictSpeed(const Edge& edge, double density);
};



#endif //SPEEDESTIMATOR_H
