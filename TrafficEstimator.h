
#ifndef HIERARCHICALSYSTEM_TRAFFICESTIMATOR_H
#define HIERARCHICALSYSTEM_TRAFFICESTIMATOR_H

#include <xgboost/c_api.h>

#include <memory>
#include <unordered_map>
#include <vector>

#include "CoverEdge.h"
#include "Edge.h"

class TrafficEstimator {
   private:
    BoosterHandle mBooster;

    int mNumFeatures = 8;

    std::vector<double> toContiguousVector(const std::vector<std::vector<double>>& data);

    void calculateClassificationMetrics(const std::vector<double>& actual, const std::vector<int>& predicted);

   public:
    explicit TrafficEstimator(const std::string& pathToModel);

    std::unordered_map<int, double> predictDensities(const std::unordered_map<int, std::vector<double>>& edgeData);

    double predictDensity(const Edge& edge, std::pair<int, int> flows, double currentDensity);
};

#endif  // HIERARCHICALSYSTEM_TRAFFICESTIMATOR_H
