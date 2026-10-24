/**
 * @file FuelEstimator.cpp
 * @brief XGBoost-based fuel consumption estimator implementation.
 */
#include "FuelEstimator.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>

FuelEstimator::FuelEstimator(const std::string &pathToModel)
    : mBooster(estimator::loadModel(pathToModel, 5)) {
    std::cout << "Fuel model loaded from " + pathToModel << std::endl;
}

double FuelEstimator::predictFuel(const Edge& edge, double density, double speed) {
    std::vector<float> queryData({static_cast<float>(density), static_cast<float>(speed),
                                  static_cast<float>(edge.getRoadType()), static_cast<float>(edge.getLength()),
                                  static_cast<float>(edge.getTlsCoeff())});

    const double raw = estimator::predict(mBooster.get(), queryData, 1, mNumFeatures).at(0);
    return estimator::inverseLog(raw);
}

std::vector<double> FuelEstimator::predictFuelBatch(std::vector<const Edge*>& edges, std::vector<double>& densities,
                                                    std::vector<double>& speeds) {

    if (edges.size() != densities.size() || edges.size() != speeds.size()) {
        throw std::invalid_argument("Fuel batch arrays must have equal lengths");
    }
    const size_t numSamples = edges.size();
    std::vector<float> queryData;
    queryData.reserve(numSamples * mNumFeatures);

    // Prepare data for batch prediction
    for (int i = 0; i < numSamples; ++i) {
        const Edge* edge = edges[i];
        if (!edge) throw std::invalid_argument("Null edge in fuel batch");
        queryData.push_back(static_cast<float>(densities[i]));
        queryData.push_back(static_cast<float>(speeds[i]));
        queryData.push_back(static_cast<float>(edge->getRoadType()));
        queryData.push_back(static_cast<float>(edge->getLength()));
        queryData.push_back(static_cast<float>(edge->getTlsCoeff()));
    }

    auto results = estimator::predict(mBooster.get(), queryData, numSamples, mNumFeatures);
    for (double &value : results) value = estimator::inverseLog(value);
    return results;
}