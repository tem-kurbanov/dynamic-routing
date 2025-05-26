#include "FuelEstimator.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>

FuelEstimator::FuelEstimator(const std::string& pathToModel) {
    BoosterHandle booster;
    XGBoosterCreate(nullptr, 0, &booster);
    XGBoosterLoadModel(booster, pathToModel.c_str());
    mBooster = booster;
    std::cout << "Fuel model loaded from " + pathToModel << std::endl;
}

double FuelEstimator::predictFuel(const Edge& edge, double density, double speed) {
    std::vector<float> queryData({static_cast<float>(density), static_cast<float>(speed),
                                  static_cast<float>(edge.getRoadType()), static_cast<float>(edge.getLength()),
                                  static_cast<float>(edge.getTlsCoeff())});

    DMatrixHandle dtest;
    XGDMatrixCreateFromMat(queryData.data(), 1, mNumFeatures, -1, &dtest);

    // Run prediction
    bst_ulong out_len;
    const float* out_result;
    XGBoosterPredict(mBooster, dtest, 0, 0, 0, &out_len, &out_result);

    // Apply inverse log transformation (expm1) and ensure positive values
    double transformed_pred = std::max<double>(0.01, std::expm1(out_result[0]));

    // Clean up
    XGDMatrixFree(dtest);

    return transformed_pred;
}

std::vector<double> FuelEstimator::predictFuelBatch(std::vector<const Edge*>& edges, std::vector<double>& densities,
                                                    std::vector<double>& speeds) {

    int numSamples = edges.size();
    std::vector<float> queryData;
    queryData.reserve(numSamples * mNumFeatures);

    // Prepare data for batch prediction
    for (int i = 0; i < numSamples; ++i) {
        const Edge* edge = edges[i];
        queryData.push_back(static_cast<float>(densities[i]));
        queryData.push_back(static_cast<float>(speeds[i]));
        queryData.push_back(static_cast<float>(edge->getRoadType()));
        queryData.push_back(static_cast<float>(edge->getLength()));
        queryData.push_back(static_cast<float>(edge->getTlsCoeff()));
    }

    // Create DMatrix for batch prediction
    DMatrixHandle dtest;
    XGDMatrixCreateFromMat(queryData.data(), numSamples, mNumFeatures, -1, &dtest);

    // Run batch prediction
    bst_ulong out_len;
    const float* out_result;
    XGBoosterPredict(mBooster, dtest, 0, 0, 0, &out_len, &out_result);

    // Convert results to vector and apply inverse log transformation (expm1)
    std::vector<double> results;
    results.reserve(out_len);
    for (size_t i = 0; i < out_len; ++i) {
        results.push_back(std::max<double>(0.01, std::expm1(out_result[i])));
    }

    // Clean up
    XGDMatrixFree(dtest);

    return results;
}