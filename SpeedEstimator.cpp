/**
 * @file SpeedEstimator.cpp
 * @brief XGBoost-based traffic speed estimator implementation.
 */
#include "SpeedEstimator.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>

SpeedEstimator::SpeedEstimator(const std::string &pathToModel) {
    BoosterHandle booster;
    XGBoosterCreate(nullptr, 0, &booster);
    XGBoosterLoadModel(booster, pathToModel.c_str());
    mBooster = booster;
    std::cout << "Speed model loaded from " + pathToModel << std::endl;
}

double SpeedEstimator::predictSpeed(const Edge &edge, double density) {
    std::vector<float> queryData({static_cast<float>(edge.getRoadType()), static_cast<float>(edge.getLength()),
                                  static_cast<float>(edge.getNumLanes()), static_cast<float>(edge.getSpeedLimit()),
                                  static_cast<float>(edge.getTlsCoeff()), static_cast<float>(density)});

    DMatrixHandle dtest;
    XGDMatrixCreateFromMat(queryData.data(), 1, mNumFeatures, -1, &dtest);

    // Run prediction
    bst_ulong out_len;
    const float *out_result;
    XGBoosterPredict(mBooster, dtest, 0, 0, 0, &out_len, &out_result);

    // Apply inverse log transformation (expm1) and ensure positive values
    double transformed_pred = std::max<double>(0.01, std::expm1(out_result[0]));

    // Clean up
    XGDMatrixFree(dtest);

    return transformed_pred;
}
