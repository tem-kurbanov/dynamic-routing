/**
 * @file SpeedEstimator.cpp
 * @brief XGBoost-based traffic speed estimator implementation.
 */
#include "SpeedEstimator.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>

SpeedEstimator::SpeedEstimator(const std::string &pathToModel)
    : mBooster(estimator::loadModel(pathToModel, 6)) {
    std::cout << "Speed model loaded from " + pathToModel << std::endl;
}

double SpeedEstimator::predictSpeed(const Edge &edge, double density) {
    std::vector<float> queryData({static_cast<float>(edge.getRoadType()), static_cast<float>(edge.getLength()),
                                  static_cast<float>(edge.getNumLanes()), static_cast<float>(edge.getSpeedLimit()),
                                  static_cast<float>(edge.getTlsCoeff()), static_cast<float>(density)});

    const double raw = estimator::predict(mBooster.get(), queryData, 1, mNumFeatures).at(0);
    return estimator::inverseLog(raw);
}
