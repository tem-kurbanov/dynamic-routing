/**
 * @file TrafficEstimator.cpp
 * @brief XGBoost-based traffic density estimator implementation.
 */
//
// Created by Tim on 20.03.2024.
//

#include "TrafficEstimator.h"

#include <cmath>
#include <fstream>
#include <iostream>

TrafficEstimator::TrafficEstimator(const std::string &pathToModel) {
    BoosterHandle booster;
    XGBoosterCreate(nullptr, 0, &booster);
    XGBoosterLoadModel(booster, pathToModel.c_str());
    mBooster = booster;
    std::cout << "Density model loaded from " + pathToModel << std::endl;
}

double TrafficEstimator::predictDensity(const Edge &edge, std::pair<int, int> flows, double currentDensity) {
    // Prepare query data, predict using the model and return prediction
    std::vector<float> queryData({static_cast<float>(flows.first), static_cast<float>(flows.second),
                                  static_cast<float>(edge.getRoadType()), static_cast<float>(edge.getLength()),
                                  static_cast<float>(edge.getNumLanes()), static_cast<float>(edge.getSpeedLimit()),
                                  static_cast<float>(edge.getTlsCoeff()), static_cast<float>(currentDensity)});

    DMatrixHandle dtest;
    XGDMatrixCreateFromMat(queryData.data(), 1, mNumFeatures, -1, &dtest);

    // Run prediction
    bst_ulong out_len;
    const float *out_result;
    XGBoosterPredict(mBooster, dtest, 0, 0, 0, &out_len, &out_result);

    // Apply inverse logit transformation (sigmoid)
    double transformed_pred = 1.0 / (1.0 + std::exp(-out_result[0]));

    // Clean up
    XGDMatrixFree(dtest);

    return transformed_pred;
}

std::vector<double> TrafficEstimator::toContiguousVector(const std::vector<std::vector<double>> &data) {
    std::vector<double> contiguous;
    for (const auto &row : data) {
        contiguous.insert(contiguous.end(), row.begin(), row.end());
    }
    return contiguous;
}

void TrafficEstimator::calculateClassificationMetrics(const std::vector<double> &actual,
                                                      const std::vector<int> &predicted) {
    if (actual.size() != predicted.size()) {
        throw std::invalid_argument("Size of actual and predicted values must be the same.");
    }

    size_t n = actual.size();
    int num_classes = 5;  // Since labels range from 1 to 5
    std::vector<std::vector<int>> confusion_matrix(num_classes, std::vector<int>(num_classes, 0));

    int correct_predictions = 0;

    // Compute confusion matrix
    for (size_t i = 0; i < n; ++i) {
        int actual_class = static_cast<int>(std::round(actual[i])) - 1;  // Convert to zero-based index
        int predicted_class = predicted[i] - 1;                          // Convert to zero-based index

        confusion_matrix[actual_class][predicted_class] += 1;

        if (actual_class == predicted_class) {
            correct_predictions++;
        }
    }

    // Accuracy
    double accuracy = static_cast<double>(correct_predictions) / n * 100.0;

    // Calculate Precision, Recall, and F1-Score for each class
    std::vector<double> precision(num_classes, 0.0);
    std::vector<double> recall(num_classes, 0.0);
    std::vector<double> f1_score(num_classes, 0.0);

    for (int i = 0; i < num_classes; ++i) {
        int true_positives = confusion_matrix[i][i];
        int false_positives = 0;
        int false_negatives = 0;

        for (int j = 0; j < num_classes; ++j) {
            if (i != j) {
                false_positives += confusion_matrix[j][i];
                false_negatives += confusion_matrix[i][j];
            }
        }

        int predicted_positives = true_positives + false_positives;
        int actual_positives = true_positives + false_negatives;

        precision[i] = (predicted_positives > 0) ? static_cast<double>(true_positives) / predicted_positives : 0.0;
        recall[i] = (actual_positives > 0) ? static_cast<double>(true_positives) / actual_positives : 0.0;
        f1_score[i] =
            (precision[i] + recall[i] > 0) ? 2 * (precision[i] * recall[i]) / (precision[i] + recall[i]) : 0.0;
    }

    // Print all metrics
    std::cout << "Classification Metrics:" << std::endl;
    std::cout << "1. Accuracy: " << accuracy << "%" << std::endl;

    // Print Precision, Recall, and F1-Score for each class
    for (int i = 0; i < num_classes; ++i) {
        std::cout << "Class " << (i + 1) << " Metrics:" << std::endl;
        std::cout << "   Precision: " << precision[i] * 100 << "%" << std::endl;
        std::cout << "   Recall: " << recall[i] * 100 << "%" << std::endl;
        std::cout << "   F1-Score: " << f1_score[i] * 100 << "%" << std::endl;
    }

    // Optionally: Print the confusion matrix
    std::cout << "\nConfusion Matrix:" << std::endl;
    for (int i = 0; i < num_classes; ++i) {
        for (int j = 0; j < num_classes; ++j) {
            std::cout << confusion_matrix[i][j] << " ";
        }
        std::cout << std::endl;
    }
}
