#ifndef DYNAMIC_ROUTING_ESTIMATOR_SUPPORT_H
#define DYNAMIC_ROUTING_ESTIMATOR_SUPPORT_H

#include <xgboost/c_api.h>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace estimator {
inline void check(int result, const std::string &operation) {
    if (result != 0) throw std::runtime_error(operation + ": " + XGBGetLastError());
}
struct BoosterDeleter {
    void operator()(void *handle) const { if (handle) XGBoosterFree(handle); }
};
struct MatrixDeleter {
    void operator()(void *handle) const { if (handle) XGDMatrixFree(handle); }
};
using Booster = std::unique_ptr<void, BoosterDeleter>;

inline Booster loadModel(const std::string &path, bst_ulong features) {
    BoosterHandle handle = nullptr;
    check(XGBoosterCreate(nullptr, 0, &handle), "Create model " + path);
    Booster booster(handle);
    check(XGBoosterLoadModel(handle, path.c_str()), "Load model " + path);
    bst_ulong actual = 0;
    check(XGBoosterGetNumFeature(handle, &actual), "Read model feature count " + path);
    if (actual != features) throw std::runtime_error("Unexpected feature count in model " + path);
    return booster;
}

inline std::vector<double> predict(BoosterHandle booster, const std::vector<float> &data,
                                   bst_ulong rows, bst_ulong features) {
    if (!booster || data.size() != rows * features) throw std::invalid_argument("Invalid prediction input shape");
    for (float value : data) {
        if (!std::isfinite(value)) throw std::invalid_argument("Nonfinite prediction feature");
    }
    if (rows == 0) return {};
    DMatrixHandle handle = nullptr;
    check(XGDMatrixCreateFromMat(data.data(), rows, features, -1, &handle), "Create prediction matrix");
    std::unique_ptr<void, MatrixDeleter> matrix(handle);
    bst_ulong length = 0;
    const float *output = nullptr;
    check(XGBoosterPredict(booster, handle, 0, 0, 0, &length, &output), "Predict");
    if (!output || length != rows) throw std::runtime_error("Unexpected prediction output size");
    std::vector<double> result(output, output + length);
    for (double value : result) {
        if (!std::isfinite(value)) throw std::runtime_error("Nonfinite model prediction");
    }
    return result;
}

inline double inverseLog(double raw) {
    const double value = std::expm1(raw);
    if (!std::isfinite(value)) throw std::runtime_error("Nonfinite inverse-log prediction");
    return value < 0.01 ? 0.01 : value;
}
}
#endif
