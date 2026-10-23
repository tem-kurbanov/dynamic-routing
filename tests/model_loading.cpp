#include <xgboost/c_api.h>

#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <vector>

void check(int result) {
    if (result != 0) throw std::runtime_error(XGBGetLastError());
}

int main(int argc, char **argv) {
    if (argc != 2) return 2;
    int major, minor, patch;
    XGBoostVersion(&major, &minor, &patch);
    if (major != 3 || minor != 0 || patch != 5)
        throw std::runtime_error("Expected XGBoost runtime 3.0.5");
    int count = 0;
    for (const char *directory : {"xgrid/models", "xgrid/models/log", "intas/models/log"}) {
        for (auto [name, features] : {std::pair{"density", 8}, {"speed", 6}, {"fuel", 5}}) {
            auto path = std::filesystem::path(argv[1]) / directory / (std::string(name) + "_60.json");
            BoosterHandle booster;
            check(XGBoosterCreate(nullptr, 0, &booster));
            check(XGBoosterLoadModel(booster, path.c_str()));
            bst_ulong actual;
            check(XGBoosterGetNumFeature(booster, &actual));
            if (actual != static_cast<bst_ulong>(features)) throw std::runtime_error("Feature count mismatch");
            std::vector<float> row(features, 0.1F);
            DMatrixHandle matrix;
            check(XGDMatrixCreateFromMat(row.data(), 1, features, -1, &matrix));
            bst_ulong length;
            const float *prediction;
            check(XGBoosterPredict(booster, matrix, 0, 0, 0, &length, &prediction));
            if (length != 1 || !std::isfinite(prediction[0])) throw std::runtime_error("Invalid model prediction");
            check(XGDMatrixFree(matrix));
            check(XGBoosterFree(booster));
            ++count;
        }
    }
    std::cout << "XGBoost 3.0.5: loaded and predicted with " << count << " bundled models\n";
}
