/**
 * @mainpage Dynamic Routing System
 *
 * @section overview Overview
 * A distributed dynamic routing system for road networks. It uses hierarchical
 * cover graphs and multi-criteria label-setting (MLS) search, combined with
 * learned estimators (density, speed, fuel) to plan routes in real time.
 *
 * @section modules Modules
 * - Graph: original network representation (`Node`, `Edge`, `Graph`).
 * - Cover: hierarchical abstraction (`CoverEdge`, `CoverGraph`).
 * - Estimators: XGBoost-based predictors (`TrafficEstimator`, `SpeedEstimator`, `FuelEstimator`).
 * - Router: distributed planning and MPI-based communication (`Router`).
 *
 * @section build Build
 * See CMake configuration in `CMakeLists.txt`.
 */
