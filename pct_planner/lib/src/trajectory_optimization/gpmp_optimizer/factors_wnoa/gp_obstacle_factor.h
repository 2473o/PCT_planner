#pragma once

#include <memory>

#include "gtsam/nonlinear/NonlinearFactor.h"
#include "map_manager/dense_elevation_map.h"

class GPObstacleFactorWnoa : public gtsam::NoiseModelFactor1<gtsam::Vector4> {
 public:
  GPObstacleFactorWnoa(gtsam::Key key, std::shared_ptr<DenseElevationMap> map,
                       int current_layer, const double height_hint,
                       const double q_cost, const double cost_threshold,
                       bool verbose = false)
      : NoiseModelFactor1(gtsam::noiseModel::Isotropic::Sigma(1, q_cost), key),
        map_(map),
        cost_threshold_(cost_threshold),
        current_layer_(current_layer),
        height_hint_(height_hint),
        verbose_(verbose) {}
  ~GPObstacleFactorWnoa() = default;

  int GetNodeLayer() const { return current_layer_; }

  gtsam::Vector evaluateError(
      const gtsam::Vector4& x1,
      boost::optional<gtsam::Matrix&> H1 = boost::none) const override;

  void verbose() { verbose_ = true; }

 private:
  std::shared_ptr<DenseElevationMap> map_;
  double cost_threshold_ = 0.0;
  mutable int current_layer_ = 0;
  mutable double height_hint_ = 0.0;
  bool verbose_ = false;
};