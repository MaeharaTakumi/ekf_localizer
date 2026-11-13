#include "ekf_localizer/observation_model.hpp"

#include <utility>

namespace ekf_localizer
{

MeasurementPtr ObservationModel::makeMeasurement(
  double t, Eigen::VectorXd z, Eigen::VectorXd r) const
{
  auto m = std::make_shared<Measurement>();
  m->model = shared_from_this();
  m->time = t;
  m->z = std::move(z);
  m->r = std::move(r);
  return m;
}

ApplyResult applyMeasurement(EkfCore & core, const MeasurementPtr & m)
{
  const ObservationModel & model = *m->model;
  ApplyResult out;
  out.was_initialized = core.initialized();

  // 初期化済みでも覚える（初期姿勢を与え直したあとの初期化でも、直近の値を使う）
  if (model.aidsInitialization()) {core.setInitAid(m);}

  if (!core.initialized()) {
    const auto x0 = model.initialState(*m);
    if (!x0) {return out;}
    core.initialize(*x0, m->time);
    out.initialized_now = true;
    const MeasurementPtr & aid = core.initAid();
    out.aided = aid && aid->model->aidInitialization(core, *aid, m->time);
    return out;
  }

  core.predictTo(m->time);
  out.stages = model.update(core, *m);
  return out;
}

}  // namespace ekf_localizer
