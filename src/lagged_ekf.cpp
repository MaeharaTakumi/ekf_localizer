#include "ekf_localizer/lagged_ekf.hpp"

#include <algorithm>
#include <limits>
#include <utility>

namespace ekf_localizer
{

LaggedEkf::LaggedEkf(std::unique_ptr<VehicleEkf> ekf, double history_length)
: anchor_(std::move(ekf)),
  anchor_stamp_(-std::numeric_limits<double>::infinity()),
  history_length_(history_length),
  uses_odom_(dynamic_cast<const VehicleOdomEkf *>(anchor_.get()) != nullptr)
{
}

const VehicleEkf & LaggedEkf::current() const
{
  return history_.empty() ? *anchor_ : *history_.back().state;
}

std::unique_ptr<VehicleEkf> LaggedEkf::predicted(double t) const
{
  std::unique_ptr<VehicleEkf> ekf = current().clone();
  ekf->predictTo(t);
  return ekf;
}

void LaggedEkf::reset()
{
  // 最新の状態から始める（VehicleOdomEkf は直近のオドメトリを次の初期化に使う）
  std::unique_ptr<VehicleEkf> ekf = current().clone();
  ekf->reset();
  anchor_ = std::move(ekf);
  anchor_stamp_ = -std::numeric_limits<double>::infinity();
  history_.clear();
}

LaggedEkf::Outcome LaggedEkf::apply(VehicleEkf & ekf, const Measurement & m)
{
  Outcome out;
  out.was_initialized = ekf.initialized();
  if (m.type == Measurement::Type::kNdt) {
    if (!ekf.initialized()) {
      ekf.initialize(m.z_ndt, m.stamp);
      out.initialized_now = true;
    } else {
      ekf.predictTo(m.stamp);
      out.ndt = ekf.update(m.z_ndt);
    }
  } else {
    // オドメトリは uses_odom_ のときだけ入るので VehicleOdomEkf
    out.odom_accepted =
      static_cast<VehicleOdomEkf &>(ekf).updateOdom(m.v, m.omega, m.r_odom, m.stamp);
  }
  return out;
}

bool LaggedEkf::insert(
  const Measurement & m, Outcome & outcome, const VehicleEkf *& out_state, std::size_t & replayed)
{
  if (m.stamp < anchor_stamp_) {return false;}

  // 同じ時刻の観測の後ろに入れる（届いた順を保つ）
  const auto it = std::upper_bound(
    history_.begin(), history_.end(), m.stamp,
    [](double t, const Entry & e) {return t < e.m.stamp;});
  const std::size_t pos = static_cast<std::size_t>(it - history_.begin());

  // 直前の状態から m を適用する
  const VehicleEkf & before = (pos == 0) ? *anchor_ : *history_[pos - 1].state;
  Entry entry{m, before.clone()};
  outcome = apply(*entry.state, m);
  history_.insert(it, std::move(entry));

  // 後続の観測を時刻順に適用し直す
  replayed = history_.size() - pos - 1;
  for (std::size_t i = pos + 1; i < history_.size(); ++i) {
    history_[i].state = history_[i - 1].state->clone();
    apply(*history_[i].state, history_[i].m);
  }
  out_state = history_[pos].state.get();
  return true;
}

void LaggedEkf::prune()
{
  if (history_.empty()) {return;}
  const double oldest_kept = history_.back().m.stamp - history_length_;
  while (history_.size() > 1 && history_.front().m.stamp < oldest_kept) {
    anchor_ = std::move(history_.front().state);
    anchor_stamp_ = history_.front().m.stamp;
    history_.pop_front();
  }
}

LaggedEkf::NdtResult LaggedEkf::addNdt(const VehicleEkf::Vector6d & z_lidar, double stamp)
{
  Measurement m;
  m.type = Measurement::Type::kNdt;
  m.stamp = stamp;
  m.z_ndt = z_lidar;

  NdtResult res;
  Outcome outcome;
  const VehicleEkf * state = nullptr;
  if (!insert(m, outcome, state, res.replayed)) {return res;}

  res.status = res.replayed > 0 ? Status::kReplayed : Status::kApplied;
  res.initialized = outcome.initialized_now;
  res.update = outcome.ndt;
  res.horizontal_d2 = state->lastHorizontalMahalanobis();
  res.horizontal_reject_count = state->horizontalRejectCount();
  res.z_gate = state->zGate();
  res.roll_gate = state->rollGate();
  res.pitch_gate = state->pitchGate();
  prune();
  return res;
}

LaggedEkf::OdomResult LaggedEkf::addOdom(
  double v, double omega, const Eigen::Vector2d & r, double stamp)
{
  OdomResult res;
  if (!uses_odom_) {
    res.status = Status::kIgnored;
    return res;
  }

  Measurement m;
  m.type = Measurement::Type::kOdom;
  m.stamp = stamp;
  m.v = v;
  m.omega = omega;
  m.r_odom = r;

  Outcome outcome;
  const VehicleEkf * state = nullptr;
  if (!insert(m, outcome, state, res.replayed)) {return res;}

  const auto & odom_state = static_cast<const VehicleOdomEkf &>(*state);
  res.status = res.replayed > 0 ? Status::kReplayed : Status::kApplied;
  res.filter_initialized = outcome.was_initialized;
  res.accepted = outcome.odom_accepted;
  res.d2 = odom_state.lastOdomMahalanobis();
  res.reject_count = odom_state.odomRejectCount();
  prune();
  return res;
}

}  // namespace ekf_localizer
