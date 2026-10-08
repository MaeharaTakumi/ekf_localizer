#include "ekf_localizer/lagged_ekf.hpp"

#include <algorithm>
#include <limits>

namespace ekf_localizer
{

LaggedEkf::LaggedEkf(const EkfCore & core, double history_length)
: anchor_(core),
  anchor_stamp_(-std::numeric_limits<double>::infinity()),
  history_length_(history_length)
{
}

const EkfCore & LaggedEkf::current() const
{
  return history_.empty() ? anchor_ : history_.back().state;
}

EkfCore LaggedEkf::predicted(double t) const
{
  EkfCore ekf = current();
  ekf.predictTo(t);
  return ekf;
}

void LaggedEkf::reset()
{
  // 最新の状態から始める（初期化用の観測は残り、次の初期化で使う）
  EkfCore ekf = current();
  ekf.reset();
  anchor_ = ekf;
  anchor_stamp_ = -std::numeric_limits<double>::infinity();
  history_.clear();
}

LaggedEkf::Result LaggedEkf::add(const MeasurementPtr & m)
{
  Result res;
  if (m->stamp < anchor_stamp_) {return res;}

  // 同じ時刻の観測の後ろに入れる（届いた順を保つ）
  const auto it = std::upper_bound(
    history_.begin(), history_.end(), m->stamp,
    [](double t, const Entry & e) {return t < e.m->stamp;});
  const std::size_t pos = static_cast<std::size_t>(it - history_.begin());

  // 直前の状態から m を適用する
  Entry entry{m, (pos == 0) ? anchor_ : history_[pos - 1].state};
  res.outcome = applyMeasurement(entry.state, m);
  history_.insert(it, std::move(entry));

  // 後続の観測を時刻順に適用し直す
  res.replayed = history_.size() - pos - 1;
  for (std::size_t i = pos + 1; i < history_.size(); ++i) {
    history_[i].state = history_[i - 1].state;
    applyMeasurement(history_[i].state, history_[i].m);
  }
  res.status = res.replayed > 0 ? Status::kReplayed : Status::kApplied;
  prune();
  return res;
}

void LaggedEkf::prune()
{
  if (history_.empty()) {return;}
  const double oldest_kept = history_.back().m->stamp - history_length_;
  while (history_.size() > 1 && history_.front().m->stamp < oldest_kept) {
    anchor_ = std::move(history_.front().state);
    anchor_stamp_ = history_.front().m->stamp;
    history_.pop_front();
  }
}

}  // namespace ekf_localizer
