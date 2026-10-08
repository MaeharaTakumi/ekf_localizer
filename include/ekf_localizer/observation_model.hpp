#ifndef EKF_LOCALIZER__OBSERVATION_MODEL_HPP_
#define EKF_LOCALIZER__OBSERVATION_MODEL_HPP_

#include <Eigen/Core>

#include <memory>
#include <optional>
#include <vector>

#include "ekf_localizer/ekf_core.hpp"

namespace ekf_localizer
{

class ObservationModel;

/// 観測 1 つ。作ったあとは変えない（履歴と初期化用の観測で共有する）
struct Measurement
{
  /// この観測を処理する観測モデル
  std::shared_ptr<const ObservationModel> model;
  /// 観測の時刻 t_z [s]（stamp から遅れを引いた値）
  double stamp{0.0};
  /// 観測値
  Eigen::VectorXd z;
  /// 観測ノイズの分散（z と同じ並び）
  Eigen::VectorXd r;
};

using MeasurementPtr = std::shared_ptr<const Measurement>;

/// 更新の 1 段の結果（観測モデルは更新を 1 段以上に分けて行う）
struct StageResult
{
  enum class Status {kUpdated, kRejected, kReinitialized};

  /// 段の名前（ログ用。"x/y/yaw" など）
  const char * name{""};
  Status status{Status::kRejected};
  double d2{0.0};
  /// この段の連続棄却回数（判定のあと）
  int reject_count{0};
  /// kReinitialized のとき、合わせた観測値
  double reinitialized_to{0.0};
};

/// 観測モデルのインターフェース（センサごとに 1 つ実装する）
///
///   観測モデルは設定だけを持ち、変わる値は持たない。ゲートの状態など、履歴と一緒に巻き戻す値は
///   EkfCore に置く（コンストラクタで EkfCore::addGates() を呼んで確保する）。
///   std::make_shared で作ること（measurement() が自分への shared_ptr を観測に入れる）。
class ObservationModel : public std::enable_shared_from_this<ObservationModel>
{
public:
  virtual ~ObservationModel() = default;

  /// センサの名前（ログ用）
  virtual const char * name() const = 0;

  /// この観測で未初期化の EKF を初期化できるなら、初期状態を返す（速度など観測しない成分は 0）
  virtual std::optional<EkfCore::StateVector> initialState(const Measurement &) const
  {
    return std::nullopt;
  }

  /// 初期化のときに使うため、最新の 1 つを覚えておく観測か
  virtual bool aidsInitialization() const {return false;}
  /// 覚えておいた観測 m で、時刻 t_init に初期化した直後の状態を補う。補ったら true
  virtual bool aidInitialization(EkfCore &, const Measurement &, double /*t_init*/) const
  {
    return false;
  }

  /// 観測で更新する（予測は済ませてある）。段ごとの結果を返す
  virtual std::vector<StageResult> update(EkfCore & core, const Measurement & m) const = 0;

protected:
  /// この観測モデルの観測を作る
  MeasurementPtr makeMeasurement(double stamp, Eigen::VectorXd z, Eigen::VectorXd r) const;
};

/// 観測を 1 つ適用した結果
struct Outcome
{
  /// 適用する前に初期化済みだったか
  bool was_initialized{false};
  /// この観測で初期化した
  bool initialized_now{false};
  /// 初期化のとき、覚えておいた観測で状態を補った
  bool aided{false};
  /// 更新の段ごとの結果（初期化済みで更新したときだけ）
  std::vector<StageResult> stages;
};

/// 観測 m を core に適用する（NDT もオドメトリも同じ流れ）
///   1. 初期化用の観測なら覚えておく
///   2. 未初期化：初期化できる観測なら初期化し、覚えておいた観測で補う。できなければ何もしない
///   3. 初期化済み：t_z まで予測して更新する
Outcome applyMeasurement(EkfCore & core, const MeasurementPtr & m);

}  // namespace ekf_localizer

#endif  // EKF_LOCALIZER__OBSERVATION_MODEL_HPP_
