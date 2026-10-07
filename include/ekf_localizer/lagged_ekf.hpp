#ifndef EKF_LOCALIZER__LAGGED_EKF_HPP_
#define EKF_LOCALIZER__LAGGED_EKF_HPP_

#include <cstddef>
#include <deque>
#include <memory>

#include "ekf_localizer/vehicle_ekf.hpp"
#include "ekf_localizer/vehicle_odom_ekf.hpp"

namespace ekf_localizer
{

/// 遅れて届く観測を時刻順に入れ直す EKF（巻き戻し・再計算）
///
///   NDT は点群の取得時刻 t_scan の観測だが、計算に時間がかかるので届くのは後になる。
///   その間に届いたオドメトリは届いた時点で適用しておき、NDT が届いたら t_scan の直前の状態に戻して
///   NDT → それ以降のオドメトリ の順に適用し直す。
///
///   観測と「その観測を適用した直後の状態」を時刻順に保持する。最新の観測から history_length 秒より
///   古い分は先頭から捨て、捨てた最後の状態を起点（anchor）にする。起点より古い観測は使えない（kTooOld）。
///
///   初期化前に届いたオドメトリも履歴に入れる。遅れて届いた NDT で初期化したとき、
///   それ以降のオドメトリを適用し直して最新の時刻まで追いつく。
///
/// ROS に依存しない。時刻はすべて double（秒）。
class LaggedEkf
{
public:
  enum class Status
  {
    kApplied,    ///< 最新の観測として適用した
    kReplayed,   ///< 過去に挿入し、後続の観測を適用し直した
    kTooOld,     ///< 履歴より古いので捨てた
    kIgnored,    ///< このモデルでは使わない観測（use_odom: false でオドメトリ）
  };

  /// NDT 観測を入れた結果（入れた直後の判定。後続の再計算の影響は含まない）
  struct NdtResult
  {
    Status status{Status::kTooOld};
    /// この観測で EKF を初期化した
    bool initialized{false};
    /// 更新の判定（initialized のときは意味なし）
    VehicleEkf::UpdateResult update;
    double horizontal_d2{0.0};
    int horizontal_reject_count{0};
    VehicleEkf::ScalarGate z_gate;
    VehicleEkf::ScalarGate roll_gate;
    VehicleEkf::ScalarGate pitch_gate;
    /// 適用し直した後続の観測の数
    std::size_t replayed{0};
  };

  /// オドメトリ観測を入れた結果
  struct OdomResult
  {
    Status status{Status::kTooOld};
    /// 適用した時点で EKF が初期化済みだったか（未初期化なら値を記録しただけ）
    bool filter_initialized{false};
    bool accepted{false};
    double d2{0.0};
    int reject_count{0};
    std::size_t replayed{0};
  };

  /// ekf は初期化前のもの。オドメトリを使うなら VehicleOdomEkf を渡す
  LaggedEkf(std::unique_ptr<VehicleEkf> ekf, double history_length);

  /// NDT 観測（LiDAR の [x, y, z, roll, pitch, yaw]、stamp は点群の取得時刻）
  NdtResult addNdt(const VehicleEkf::Vector6d & z_lidar, double stamp);
  /// オドメトリ観測 [v, omega]。r は観測ノイズの分散
  OdomResult addOdom(double v, double omega, const Eigen::Vector2d & r, double stamp);

  /// 履歴を捨て、EKF を初期化前に戻す（次の NDT 観測で初期化し直す）
  void reset();

  bool usesOdom() const {return uses_odom_;}
  bool initialized() const {return current().initialized();}
  /// 最新の観測まで反映した状態
  const VehicleEkf & current() const;
  /// current() を t まで予測したコピー（current() は変えない）
  std::unique_ptr<VehicleEkf> predicted(double t) const;
  std::size_t historySize() const {return history_.size();}
  double historyLength() const {return history_length_;}

private:
  struct Measurement
  {
    enum class Type {kNdt, kOdom};
    Type type{Type::kNdt};
    double stamp{0.0};
    VehicleEkf::Vector6d z_ndt{VehicleEkf::Vector6d::Zero()};
    double v{0.0};
    double omega{0.0};
    Eigen::Vector2d r_odom{Eigen::Vector2d::Zero()};
  };

  struct Entry
  {
    Measurement m;
    /// m を適用した直後の状態
    std::unique_ptr<VehicleEkf> state;
  };

  /// 1 つの観測を適用した結果
  struct Outcome
  {
    bool was_initialized{false};
    bool initialized_now{false};
    VehicleEkf::UpdateResult ndt;
    bool odom_accepted{false};
  };

  static Outcome apply(VehicleEkf & ekf, const Measurement & m);
  /// m を時刻順の位置に入れ、後続を適用し直す。古すぎれば false。
  /// out_state は m を適用した直後の状態（次に履歴を変えるまで有効）
  bool insert(
    const Measurement & m, Outcome & outcome, const VehicleEkf *& out_state,
    std::size_t & replayed);
  /// 最新から history_length_ より古い履歴を anchor_ に畳み込む
  void prune();

  std::unique_ptr<VehicleEkf> anchor_;
  /// anchor_ が表す時刻。これより古い観測は捨てる
  double anchor_stamp_;
  std::deque<Entry> history_;
  double history_length_;
  bool uses_odom_;
};

}  // namespace ekf_localizer

#endif  // EKF_LOCALIZER__LAGGED_EKF_HPP_
