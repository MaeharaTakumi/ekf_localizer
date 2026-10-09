#ifndef EKF_LOCALIZER__LAGGED_EKF_HPP_
#define EKF_LOCALIZER__LAGGED_EKF_HPP_

#include <cstddef>
#include <deque>

#include "ekf_localizer/ekf_core.hpp"
#include "ekf_localizer/observation_model.hpp"

namespace ekf_localizer
{

/// 遅れて届く観測を時刻順に入れ直す EKF（巻き戻し・再計算）
///
///   NDT は点群の取得時刻 t_z の観測だが、計算に時間がかかるので届くのは後になる。
///   その間に届いたオドメトリは届いた時点で適用しておき、NDT が届いたら t_z の直前の状態に戻して
///   NDT → それ以降のオドメトリ の順に適用し直す。
///
///   観測と「その観測を適用した直後の状態」を時刻順に保持する。最新の観測から history_length 秒より
///   古い分は先頭から捨て、捨てた最後の状態を起点（anchor）にする。起点より古い観測は使えない（kTooOld）。
///
///   初期化前に届いたオドメトリも履歴に入れる。遅れて届いた NDT で初期化したとき、
///   それ以降のオドメトリを適用し直して最新の時刻まで追いつく。
///
///   センサの種類を知らない。観測の処理は applyMeasurement()（観測モデル）に任せる。
/// ROS に依存しない。時刻はすべて double（秒）。
class LaggedEkf
{
public:
  enum class Status
  {
    kApplied,    ///< 最新の観測として適用した
    kReplayed,   ///< 過去に挿入し、後続の観測を適用し直した
    kTooOld,     ///< 履歴より古いので捨てた
  };

  /// 観測を入れた結果（入れた直後の判定。後続の再計算の影響は含まない）
  struct Result
  {
    Status status{Status::kTooOld};
    Outcome outcome;
    /// 適用し直した後続の観測の数
    std::size_t replayed{0};
  };

  /// core は初期化前のもの（観測モデルのゲートを確保したあと）
  LaggedEkf(const EkfCore & core, double history_length);

  /// 観測を t_z の位置に入れ、後続を適用し直す
  Result add(const MeasurementPtr & m);

  /// 履歴を捨て、EKF を初期化前に戻す（次に初期化できる観測で初期化し直す）
  void reset();

  bool initialized() const {return current().initialized();}
  /// 最新の観測まで反映した状態
  const EkfCore & current() const;
  /// current() を t まで予測したコピー（current() は変えない）
  EkfCore predicted(double t) const;
  /// 時刻 t の状態：t 以前の最後の観測を適用した直後の状態を t まで予測したコピー。
  /// t より新しい観測は使わない（t が起点より古ければ起点の状態のまま）
  EkfCore at(double t) const;
  std::size_t historySize() const {return history_.size();}
  double historyLength() const {return history_length_;}
  /// 最新の観測の時刻。履歴が空なら起点の時刻（リセット直後は -inf）
  double latestStamp() const {return history_.empty() ? anchor_stamp_ : history_.back().m->stamp;}

private:
  struct Entry
  {
    MeasurementPtr m;
    /// m を適用した直後の状態
    EkfCore state;
  };

  /// 最新から history_length_ より古い履歴を anchor_ に畳み込む
  void prune();

  EkfCore anchor_;
  /// anchor_ が表す時刻。これより古い観測は捨てる
  double anchor_stamp_;
  std::deque<Entry> history_;
  double history_length_;
};

}  // namespace ekf_localizer

#endif  // EKF_LOCALIZER__LAGGED_EKF_HPP_
