// WHILL の motor_speed_ts（左右輪の並進速度 [km/h]、Vector3Stamped）を
// 車体の速度 v, omega に換算し、タイムスタンプを付けて配信する。
//
//   出力 /wheelchair/odom : nav_msgs/Odometry         （twist のみ。pose は使わない）

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

class WhillOdometry : public rclcpp::Node
{
public:
  WhillOdometry()
  : Node("whill_odometry")
  {
    const auto topic = declare_parameter<std::string>(
      "input_topic_name", "/Drp5_whill_red/whill_node/motor_speed_ts");
    // true: 入力の header.stamp を使う、false: 受信時刻を付ける
    use_stamped_input_ = declare_parameter<bool>("use_stamped_input", false);
    tread_width_ = declare_parameter<double>("tread_width", 0.495);
    const auto sign = declare_parameter<std::vector<double>>("wheel_sign", {1.0, -1.0});
    odom_frame_id_ = declare_parameter<std::string>("odom_frame_id", "odom");
    base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base_link");

    if (sign.size() != 2) {
      throw std::invalid_argument("wheel_sign は 2 要素（左, 右）で指定する");
    }
    sign_l_ = sign[0];
    sign_r_ = sign[1];

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/wheelchair/odom", 10);
    // whill_node の publisher は RELIABLE なので既定の QoS で接続できる
    sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      topic, 10, std::bind(&WhillOdometry::motorSpeedReceived, this, std::placeholders::_1));
  }

private:
  void motorSpeedReceived(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg)
  {
    const auto & speed = msg->vector;

    rclcpp::Time stamp = use_stamped_input_ ?
      rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type()) : now();
    if (has_prev_ && stamp <= prev_stamp_) {
      // 時刻が戻った・重なった場合も単調増加を保つ
      stamp = prev_stamp_ + rclcpp::Duration(0, 1);
    }

    // [km/h] → [m/s]、左右を前進正に揃えてから差動二輪の式で換算
    constexpr double kKmphToMps = 1.0 / 3.6;
    // motor_speed は x = 右輪（前進で負）、y = 左輪（前進で正）。
    // 前進で v > 0 は実機で確認済み。左旋回で omega < 0 になったため x/y を右/左と判断した
    const double w_l = speed.y * kKmphToMps * sign_l_;
    const double w_r = speed.x * kKmphToMps * sign_r_;
    const double v = (w_l + w_r) * 0.5;
    const double omega = (w_r - w_l) / tread_width_;

    has_prev_ = true;
    prev_stamp_ = stamp;

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_id_;
    odom.child_frame_id = base_frame_id_;
    odom.pose.pose.orientation.w = 1.0;
    // pose は配信しない。受け取り側が使わないよう分散を大きくしておく
    for (int i = 0; i < 6; ++i) {
      odom.pose.covariance[i * 7] = 1.0e9;
    }
    odom.twist.twist.linear.x = v;
    odom.twist.twist.angular.z = omega;
    // twist の共分散は付けない（観測ノイズは ekf_localizer が計算する）
    odom_pub_->publish(odom);
  }

  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  bool use_stamped_input_;
  double tread_width_;
  double sign_l_, sign_r_;
  std::string odom_frame_id_, base_frame_id_;

  bool has_prev_{false};
  rclcpp::Time prev_stamp_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WhillOdometry>());
  rclcpp::shutdown();
  return 0;
}
