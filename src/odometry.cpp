// WHILL の motor_speed_ts（左右輪の並進速度 [km/h]、Vector3Stamped）を
// 車体の速度 v, omega と加速度に換算し、タイムスタンプを付けて配信する。
//
//   出力 /wheelchair/odom : nav_msgs/Odometry         （twist のみ。pose は使わない）
//        /wheelchair/accel : geometry_msgs/AccelStamped （linear.x = dv/dt, angular.z = domega/dt）

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/accel_stamped.hpp"
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
    accel_tau_ = declare_parameter<double>("accel_time_constant", 0.2);
    input_timeout_ = declare_parameter<double>("input_timeout", 0.5);
    var_v_ = declare_parameter<double>("var_v", 2.5e-3);
    var_omega_ = declare_parameter<double>("var_omega", 5.0e-3);

    if (sign.size() != 2) {
      throw std::invalid_argument("wheel_sign は 2 要素（左, 右）で指定する");
    }
    sign_l_ = sign[0];
    sign_r_ = sign[1];

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/wheelchair/odom", 10);
    accel_pub_ = create_publisher<geometry_msgs::msg::AccelStamped>("/wheelchair/accel", 10);
    // whill_node の publisher は RELIABLE なので既定の QoS で接続できる
    sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      topic, 10, std::bind(&WhillOdometry::motorSpeedReceived, this, std::placeholders::_1));
  }

private:
  void motorSpeedReceived(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg)
  {
    const auto & speed = msg->vector;

    // motor_speed_ts の header.stamp は停止中に 0 になることがあるため、
    // その場合は use_stamped_input でも受信時刻で代用する
    const rclcpp::Time msg_stamp(msg->header.stamp, get_clock()->get_clock_type());
    rclcpp::Time stamp =
      (use_stamped_input_ && msg_stamp.nanoseconds() != 0) ? msg_stamp : now();
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

    // 加速度：速度の差分に 1 次ローパスを掛ける。
    // 0.1 km/h の量子化が 1 サンプル (50 ms) で約 0.56 m/s^2 の段差になるため平滑化は必須
    if (has_prev_) {
      const double dt = (stamp - prev_stamp_).seconds();
      if (dt > input_timeout_) {
        accel_v_ = 0.0;
        accel_omega_ = 0.0;
      } else {
        const double alpha = dt / (accel_tau_ + dt);
        accel_v_ += alpha * ((v - prev_v_) / dt - accel_v_);
        accel_omega_ += alpha * ((omega - prev_omega_) / dt - accel_omega_);
      }
    }
    has_prev_ = true;
    prev_stamp_ = stamp;
    prev_v_ = v;
    prev_omega_ = omega;

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
    // 並びは (x, y, z, roll, pitch, yaw)。差動二輪なので y, z, roll, pitch は拘束で 0
    odom.twist.covariance[0] = var_v_;
    odom.twist.covariance[7] = 1.0e-6;
    odom.twist.covariance[14] = 1.0e-6;
    odom.twist.covariance[21] = 1.0e-6;
    odom.twist.covariance[28] = 1.0e-6;
    odom.twist.covariance[35] = var_omega_;
    odom_pub_->publish(odom);

    geometry_msgs::msg::AccelStamped accel;
    accel.header.stamp = stamp;
    accel.header.frame_id = base_frame_id_;
    accel.accel.linear.x = accel_v_;
    accel.accel.angular.z = accel_omega_;
    accel_pub_->publish(accel);
  }

  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr accel_pub_;

  bool use_stamped_input_;
  double tread_width_;
  double sign_l_, sign_r_;
  std::string odom_frame_id_, base_frame_id_;
  double accel_tau_, input_timeout_;
  double var_v_, var_omega_;

  bool has_prev_{false};
  rclcpp::Time prev_stamp_;
  double prev_v_{0.0}, prev_omega_{0.0};
  double accel_v_{0.0}, accel_omega_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WhillOdometry>());
  rclcpp::shutdown();
  return 0;
}
