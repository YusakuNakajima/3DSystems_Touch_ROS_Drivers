// === ROS 2 Core ===
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>

// === Phantom Omni SDK (OpenHaptics) ===
#include <HD/hd.h>
#include <HL/hl.h>
#include <HDU/hduVector.h>
#include <HDU/hduError.h>
#include <HDU/hduMatrix.h>

// === Custom Message ===
#include "omni_msgs/msg/omni_button_event.hpp"
#include "omni_msgs/msg/omni_feedback.hpp"

// === C++ Standard Library ===
#include <string>
#include <sstream>
#include <memory>
#include <cmath>
#include <cassert>
#include <thread>
#include <chrono>

int calibrationStyle = HD_CALIBRATION_AUTO;
const char* DEVICE_NAME = "Default Device";

// Phantom Omniの状態を保存する構造体
struct OmniState {
  // 現在の位置と速度
  hduVector3Dd position;   // デバイス位置 (mm)
  hduVector3Dd velocity;   // フィルタリングされた速度 (mm/s)

  // 速度フィルタリング用の過去履歴（3ステップのIIR）
  hduVector3Dd inp_vel1;
  hduVector3Dd inp_vel2;
  hduVector3Dd inp_vel3;
  hduVector3Dd out_vel1;
  hduVector3Dd out_vel2;
  hduVector3Dd out_vel3;

  // 2次の後退差分で使う過去の位置履歴
  hduVector3Dd pos_hist1;
  hduVector3Dd pos_hist2;

  // 回転（GIMBAL角）とジョイント角（7DOF風に記録）
  hduVector3Dd rot;       // Gimbal angles
  hduVector3Dd joints;    // Joint angles (from Omni)

  // 力（x, y, z）
  hduVector3Dd force;

  // 各関節角（仮想的な6自由度用）
  float thetas[7];

  // ボタン状態と直前の状態
  int buttons[2];
  int buttons_prev[2];

  // ロック機能（ボタン同時押しで切り替え）
  bool lock;
  hduVector3Dd lock_pos;

  // コンストラクタでゼロ初期化
  OmniState() {
    hduVector3Dd zero(0, 0, 0);
    position = velocity = zero;
    inp_vel1 = inp_vel2 = inp_vel3 = zero;
    out_vel1 = out_vel2 = out_vel3 = zero;
    pos_hist1 = pos_hist2 = zero;
    rot = joints = force = lock_pos = zero;

    for (int i = 0; i < 7; ++i) thetas[i] = 0.0f;
    buttons[0] = buttons[1] = 0;
    buttons_prev[0] = buttons_prev[1] = 0;
    lock = true;
  }
};



class PhantomROS : public rclcpp::Node {
public:
  PhantomROS(OmniState* state)
  : Node("omni_haptic_node"), state_(state) {

    // パラメータの取得
    this->declare_parameter<std::string>("omni_name", "phantom");
    this->get_parameter("omni_name", omni_name_);

    // パブリッシャの作成
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(omni_name_ + "/pose", 10);
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    button_pub_ = this->create_publisher<omni_msgs::msg::OmniButtonEvent>(omni_name_ + "/button", 10);

    // サブスクライバの作成（力フィードバック）
    haptic_sub_ = this->create_subscription<omni_msgs::msg::OmniFeedback>(
      omni_name_ + "/force_feedback", 10,
      std::bind(&PhantomROS::force_callback, this, std::placeholders::_1)
    );

    // TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // 各リンク名
    for (int i = 0; i < 7; ++i) {
      link_names_[i] = omni_name_ + "_link" + std::to_string(i);
    }

    // 10ms周期で状態をPublish
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&PhantomROS::publish_omni_state, this));
  }

private:
  void force_callback(const omni_msgs::msg::OmniFeedback::SharedPtr msg) {
    // 速度減衰付きのフィードバック力
    state_->force[0] = msg->force.x - 0.001 * state_->velocity[0];
    state_->force[1] = msg->force.y - 0.001 * state_->velocity[1];
    state_->force[2] = msg->force.z - 0.001 * state_->velocity[2];

    // ロック位置更新
    state_->lock_pos[0] = msg->position.x;
    state_->lock_pos[1] = msg->position.y;
    state_->lock_pos[2] = msg->position.z;
  }

  void publish_omni_state() {
    // JointState の発行
    auto joint_msg = sensor_msgs::msg::JointState();
    joint_msg.header.stamp = this->now();
    joint_msg.name = {"waist", "shoulder", "elbow", "yaw", "pitch", "roll"};
    joint_msg.position = {
      -state_->thetas[1],
      state_->thetas[2],
      state_->thetas[3],
      -state_->thetas[4] + M_PI,
      -state_->thetas[5] - 3 * M_PI / 4,
      -state_->thetas[6] - M_PI
    };
    joint_pub_->publish(joint_msg);

    // Pose の発行（エンドエフェクタの位置）
    auto pose_msg = geometry_msgs::msg::PoseStamped();
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = link_names_[6];
    pose_msg.pose.position.x = 0.0;
    pose_msg.pose.orientation.w = 1.0;
    pose_pub_->publish(pose_msg);

    // ボタンイベントの検出と送信
    if ((state_->buttons[0] != state_->buttons_prev[0]) || (state_->buttons[1] != state_->buttons_prev[1])) {
      if ((state_->buttons[0] == 1) && (state_->buttons[1] == 1)) {
        state_->lock = !state_->lock;
      }

      auto button_event = omni_msgs::msg::OmniButtonEvent();
      button_event.grey_button = state_->buttons[0];
      button_event.white_button = state_->buttons[1];
      button_pub_->publish(button_event);

      state_->buttons_prev[0] = state_->buttons[0];
      state_->buttons_prev[1] = state_->buttons[1];
    }
  }

  // 内部メンバ変数
  OmniState* state_;
  std::string omni_name_;
  std::string link_names_[7];

  // ROS 2 通信用オブジェクト
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<omni_msgs::msg::OmniButtonEvent>::SharedPtr button_pub_;
  rclcpp::Subscription<omni_msgs::msg::OmniFeedback>::SharedPtr haptic_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};



HDCallbackCode HDCALLBACK omni_state_callback(void *pUserData) {
  OmniState *omni_state = static_cast<OmniState *>(pUserData);

  if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE) {
    RCLCPP_DEBUG(rclcpp::get_logger("omni_haptic_node"), "Updating calibration...");
    hdUpdateCalibration(calibrationStyle);
  }

  hdBeginFrame(hdGetCurrentDevice());

  // デバイス状態の取得
  hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, omni_state->rot);
  hdGetDoublev(HD_CURRENT_POSITION, omni_state->position);
  hdGetDoublev(HD_CURRENT_JOINT_ANGLES, omni_state->joints);

  // 速度推定（2次の後退差分 + IIRフィルタ）
  hduVector3Dd vel_buff = (omni_state->position * 3 - 4 * omni_state->pos_hist1
                          + omni_state->pos_hist2) / 0.002;
  omni_state->velocity = (0.2196 * (vel_buff + omni_state->inp_vel3)
                        + 0.6588 * (omni_state->inp_vel1 + omni_state->inp_vel2)) / 1000.0
                        - (-2.7488 * omni_state->out_vel1
                        + 2.5282 * omni_state->out_vel2
                        - 0.7776 * omni_state->out_vel3);

  // 履歴更新
  omni_state->pos_hist2 = omni_state->pos_hist1;
  omni_state->pos_hist1 = omni_state->position;
  omni_state->inp_vel3 = omni_state->inp_vel2;
  omni_state->inp_vel2 = omni_state->inp_vel1;
  omni_state->inp_vel1 = vel_buff;
  omni_state->out_vel3 = omni_state->out_vel2;
  omni_state->out_vel2 = omni_state->out_vel1;
  omni_state->out_vel1 = omni_state->velocity;

  // ロックモード中は仮想スプリングによる力生成
  if (omni_state->lock) {
    omni_state->force = 0.04 * (omni_state->lock_pos - omni_state->position)
                      - 0.001 * omni_state->velocity;
  }

  // 力をデバイスに送信
  hdSetDoublev(HD_CURRENT_FORCE, omni_state->force);

  // ボタン取得
  int nButtons = 0;
  hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
  omni_state->buttons[0] = (nButtons & HD_DEVICE_BUTTON_1) ? 1 : 0;
  omni_state->buttons[1] = (nButtons & HD_DEVICE_BUTTON_2) ? 1 : 0;

  hdEndFrame(hdGetCurrentDevice());

  // エラーチェック
  HDErrorInfo error;
  if (HD_DEVICE_ERROR(error = hdGetError())) {
    hduPrintError(stderr, &error, "Error during main scheduler callback");
    if (hduIsSchedulerError(&error))
      return HD_CALLBACK_DONE;
  }

  // 仮想関節角の更新（全体6自由度想定）
  float t[7] = {
    0.0,
    omni_state->joints[0],
    omni_state->joints[1],
    omni_state->joints[2] - omni_state->joints[1],
    omni_state->rot[0],
    omni_state->rot[1],
    omni_state->rot[2]
  };
  for (int i = 0; i < 7; ++i)
    omni_state->thetas[i] = t[i];

  return HD_CALLBACK_CONTINUE;
}


void HHD_Auto_Calibration() {
  int supportedCalibrationStyles;
  HDErrorInfo error;

  hdGetIntegerv(HD_CALIBRATION_STYLE, &supportedCalibrationStyles);

  if (supportedCalibrationStyles & HD_CALIBRATION_ENCODER_RESET) {
    calibrationStyle = HD_CALIBRATION_ENCODER_RESET;
    RCLCPP_INFO(rclcpp::get_logger("omni_haptic_node"), "HD_CALIBRATION_ENCODER_RESET selected.");
  }
  if (supportedCalibrationStyles & HD_CALIBRATION_INKWELL) {
    calibrationStyle = HD_CALIBRATION_INKWELL;
    RCLCPP_INFO(rclcpp::get_logger("omni_haptic_node"), "HD_CALIBRATION_INKWELL selected.");
  }
  if (supportedCalibrationStyles & HD_CALIBRATION_AUTO) {
    calibrationStyle = HD_CALIBRATION_AUTO;
    RCLCPP_INFO(rclcpp::get_logger("omni_haptic_node"), "HD_CALIBRATION_AUTO selected.");
  }

  if (calibrationStyle == HD_CALIBRATION_ENCODER_RESET) {
    do {
      hdUpdateCalibration(calibrationStyle);
      RCLCPP_INFO(rclcpp::get_logger("omni_haptic_node"), "Calibrating... (put stylus in well)");
      if (HD_DEVICE_ERROR(error = hdGetError())) {
        hduPrintError(stderr, &error, "Reset encoders reset failed.");
        break;
      }
    } while (hdCheckCalibration() != HD_CALIBRATION_OK);

    RCLCPP_INFO(rclcpp::get_logger("omni_haptic_node"), "Calibration complete.");
  }

  if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_MANUAL_INPUT) {
    RCLCPP_INFO(rclcpp::get_logger("omni_haptic_node"), "Please place the device into the inkwell for calibration.");
  }
}

int main(int argc, char** argv) {
  // === ROS 2 初期化 ===
  rclcpp::init(argc, argv);

  // === Phantom デバイス初期化 ===
  HDErrorInfo error;
  HHD hHD = hdInitDevice(DEVICE_NAME);
  if (HD_DEVICE_ERROR(error = hdGetError())) {
    RCLCPP_ERROR(rclcpp::get_logger("omni_haptic_node"), "Failed to initialize haptic device.");
    return -1;
  }

  RCLCPP_INFO(rclcpp::get_logger("omni_haptic_node"), "Found %s.", hdGetString(HD_DEVICE_MODEL_TYPE));

  hdEnable(HD_FORCE_OUTPUT);
  hdStartScheduler();

  if (HD_DEVICE_ERROR(error = hdGetError())) {
    RCLCPP_ERROR(rclcpp::get_logger("omni_haptic_node"), "Failed to start the scheduler.");
    return -1;
  }

  // === キャリブレーション ===
  HHD_Auto_Calibration();

  // === Omni ROS ノード起動 ===
  OmniState state;
  auto node = std::make_shared<PhantomROS>(&state);

  // デバイス状態取得の非同期スケジューリング
  hdScheduleAsynchronous(omni_state_callback, &state, HD_MAX_SCHEDULER_PRIORITY);

  // === ROS 2 スピン（タイマー内で状態発行）===
  rclcpp::spin(node);

  // === 終了処理 ===
  RCLCPP_INFO(rclcpp::get_logger("omni_haptic_node"), "Ending session...");
  hdStopScheduler();
  hdDisableDevice(hHD);
  rclcpp::shutdown();

  return 0;
}
