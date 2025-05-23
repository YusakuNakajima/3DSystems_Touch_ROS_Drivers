// === ROS 2 Core ===
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

// === Phantom Omni SDK (OpenHaptics) ===
#include <HD/hd.h>
#include <HL/hl.h>
#include <HDU/hduVector.h>
#include <HDU/hduError.h>
#include <HDU/hduMatrix.h>
#include <HDU/hduQuaternion.h>

// === Custom Message (Assuming migrated to ROS 2) ===
#include "omni_msgs/msg/omni_button_event.hpp"
#include "omni_msgs/msg/omni_feedback.hpp"
#include "omni_msgs/msg/omni_state.hpp"

// === URDF / KDL (same as ROS 1) ===
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

// === Standard C++ ===
#include <string>
#include <memory>
#include <cmath>
#include <sstream>
#include <thread>
#include <chrono>
#include <vector>
#include <pthread.h>

// === Bullet (for Euler conversions if needed) ===
#define BT_EULER_DEFAULT_ZYX
#include <bullet/LinearMath/btMatrix3x3.h>

int calibrationStyle = HD_CALIBRATION_AUTO;


// Phantom Omniの状態を保持する構造体（ROS 2でもそのまま使えます）
struct OmniState {
  // 位置・速度・過去の位置・速度
  hduVector3Dd position;     // 現在位置 (mm)
  hduVector3Dd velocity;     // 現在速度 (mm/s)
  hduVector3Dd inp_vel1;     // 過去の速度入力1
  hduVector3Dd inp_vel2;     // 過去の速度入力2
  hduVector3Dd inp_vel3;     // 過去の速度入力3
  hduVector3Dd out_vel1;     // フィルタ出力速度1
  hduVector3Dd out_vel2;     // フィルタ出力速度2
  hduVector3Dd out_vel3;     // フィルタ出力速度3
  hduVector3Dd pos_hist1;    // 過去の位置1
  hduVector3Dd pos_hist2;    // 過去の位置2

  // 回転（クォータニオン）
  hduQuaternion rot;

  // ジョイント角度と力
  hduVector3Dd joints;       // 関節角
  hduVector3Dd force;        // 力フィードバックベクトル

  float thetas[7];           // 計算されたジョイント角（7自由度想定）

  int buttons[2];            // 現在のボタン状態
  int buttons_prev[2];       // 過去のボタン状態

  bool lock;                 // ロック状態（位置を固定するか）
  bool close_gripper;        // グリッパーを閉じるフラグ
  hduVector3Dd lock_pos;     // ロック時の位置

  double units_ratio;        // 単位変換比（例：1mm = 1.0, 1m = 1000.0）
};

class PhantomROS : public rclcpp::Node {
public:
  explicit PhantomROS(std::shared_ptr<OmniState> state)
  : Node("phantom_ros"), state_(state) {
    declare_parameter<std::string>("prefix", "phantom");
    declare_parameter<std::string>("reference_frame", "base");
    declare_parameter<std::string>("units", "mm");
    declare_parameter<std::string>("robot_description_name", "robot_description");

    prefix_ = get_parameter("prefix").as_string();
    reference_frame_ = get_parameter("reference_frame").as_string();
    units_ = get_parameter("units").as_string();
    robot_description_name_ = get_parameter("robot_description_name").as_string();

    initialize_state();

    button_pub_ = create_publisher<omni_msgs::msg::OmniButtonEvent>(prefix_ + "/button", 10);
    state_pub_ = create_publisher<omni_msgs::msg::OmniState>(prefix_ + "/state", 10);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(prefix_ + "/pose", 10);
    tip_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(prefix_ + "/tip_pose", 10);
    stylus_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(prefix_ + "/stylus_pose", 10);
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>(prefix_ + "/joint_states", 10);

    force_sub_ = create_subscription<omni_msgs::msg::OmniFeedback>(
      prefix_ + "/force_feedback", 10,
      std::bind(&PhantomROS::force_callback, this, std::placeholders::_1));

    if (!get_parameter(robot_description_name_, robot_description_)) {
      RCLCPP_ERROR(get_logger(), "Failed to get robot_description parameter.");
      return;
    }

    if (!kdl_parser::treeFromString(robot_description_, kdl_tree_)) {
      RCLCPP_ERROR(get_logger(), "Failed to parse URDF into KDL tree.");
      return;
    }

    if (!kdl_tree_.getChain(prefix_ + "_base", prefix_ + "_tip", kdl_chain_tip_)) {
      RCLCPP_ERROR(get_logger(), "Failed to get KDL chain (tip).");
      return;
    }

    if (!kdl_tree_.getChain(prefix_ + "_base", prefix_ + "_stylus", kdl_chain_stylus_)) {
      RCLCPP_ERROR(get_logger(), "Failed to get KDL chain (stylus).");
      return;
    }

    RCLCPP_INFO(get_logger(), "PhantomROS initialized with prefix '%s'", prefix_.c_str());
  }

  std::shared_ptr<OmniState> get_state() { return state_; }
  rclcpp::Publisher<omni_msgs::msg::OmniState>::SharedPtr get_state_publisher() { return state_pub_; }
  const KDL::Chain& get_tip_chain() const { return kdl_chain_tip_; }
  const KDL::Chain& get_stylus_chain() const { return kdl_chain_stylus_; }

  void publish_omni_state() {
    omni_msgs::msg::OmniState msg;

    msg.header.stamp = now();
    msg.header.frame_id = reference_frame_;
    msg.locked = state_->lock;
    msg.close_gripper = state_->close_gripper;

    msg.pose.position.x = state_->position[0];
    msg.pose.position.y = state_->position[1];
    msg.pose.position.z = state_->position[2];

    // 回転クォータニオン（OpenHapticsのhduQuaternion → tf2変換が必要なら追加）
    // 省略：msg.pose.orientation = ...

    msg.velocity.x = state_->velocity[0];
    msg.velocity.y = state_->velocity[1];
    msg.velocity.z = state_->velocity[2];

    msg.current.x = state_->force[0];
    msg.current.y = state_->force[1];
    msg.current.z = state_->force[2];

    state_pub_->publish(msg);
  }

private:
  std::shared_ptr<OmniState> state_;
  std::string prefix_, reference_frame_, units_, robot_description_name_;
  std::string robot_description_;

  rclcpp::Publisher<omni_msgs::msg::OmniButtonEvent>::SharedPtr button_pub_;
  rclcpp::Publisher<omni_msgs::msg::OmniState>::SharedPtr state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tip_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr stylus_pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Subscription<omni_msgs::msg::OmniFeedback>::SharedPtr force_sub_;

  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_tip_;
  KDL::Chain kdl_chain_stylus_;

  void initialize_state() {
    hduVector3Dd zero(0.0, 0.0, 0.0);
    state_->velocity = state_->inp_vel1 = state_->inp_vel2 = state_->inp_vel3 = zero;
    state_->out_vel1 = state_->out_vel2 = state_->out_vel3 = zero;
    state_->pos_hist1 = state_->pos_hist2 = zero;
    state_->lock_pos = zero;
    state_->lock = false;
    state_->close_gripper = false;
    state_->buttons[0] = state_->buttons[1] = 0;
    state_->buttons_prev[0] = state_->buttons_prev[1] = 0;

    if (units_ == "mm") state_->units_ratio = 1.0;
    else if (units_ == "cm") state_->units_ratio = 10.0;
    else if (units_ == "dm") state_->units_ratio = 100.0;
    else if (units_ == "m") state_->units_ratio = 1000.0;
    else {
      RCLCPP_WARN(get_logger(), "Unknown units '%s', defaulting to mm", units_.c_str());
      state_->units_ratio = 1.0;
    }
  }

  void force_callback(const omni_msgs::msg::OmniFeedback::SharedPtr msg) {
    state_->force[0] = msg->force.x - 0.001 * state_->velocity[0];
    state_->force[1] = msg->force.y - 0.001 * state_->velocity[1];
    state_->force[2] = msg->force.z - 0.001 * state_->velocity[2];

    state_->lock_pos[0] = msg->position.x;
    state_->lock_pos[1] = msg->position.y;
    state_->lock_pos[2] = msg->position.z;
  }
};

HDCallbackCode HDCALLBACK omni_state_callback(void *pUserData)
{
  auto* omni_state = static_cast<OmniState *>(pUserData);

  // キャリブレーションが必要な場合は更新
  if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE) {
    hdUpdateCalibration(calibrationStyle);
  }

  hdBeginFrame(hdGetCurrentDevice());

  // 各種デバイス情報の取得
  hduMatrix transform;
  hdGetDoublev(HD_CURRENT_TRANSFORM, transform);
  hdGetDoublev(HD_CURRENT_JOINT_ANGLES, omni_state->joints);
  hduVector3Dd gimbal_angles;
  hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, gimbal_angles);

  // 座標変換（OpenHaptics座標系 -> ROS座標系: Y ↔ Z、Z反転）
  hduVector3Dd position(transform[3][0], -transform[3][2], transform[3][1]);
  position /= omni_state->units_ratio;
  omni_state->position = position;

  // 回転行列 → クォータニオン
  hduMatrix rotation(transform);
  rotation.getRotationMatrix(rotation);

  // 座標系調整行列（Y <-> Xスワップ）
  hduMatrix rotation_offset(
      0.0, -1.0, 0.0, 0.0,
      1.0,  0.0, 0.0, 0.0,
      0.0,  0.0, 1.0, 0.0,
      0.0,  0.0, 0.0, 1.0);

  rotation_offset.getRotationMatrix(rotation_offset);
  omni_state->rot = hduQuaternion(rotation_offset * rotation);

  // 速度の数値微分 + フィルタリング（20Hz low-pass）
  hduVector3Dd vel_buff = (omni_state->position * 3
                         - 4 * omni_state->pos_hist1
                         + omni_state->pos_hist2) / 0.002;

  hduVector3Dd filtered_vel = (.2196 * (vel_buff + omni_state->inp_vel3)
                              + .6588 * (omni_state->inp_vel1 + omni_state->inp_vel2)) / 1000.0
                              - (-2.7488 * omni_state->out_vel1
                                 + 2.5282 * omni_state->out_vel2
                                 - 0.7776 * omni_state->out_vel3);

  omni_state->velocity = filtered_vel;

  // 状態更新
  omni_state->pos_hist2 = omni_state->pos_hist1;
  omni_state->pos_hist1 = omni_state->position;

  omni_state->inp_vel3 = omni_state->inp_vel2;
  omni_state->inp_vel2 = omni_state->inp_vel1;
  omni_state->inp_vel1 = vel_buff;

  omni_state->out_vel3 = omni_state->out_vel2;
  omni_state->out_vel2 = omni_state->out_vel1;
  omni_state->out_vel1 = filtered_vel;

  // 力フィードバック（Z軸反転・YZスワップ）
  hduVector3Dd feedback;
  feedback[0] = omni_state->force[0];
  feedback[1] = omni_state->force[2];
  feedback[2] = -omni_state->force[1];
  hdSetDoublev(HD_CURRENT_FORCE, feedback);

  // ボタン入力
  int nButtons = 0;
  hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
  omni_state->buttons[0] = (nButtons & HD_DEVICE_BUTTON_1) ? 1 : 0;
  omni_state->buttons[1] = (nButtons & HD_DEVICE_BUTTON_2) ? 1 : 0;

  hdEndFrame(hdGetCurrentDevice());

  // エラーチェック
  HDErrorInfo error;
  if (HD_DEVICE_ERROR(error = hdGetError())) {
    hduPrintError(stderr, &error, "Error during haptics callback");
    if (hduIsSchedulerError(&error)) {
      return HD_CALLBACK_DONE;
    }
  }

  // ジョイント角変換（theta[1]~theta[6] へのマッピング）
  float t[7] = {
    0.0,
    omni_state->joints[0],
    omni_state->joints[1],
    omni_state->joints[2] - omni_state->joints[1],
    gimbal_angles[0],
    gimbal_angles[1],
    gimbal_angles[2]
  };
  for (int i = 0; i < 7; ++i) {
    omni_state->thetas[i] = t[i];
  }

  return HD_CALLBACK_CONTINUE;
}


void HHD_Auto_Calibration(const rclcpp::Logger& logger)
{
  int supportedCalibrationStyles = 0;
  HDErrorInfo error;

  hdGetIntegerv(HD_CALIBRATION_STYLE, &supportedCalibrationStyles);

  if (supportedCalibrationStyles & HD_CALIBRATION_ENCODER_RESET) {
    calibrationStyle = HD_CALIBRATION_ENCODER_RESET;
    RCLCPP_INFO(logger, "Using HD_CALIBRATION_ENCODER_RESET");
  }
  if (supportedCalibrationStyles & HD_CALIBRATION_INKWELL) {
    calibrationStyle = HD_CALIBRATION_INKWELL;
    RCLCPP_INFO(logger, "Using HD_CALIBRATION_INKWELL");
  }
  if (supportedCalibrationStyles & HD_CALIBRATION_AUTO) {
    calibrationStyle = HD_CALIBRATION_AUTO;
    RCLCPP_INFO(logger, "Using HD_CALIBRATION_AUTO");
  }

  if (calibrationStyle == HD_CALIBRATION_ENCODER_RESET) {
    do {
      hdUpdateCalibration(calibrationStyle);
      RCLCPP_INFO(logger, "Calibrating... (place stylus in well)");
      if (HD_DEVICE_ERROR(error = hdGetError())) {
        hduPrintError(stderr, &error, "Reset encoders failed.");
        break;
      }
    } while (hdCheckCalibration() != HD_CALIBRATION_OK);

    RCLCPP_INFO(logger, "Calibration complete.");
  }

  // その他キャリブレーション状態の確認ループ
  while (hdCheckCalibration() != HD_CALIBRATION_OK) {
    usleep(1e6);  // 1秒待機

    int status = hdCheckCalibration();
    if (status == HD_CALIBRATION_NEEDS_MANUAL_INPUT) {
      RCLCPP_INFO(logger, "Please place the device into the inkwell for calibration");
    } else if (status == HD_CALIBRATION_NEEDS_UPDATE) {
      RCLCPP_INFO(logger, "Calibration updated successfully");
      hdUpdateCalibration(calibrationStyle);
    } else {
      RCLCPP_FATAL(logger, "Unknown calibration status");
      break;
    }
  }
}

void* ros_publish(void* ptr) {
  auto* omni_ros = static_cast<PhantomROS*>(ptr);

  // パラメータから publish_rate を取得（なければ 1000 Hz）
  int publish_rate = 1000;
  if (omni_ros->has_parameter("publish_rate")) {
    publish_rate = omni_ros->get_parameter("publish_rate").as_int();
  } else {
    omni_ros->declare_parameter("publish_rate", 1000);
  }

  RCLCPP_INFO(omni_ros->get_logger(), "Publishing PHaNTOM state at [%d] Hz", publish_rate);
  rclcpp::Rate loop_rate(publish_rate);

  // ROS 2 の executor を別スレッドで回す
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(omni_ros->shared_from_this());

  std::thread spin_thread([&executor]() {
    executor.spin();
  });

  // メインループ：OmniState の publish を継続
  while (rclcpp::ok()) {
    omni_ros->publish_omni_state();
    loop_rate.sleep();
  }

  // シャットダウン処理
  executor.cancel();
  spin_thread.join();
  return nullptr;
}


int main(int argc, char** argv)
{
  // =======================
  // 1. ROS 2初期化
  // =======================
  rclcpp::init(argc, argv);

  // 共有状態構造体の生成
  auto state = std::make_shared<OmniState>();

  // ノードインスタンス生成（PhantomROS は shared_from_this() を使うので shared_ptrが必要）
  auto omni_ros = std::make_shared<PhantomROS>(state);

  // =======================
  // 2. Phantom Omni初期化
  // =======================
  HDErrorInfo error;
  HHD hHD;

  // パラメータから device_name を取得
  std::string device_name = "Default Device";
  if (omni_ros->has_parameter("device_name")) {
    device_name = omni_ros->get_parameter("device_name").as_string();
  } else {
    omni_ros->declare_parameter<std::string>("device_name", "Default Device");
  }

  HDstring target_dev = device_name.c_str();
  hHD = hdInitDevice(target_dev);

  if (HD_DEVICE_ERROR(error = hdGetError())) {
    RCLCPP_ERROR(omni_ros->get_logger(), "Failed to initialize haptic device");
    return -1;
  }

  RCLCPP_INFO(omni_ros->get_logger(), "Found haptic device: %s", hdGetString(HD_DEVICE_MODEL_TYPE));

  hdEnable(HD_FORCE_OUTPUT);
  hdStartScheduler();

  if (HD_DEVICE_ERROR(error = hdGetError())) {
    RCLCPP_ERROR(omni_ros->get_logger(), "Failed to start scheduler");
    return -1;
  }

  // =======================
  // 3. 自動キャリブレーション
  // =======================
  HHD_Auto_Calibration(omni_ros->get_logger());

  // =======================
  // 4. スケジューラ登録
  // =======================
  hdScheduleAsynchronous(omni_state_callback, static_cast<void*>(state.get()), HD_MAX_SCHEDULER_PRIORITY);

  // =======================
  // 5. ROSのパブリッシュスレッド開始
  // =======================
  pthread_t publish_thread;
  pthread_create(&publish_thread, nullptr, ros_publish, omni_ros.get());

  // メインスレッドでスレッド待機
  pthread_join(publish_thread, nullptr);

  // =======================
  // 6. 終了処理
  // =======================
  RCLCPP_INFO(omni_ros->get_logger(), "Ending Phantom session...");
  hdStopScheduler();
  hdDisableDevice(hHD);

  rclcpp::shutdown();
  return 0;
}
