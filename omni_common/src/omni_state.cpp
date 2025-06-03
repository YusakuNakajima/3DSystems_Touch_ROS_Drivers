#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <urdf/model.h>

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <sstream>


#include <HL/hl.h>
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h>
#include <HDU/hduQuaternion.h>
#define BT_EULER_DEFAULT_ZYX
#include <bullet/LinearMath/btMatrix3x3.h>

#include "omni_msgs/msg/omni_button_event.hpp"
#include "omni_msgs/msg/omni_feedback.hpp"
#include "omni_msgs/msg/omni_state.hpp"



int calibrationStyle;

struct OmniState {
  hduVector3Dd position;  
  hduVector3Dd velocity;   
  hduVector3Dd inp_vel1;   
  hduVector3Dd inp_vel2;  
  hduVector3Dd inp_vel3;    
  hduVector3Dd out_vel1;    
  hduVector3Dd out_vel2;    
  hduVector3Dd out_vel3;    
  hduVector3Dd pos_hist1;    
  hduVector3Dd pos_hist2;   
  hduQuaternion rot;
  hduVector3Dd joints;       
  hduVector3Dd force;       
  float thetas[7];           
  int buttons[2];         
  int buttons_prev[2];     
  bool lock;               
  bool close_gripper;       
  hduVector3Dd lock_pos;     
  double units_ratio;      
};


//TODO
class PhantomROS : public rclcpp::Node {
private:
  std::shared_ptr<OmniState> state;
  rclcpp::Publisher<omni_msgs::msg::OmniButtonEvent>::SharedPtr button_pub;
  rclcpp::Publisher<omni_msgs::msg::OmniState>::SharedPtr device_state_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr device_pose_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tip_5axis_pose_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tip_6axis_pose_pub;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub;
  rclcpp::Subscription<omni_msgs::msg::OmniFeedback>::SharedPtr force_sub;
  KDL::Chain kdl_chain_tip, kdl_chain_stylus;
  std::string prefix, reference_frame, units, robot_description;

  void force_callback(const omni_msgs::msg::OmniFeedback::SharedPtr omnifeed) {
    state->force[0] = omnifeed->force.x - 0.001 * state->velocity[0];
    state->force[1] = omnifeed->force.y - 0.001 * state->velocity[1];
    state->force[2] = omnifeed->force.z - 0.001 * state->velocity[2];

    state->lock_pos[0] = omnifeed->position.x;
    state->lock_pos[1] = omnifeed->position.y;
    state->lock_pos[2] = omnifeed->position.z;
  }

public:
  PhantomROS(std::shared_ptr<OmniState> s) : Node("phantom_ros"), state(s) {}

  void init(std::shared_ptr<OmniState> s){
    this->declare_parameter("prefix", "phantom");
    this->declare_parameter("reference_frame", "base");
    this->declare_parameter("units", "mm");
    this->declare_parameter("robot_description_name", "robot_description");

    prefix = this->get_parameter("prefix").as_string();
    reference_frame = this->get_parameter("reference_frame").as_string();
    units = this->get_parameter("units").as_string();
    robot_description = this->get_parameter("robot_description_name").as_string();

    button_pub = this->create_publisher<omni_msgs::msg::OmniButtonEvent>("button_event", 10);
    device_state_pub = this->create_publisher<omni_msgs::msg::OmniState>("device_state", 10);
    device_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("device_pose", 10);
    tip_5axis_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("tip_5axis_pose", 10);
    tip_6axis_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("tip_6axis_pose", 10);
    joint_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    force_sub = this->create_subscription<omni_msgs::msg::OmniFeedback>(
        "force_feedback", 10, std::bind(&PhantomROS::force_callback, this, std::placeholders::_1));

    std::string robot_description_content;
    if (!this->get_parameter(robot_description, robot_description_content)) {
      RCLCPP_ERROR(this->get_logger(), "Parameter [%s] not found", robot_description.c_str());
      return;
    } else{
      RCLCPP_INFO(this->get_logger(), "Parameter [%s] found", robot_description.c_str());
    }


    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromString(robot_description_content, kdl_tree)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF into KDL tree");
      return;
    }
    if (!kdl_tree.getChain("base", "tip", kdl_chain_tip)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get KDL chain from base to tip");
      return;
    }
    if (!kdl_tree.getChain("base", "stylus", kdl_chain_stylus)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get KDL chain from base to stylus");
      return;
    }


    state = s;
    state->buttons[0] = 0;
    state->buttons[1] = 0;
    state->buttons_prev[0] = 0;
    state->buttons_prev[1] = 0;
    hduVector3Dd zeros(0, 0, 0);
    state->velocity = zeros;
    state->inp_vel1 = zeros;  
    state->inp_vel2 = zeros; 
    state->inp_vel3 = zeros; 
    state->out_vel1 = zeros;  
    state->out_vel2 = zeros; 
    state->out_vel3 = zeros;  
    state->pos_hist1 = zeros; 
    state->pos_hist2 = zeros;
    state->lock = false;
    state->close_gripper = false;
    state->lock_pos = zeros;
    if (!units.compare("mm"))
      state->units_ratio = 1.0;
    else if (!units.compare("cm"))
      state->units_ratio = 10.0;
    else if (!units.compare("dm"))
      state->units_ratio = 100.0;
    else if (!units.compare("m"))
      state->units_ratio = 1000.0;
    else
    {
      state->units_ratio = 1.0;
      RCLCPP_WARN(this->get_logger(), "Unknown units [%s] unsing [mm]", units.c_str());
      units = "mm";
    }
    RCLCPP_INFO(this->get_logger(), "PHaNTOM position given in [%s], ratio [%.1f]", units.c_str(), state->units_ratio);
  }



  void publish_omni_state() {
    omni_msgs::msg::OmniState state_msg;

    state_msg.locked = state->lock;
    state_msg.close_gripper = state->close_gripper;

    state_msg.pose.position.x = state->position[0];
    state_msg.pose.position.y = state->position[1];
    state_msg.pose.position.z = state->position[2];

    state_msg.pose.orientation.x = state->rot.v()[0];
    state_msg.pose.orientation.y = state->rot.v()[1];
    state_msg.pose.orientation.z = state->rot.v()[2];
    state_msg.pose.orientation.w = state->rot.s();

    state_msg.velocity.x = state->velocity[0];
    state_msg.velocity.y = state->velocity[1];
    state_msg.velocity.z = state->velocity[2];

    state_msg.header.stamp = now();
    state_msg.header.frame_id = reference_frame;
    device_state_pub->publish(state_msg);

    
    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = now();
    joint_state.name.resize(6);
    joint_state.position.resize(6);
    joint_state.name[0] = "waist";
    joint_state.position[0] = -state->thetas[1];
    joint_state.name[1] = "shoulder";
    joint_state.position[1] = state->thetas[2];
    joint_state.name[2] = "elbow";
    joint_state.position[2] = state->thetas[3];
    joint_state.name[3] = "yaw";
    joint_state.position[3] = -state->thetas[4] + M_PI;
    joint_state.name[4] = "pitch";
    joint_state.position[4] = -state->thetas[5] - 3*M_PI/4;
    joint_state.name[5] = "roll";
    joint_state.position[5] = state->thetas[6] - M_PI;
    joint_pub->publish(joint_state);


    // Publish the tip pose by forward kinematics
    KDL::ChainFkSolverPos_recursive fk_solver_tip(kdl_chain_tip);
    KDL::JntArray q_tip(kdl_chain_tip.getNrOfSegments());
    for (size_t i = 0; i <kdl_chain_tip.getNrOfSegments(); ++i) {
        q_tip(i) = joint_state.position[i];
    }
    KDL::Frame tip_frame;
    if (fk_solver_tip.JntToCart(q_tip, tip_frame) >= 0) {
        // ROS_INFO("End Effector Position: x=%.2f, y=%.2f, z=%.2f",
        //          tip_frame.p.x(), tip_frame.p.y(), tip_frame.p.z());
        double roll, pitch, yaw;
        tip_frame.M.GetRPY(roll, pitch, yaw);
        // ROS_INFO("End Effector Orientation: roll=%.2f, pitch=%.2f, yaw=%.2f",
        //          roll, pitch, yaw);
        geometry_msgs::msg::PoseStamped tip_pose_msg;
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(q);
      
        tip_pose_msg.header = state_msg.header;
        tip_pose_msg.header.frame_id = reference_frame;
        tip_pose_msg.pose = state_msg.pose;
        tip_pose_msg.pose.position.x = tip_frame.p.x();
        tip_pose_msg.pose.position.y = tip_frame.p.y();
        tip_pose_msg.pose.position.z = tip_frame.p.z();
        tip_pose_msg.pose.orientation = quat_msg;
        tip_5axis_pose_pub->publish(tip_pose_msg);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to compute forward kinematics for the tip frame.");
    }


    // Publish the stylus pose by forward kinematics
    KDL::ChainFkSolverPos_recursive fk_solver_stylus(kdl_chain_stylus);
    KDL::JntArray q_stylus(kdl_chain_stylus.getNrOfSegments());
    for (size_t i = 0; i <kdl_chain_stylus.getNrOfSegments(); ++i) {
        q_stylus(i) = joint_state.position[i];
    }
    KDL::Frame stylus_frame;
    if (fk_solver_stylus.JntToCart(q_stylus, stylus_frame) >= 0) {
        // ROS_INFO("End Effector Position: x=%.2f, y=%.2f, z=%.2f",
        //          stylus_frame.p.x(), stylus_frame.p.y(), stylus_frame.p.z());
        double roll, pitch, yaw;
        stylus_frame.M.GetRPY(roll, pitch, yaw);
        // ROS_INFO("End Effector Orientation: roll=%.2f, pitch=%.2f, yaw=%.2f",
        //          roll, pitch, yaw);
        geometry_msgs::msg::PoseStamped stylus_pose_msg;
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(q);
        stylus_pose_msg.header = state_msg.header;
        stylus_pose_msg.header.frame_id = reference_frame;
        stylus_pose_msg.pose = state_msg.pose;
        stylus_pose_msg.pose.position.x = stylus_frame.p.x();
        stylus_pose_msg.pose.position.y = stylus_frame.p.y();
        stylus_pose_msg.pose.position.z = stylus_frame.p.z();
        stylus_pose_msg.pose.orientation = quat_msg;
        tip_6axis_pose_pub->publish(stylus_pose_msg);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to compute forward kinematics for the stylus frame.");
    }



    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = now();
    pose_msg.header.frame_id = reference_frame;
    pose_msg.pose = state_msg.pose;
    pose_msg.pose.position.x /= 1000.0; 
    pose_msg.pose.position.y /= 1000.0;
    pose_msg.pose.position.z /= 1000.0;
    device_pose_pub->publish(pose_msg);

    if ((state->buttons[0] != state->buttons_prev[0])
        or (state->buttons[1] != state->buttons_prev[1]))
    {
      if (state->buttons[0] == 1) {
        state->close_gripper = !(state->close_gripper);
      }
      if (state->buttons[1] == 1) {
        state->lock = !(state->lock);
      }
      omni_msgs::msg::OmniButtonEvent button_event;
      button_event.grey_button = state->buttons[0];
      button_event.white_button = state->buttons[1];
      state->buttons_prev[0] = state->buttons[0];
      state->buttons_prev[1] = state->buttons[1];
      button_pub->publish(button_event);
    }
  }  
};

HDCallbackCode HDCALLBACK omni_state_callback(void *pUserData)
{
  OmniState *omni_state = static_cast<OmniState *>(pUserData);
  if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE) {
    RCLCPP_DEBUG(rclcpp::get_logger("omni_state"),"Updating calibration...");
      hdUpdateCalibration(calibrationStyle);
    }
  hdBeginFrame(hdGetCurrentDevice());
  // Get transform and angles
  hduMatrix transform;
  hduVector3Dd position;
  hdGetDoublev(HD_CURRENT_TRANSFORM, transform);
  hdGetDoublev(HD_CURRENT_JOINT_ANGLES, omni_state->joints);
  hdGetDoublev(HD_CURRENT_POSITION, position);
  hduVector3Dd gimbal_angles;
  hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, gimbal_angles);
  // Notice that we are inverting the Z-position value and changing Y <---> Z
  // Position
  omni_state->position = hduVector3Dd(transform[3][0], -transform[3][2], transform[3][1]);
  // omni_state->position = position
  omni_state->position /= omni_state->units_ratio;
  // Orientation (quaternion)
  hduMatrix rotation(transform);
  rotation.getRotationMatrix(rotation);
  hduMatrix rotation_offset( 0.0, -1.0, 0.0, 0.0,
                             1.0,  0.0, 0.0, 0.0,
                             0.0,  0.0, 1.0, 0.0,
                             0.0,  0.0, 0.0, 1.0);
  rotation_offset.getRotationMatrix(rotation_offset);
  omni_state->rot = hduQuaternion(rotation_offset * rotation);
  // Velocity estimation
  hduVector3Dd vel_buff(0, 0, 0);
  vel_buff = (omni_state->position * 3 - 4 * omni_state->pos_hist1
      + omni_state->pos_hist2) / 0.002;  //(units)/s, 2nd order backward dif
  omni_state->velocity = (.2196 * (vel_buff + omni_state->inp_vel3)
      + .6588 * (omni_state->inp_vel1 + omni_state->inp_vel2)) / 1000.0
      - (-2.7488 * omni_state->out_vel1 + 2.5282 * omni_state->out_vel2
          - 0.7776 * omni_state->out_vel3);  //cutoff freq of 20 Hz
  omni_state->pos_hist2 = omni_state->pos_hist1;
  omni_state->pos_hist1 = omni_state->position;
  omni_state->inp_vel3 = omni_state->inp_vel2;
  omni_state->inp_vel2 = omni_state->inp_vel1;
  omni_state->inp_vel1 = vel_buff;
  omni_state->out_vel3 = omni_state->out_vel2;
  omni_state->out_vel2 = omni_state->out_vel1;
  omni_state->out_vel1 = omni_state->velocity;

  //~ // Set forces if locked
  //~ if (omni_state->lock == true) {
    //~ omni_state->force = 0.04 * omni_state->units_ratio * (omni_state->lock_pos - omni_state->position)
        //~ - 0.001 * omni_state->velocity;
  //~ }
  hduVector3Dd feedback;
  // Notice that we are changing Y <---> Z and inverting the Z-force_feedback
  feedback[0] = omni_state->force[0];
  feedback[1] = omni_state->force[2];
  feedback[2] = -omni_state->force[1];
  hdSetDoublev(HD_CURRENT_FORCE, feedback);

  //Get buttons
  int nButtons = 0;
  hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
  omni_state->buttons[0] = (nButtons & HD_DEVICE_BUTTON_1) ? 1 : 0;
  omni_state->buttons[1] = (nButtons & HD_DEVICE_BUTTON_2) ? 1 : 0;

  hdEndFrame(hdGetCurrentDevice());

  HDErrorInfo error;
  if (HD_DEVICE_ERROR(error = hdGetError())) {
    hduPrintError(stderr, &error, "Error during main scheduler callback");
    if (hduIsSchedulerError(&error))
      return HD_CALLBACK_DONE;
  }

  float t[7] = { 0., omni_state->joints[0], omni_state->joints[1],
      omni_state->joints[2] - omni_state->joints[1], gimbal_angles[0],
      gimbal_angles[1], gimbal_angles[2] };
  for (int i = 0; i < 7; i++)
    omni_state->thetas[i] = t[i];
  return HD_CALLBACK_CONTINUE;
}

void HHD_Auto_Calibration() {
  int supportedCalibrationStyles;
  HDErrorInfo error;

  hdGetIntegerv(HD_CALIBRATION_STYLE, &supportedCalibrationStyles);
  if (supportedCalibrationStyles & HD_CALIBRATION_ENCODER_RESET) {
    calibrationStyle = HD_CALIBRATION_ENCODER_RESET;
    RCLCPP_INFO(rclcpp::get_logger("omni_state"), "HD_CALIBRATION_ENCODER_RESET..");
  }
  if (supportedCalibrationStyles & HD_CALIBRATION_INKWELL) {
    calibrationStyle = HD_CALIBRATION_INKWELL;
    RCLCPP_INFO(rclcpp::get_logger("omni_state"), "HD_CALIBRATION_INKWELL..");
  }
  if (supportedCalibrationStyles & HD_CALIBRATION_AUTO) {
    calibrationStyle = HD_CALIBRATION_AUTO;
    RCLCPP_INFO(rclcpp::get_logger("omni_state"), "HD_CALIBRATION_AUTO..");
  }
  if (calibrationStyle == HD_CALIBRATION_ENCODER_RESET) {
    do {
      hdUpdateCalibration(calibrationStyle);
      RCLCPP_INFO(rclcpp::get_logger("omni_state"), "Calibrating.. (put stylus in well)");
      if (HD_DEVICE_ERROR(error = hdGetError())) {
        hduPrintError(stderr, &error, "Reset encoders reset failed.");
        break;
      }
    } while (hdCheckCalibration() != HD_CALIBRATION_OK);
    RCLCPP_INFO(rclcpp::get_logger("omni_state"), "Calibration complete.");
  }
  while(hdCheckCalibration() != HD_CALIBRATION_OK) {
    usleep(1e6);
    if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_MANUAL_INPUT)
      RCLCPP_INFO(rclcpp::get_logger("omni_state"), "Please place the device into the inkwell for calibration");
    else if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE) {
      RCLCPP_INFO(rclcpp::get_logger("omni_state"), "Calibration updated successfully");
      hdUpdateCalibration(calibrationStyle);
    }
    else
      RCLCPP_FATAL(rclcpp::get_logger("omni_state"), "Unknown calibration status");
  }
}



void* ros_publish(void* ptr) {
  auto* omni_ros = static_cast<PhantomROS*>(ptr);
  omni_ros->declare_parameter("publish_rate", 1000);
  int publish_rate = omni_ros->get_parameter("publish_rate").as_int();
  RCLCPP_INFO(omni_ros->get_logger(), "Publishing PHaNTOM state at [%d] Hz", publish_rate);
  rclcpp::Rate loop_rate(publish_rate);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(omni_ros->shared_from_this());

  std::thread spin_thread([&executor]() {
    executor.spin();
  });

  while (rclcpp::ok()) {
    omni_ros->publish_omni_state();
    loop_rate.sleep();
  }

  executor.cancel();
  spin_thread.join();
  return nullptr;
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto state = std::make_shared<OmniState>();
  auto omni_ros = std::make_shared<PhantomROS>(state);

  HDErrorInfo error;
  HHD hHD;

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

  HHD_Auto_Calibration();


  hdScheduleAsynchronous(omni_state_callback, static_cast<void*>(state.get()), HD_MAX_SCHEDULER_PRIORITY);




  pthread_t publish_thread;
  pthread_create(&publish_thread, nullptr, ros_publish, omni_ros.get());

  pthread_join(publish_thread, nullptr);

  RCLCPP_INFO(omni_ros->get_logger(), "Ending Phantom session...");
  hdStopScheduler();
  hdDisableDevice(hHD);

  rclcpp::shutdown();
  return 0;
}
