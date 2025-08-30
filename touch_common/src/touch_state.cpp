#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <urdf/model.h>
#include <sensor_msgs/msg/joint_state.hpp>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <cmath>
#include <assert.h>
#include <sstream>
#include <signal.h>
#include <atomic>

#include <HL/hl.h>
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h>
#include <HDU/hduQuaternion.h>
#define BT_EULER_DEFAULT_ZYX
#include <bullet/LinearMath/btMatrix3x3.h>

#include "touch_msgs/msg/touch_button_event.hpp"
#include "touch_msgs/msg/touch_feedback.hpp"
#include "touch_msgs/msg/touch_state.hpp"
#include <pthread.h>

float prev_time;
int calibrationStyle;
KDL::Chain kdl_chain_tip, kdl_chain_stylus;

// Global shutdown flag for clean exit
std::atomic<bool> g_shutdown_requested(false);
HHD g_hHD = HD_INVALID_HANDLE;

struct TouchState {
  hduVector3Dd position;  //3x1 vector of position
  hduVector3Dd velocity;  //3x1 vector of velocity
  hduVector3Dd inp_vel1;  //3x1 history of velocity used for filtering velocity estimate
  hduVector3Dd inp_vel2;
  hduVector3Dd inp_vel3;
  hduVector3Dd out_vel1;
  hduVector3Dd out_vel2;
  hduVector3Dd out_vel3;
  hduVector3Dd pos_hist1; //3x1 history of position used for 2nd order backward difference estimate of velocity
  hduVector3Dd pos_hist2;
  hduQuaternion rot;
  hduVector3Dd joints;
  hduVector3Dd force;   //3 element double vector force[0], force[1], force[2]
  float thetas[7];
  int buttons[2];
  int buttons_prev[2];
  bool lock;
  bool close_gripper;
  hduVector3Dd lock_pos;
  double units_ratio;
};

class TouchROS : public rclcpp::Node {

public:
  rclcpp::Publisher<touch_msgs::msg::TouchState>::SharedPtr state_publisher;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tip_pose_publisher;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr stylus_pose_publisher;
  rclcpp::Publisher<touch_msgs::msg::TouchButtonEvent>::SharedPtr button_publisher;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher;
  rclcpp::Subscription<touch_msgs::msg::TouchFeedback>::SharedPtr haptic_sub;
  std::string ref_frame, units, robot_description_name;
  bool kdl_chains_initialized;

  TouchState *state;

  TouchROS() : Node("touch_haptic_node") {}

  void init(TouchState *s) {
    this->declare_parameter("reference_frame", "base");
    this->declare_parameter("units", "mm");
    this->declare_parameter("robot_description_name", "robot_description");
    this->declare_parameter("robot_description", "");

    ref_frame = this->get_parameter("reference_frame").as_string();
    units = this->get_parameter("units").as_string();
    robot_description_name = this->get_parameter("robot_description_name").as_string();

    //Publish button state on button
    button_publisher = this->create_publisher<touch_msgs::msg::TouchButtonEvent>("button", 100);

    //Publish on state
    state_publisher = this->create_publisher<touch_msgs::msg::TouchState>("state", 1);

    //Subscribe to force_feedback
    haptic_sub = this->create_subscription<touch_msgs::msg::TouchFeedback>(
        "force_feedback", 1, 
        std::bind(&TouchROS::force_callback, this, std::placeholders::_1));

    //Publish on pose
    pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 1);

    //Publish on joint_states
    joint_publisher = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);

    //Publish on tip_pose
    tip_pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("tip_pose", 1);

    //Publish on stylus_pose
    stylus_pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("stylus_pose", 1);

    // Try to get the robot description from the parameter server (optional)
    kdl_chains_initialized = false;
    if (!robot_description_name.empty()) {
        std::string robot_description_content;
        try {
            // Wait for parameter to be available
            auto parameter = this->get_parameter(robot_description_name);
            robot_description_content = parameter.as_string();
            if (robot_description_content.empty()) {
                RCLCPP_WARN(this->get_logger(), "Robot description parameter %s is empty.", robot_description_name.c_str());
            } else {
                RCLCPP_INFO(this->get_logger(), "Successfully retrieved robot description content from %s (%zu bytes).", 
                           robot_description_name.c_str(), robot_description_content.size());
            }
            
            // Parse the URDF to a KDL tree
            KDL::Tree kdl_tree;
            if (kdl_parser::treeFromString(robot_description_content, kdl_tree)) {
                // Get the chain 
                if (kdl_tree.getChain("touch_base", "touch_tip", kdl_chain_tip) &&
                    kdl_tree.getChain("touch_base", "touch_stylus", kdl_chain_stylus)) {
                    kdl_chains_initialized = true;
                    RCLCPP_INFO(this->get_logger(), "KDL chains initialized successfully.");
                } else {
                    RCLCPP_WARN(this->get_logger(), "Failed to get KDL chains from touch_base to touch_tip/touch_stylus.");
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "Failed to parse URDF to KDL tree.");
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to get robot description content: %s. Continuing without KDL chains.", e.what());
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "Robot description name is empty. Continuing without KDL chains.");
    }
    
    if (!kdl_chains_initialized) {
        RCLCPP_WARN(this->get_logger(), "KDL chains not initialized. Tip and stylus pose publishing will be disabled.");
    }

    state = s;
    state->buttons[0] = 0;
    state->buttons[1] = 0;
    state->buttons_prev[0] = 0;
    state->buttons_prev[1] = 0;
    hduVector3Dd zeros(0, 0, 0);
    state->velocity = zeros;
    state->inp_vel1 = zeros;  //3x1 history of velocity
    state->inp_vel2 = zeros;  //3x1 history of velocity
    state->inp_vel3 = zeros;  //3x1 history of velocity
    state->out_vel1 = zeros;  //3x1 history of velocity
    state->out_vel2 = zeros;  //3x1 history of velocity
    state->out_vel3 = zeros;  //3x1 history of velocity
    state->pos_hist1 = zeros; //3x1 history of position
    state->pos_hist2 = zeros; //3x1 history of position
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
      RCLCPP_WARN(this->get_logger(), "Unknown units [%s] using [mm]", units.c_str());
      units = "mm";
    }
    RCLCPP_INFO(this->get_logger(), "Touch position given in [%s], ratio [%.1f]", units.c_str(), state->units_ratio);
  }

  /*******************************************************************************
   ROS node callback.
   *******************************************************************************/
  void force_callback(const touch_msgs::msg::TouchFeedback::SharedPtr touchfeed) {
    ////////////////////Some people might not like this extra damping, but it
    ////////////////////helps to stabilize the overall force feedback. It isn't
    ////////////////////like we are getting direct impedance matching from the
    ////////////////////touch anyway
    
    // Replace NaN values with 0.0 for force
    double force_x = std::isnan(touchfeed->force.x) ? 0.0 : touchfeed->force.x;
    double force_y = std::isnan(touchfeed->force.y) ? 0.0 : touchfeed->force.y;
    double force_z = std::isnan(touchfeed->force.z) ? 0.0 : touchfeed->force.z;
    
    state->force[0] = force_x - 0.001 * state->velocity[0];
    state->force[1] = force_y - 0.001 * state->velocity[1];
    state->force[2] = force_z - 0.001 * state->velocity[2];

    // Replace NaN values with 0.0 for position
    state->lock_pos[0] = std::isnan(touchfeed->position.x) ? 0.0 : touchfeed->position.x;
    state->lock_pos[1] = std::isnan(touchfeed->position.y) ? 0.0 : touchfeed->position.y;
    state->lock_pos[2] = std::isnan(touchfeed->position.z) ? 0.0 : touchfeed->position.z;
  }

  void publish_touch_state() {
    // Build the state msg
    touch_msgs::msg::TouchState state_msg;
    // Locked
    state_msg.locked = state->lock;
    state_msg.close_gripper = state->close_gripper;
    // Position
    state_msg.pose.position.x = state->position[0];
    state_msg.pose.position.y = state->position[1];
    state_msg.pose.position.z = state->position[2];
    // Orientation
    state_msg.pose.orientation.x = state->rot.v()[0];
    state_msg.pose.orientation.y = state->rot.v()[1];
    state_msg.pose.orientation.z = state->rot.v()[2];
    state_msg.pose.orientation.w = state->rot.s();
    // Velocity
    state_msg.velocity.x = state->velocity[0];
    state_msg.velocity.y = state->velocity[1];
    state_msg.velocity.z = state->velocity[2];
    // TODO: Append Current to the state msg
    state_msg.header.stamp = this->now();
    state_publisher->publish(state_msg);

    // Publish the JointState msg
    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = this->now();
    joint_state.header.frame_id = "";  // Joint states don't need frame_id
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
    joint_publisher->publish(joint_state);

    // Publish the tip pose by forward kinematics (only if KDL chains are initialized)
    if (kdl_chains_initialized) {
        KDL::ChainFkSolverPos_recursive fk_solver_tip(kdl_chain_tip);
        KDL::JntArray q_tip(kdl_chain_tip.getNrOfSegments());
        for (size_t i = 0; i <kdl_chain_tip.getNrOfSegments(); ++i) {
            q_tip(i) = joint_state.position[i];
        }
        KDL::Frame tip_frame;
        if (fk_solver_tip.JntToCart(q_tip, tip_frame) >= 0) {
            double roll, pitch, yaw;
            tip_frame.M.GetRPY(roll, pitch, yaw);
            geometry_msgs::msg::PoseStamped tip_pose_msg;
            tip_pose_msg.header = state_msg.header;
            tip_pose_msg.header.frame_id = ref_frame;
            tip_pose_msg.pose = state_msg.pose;
            tip_pose_msg.pose.position.x = tip_frame.p.x();
            tip_pose_msg.pose.position.y = tip_frame.p.y();
            tip_pose_msg.pose.position.z = tip_frame.p.z();
            
            tf2::Quaternion quat;
            quat.setRPY(roll, pitch, yaw);
            tip_pose_msg.pose.orientation = tf2::toMsg(quat);
            tip_pose_publisher->publish(tip_pose_msg);
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
            double roll, pitch, yaw;
            stylus_frame.M.GetRPY(roll, pitch, yaw);
            geometry_msgs::msg::PoseStamped stylus_pose_msg;
            stylus_pose_msg.header = state_msg.header;
            stylus_pose_msg.header.frame_id = ref_frame;
            stylus_pose_msg.pose = state_msg.pose;
            stylus_pose_msg.pose.position.x = stylus_frame.p.x();
            stylus_pose_msg.pose.position.y = stylus_frame.p.y();
            stylus_pose_msg.pose.position.z = stylus_frame.p.z();
            
            tf2::Quaternion quat;
            quat.setRPY(roll, pitch, yaw);
            stylus_pose_msg.pose.orientation = tf2::toMsg(quat);
            stylus_pose_publisher->publish(stylus_pose_msg);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to compute forward kinematics for the stylus frame.");
        }
    }


    // Build the pose msg
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = state_msg.header;
    pose_msg.header.frame_id = ref_frame;
    pose_msg.pose = state_msg.pose;
    pose_msg.pose.position.x /= 1000.0;
    pose_msg.pose.position.y /= 1000.0;
    pose_msg.pose.position.z /= 1000.0;
    pose_publisher->publish(pose_msg);

    if ((state->buttons[0] != state->buttons_prev[0])
        or (state->buttons[1] != state->buttons_prev[1]))
    {
      if (state->buttons[0] == 1) {
        state->close_gripper = !(state->close_gripper);
      }
      if (state->buttons[1] == 1) {
        state->lock = !(state->lock);
      }
      touch_msgs::msg::TouchButtonEvent button_event;
      button_event.grey_button = state->buttons[0];
      button_event.white_button = state->buttons[1];
      state->buttons_prev[0] = state->buttons[0];
      state->buttons_prev[1] = state->buttons[1];
      button_publisher->publish(button_event);
    }
  }
};

HDCallbackCode HDCALLBACK touch_state_callback(void *pUserData)
{
  TouchState *touch_state = static_cast<TouchState *>(pUserData);
  if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE) {
    // RCLCPP_DEBUG not available outside class, using printf for now
    printf("Updating calibration...\n");
      hdUpdateCalibration(calibrationStyle);
    }
  hdBeginFrame(hdGetCurrentDevice());
  // Get transform and angles
  hduMatrix transform;
  hduVector3Dd position;
  hdGetDoublev(HD_CURRENT_TRANSFORM, transform);
  hdGetDoublev(HD_CURRENT_JOINT_ANGLES, touch_state->joints);
  hdGetDoublev(HD_CURRENT_POSITION, position);
  hduVector3Dd gimbal_angles;
  hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, gimbal_angles);
  // Notice that we are inverting the Z-position value and changing Y <---> Z
  // Position
  touch_state->position = hduVector3Dd(transform[3][0], -transform[3][2], transform[3][1]);
  // touch_state->position = position
  touch_state->position /= touch_state->units_ratio;
  // Orientation (quaternion)
  hduMatrix rotation(transform);
  rotation.getRotationMatrix(rotation);
  hduMatrix rotation_offset( 0.0, -1.0, 0.0, 0.0,
                             1.0,  0.0, 0.0, 0.0,
                             0.0,  0.0, 1.0, 0.0,
                             0.0,  0.0, 0.0, 1.0);
  rotation_offset.getRotationMatrix(rotation_offset);
  touch_state->rot = hduQuaternion(rotation_offset * rotation);
  // Velocity estimation
  hduVector3Dd vel_buff(0, 0, 0);
  vel_buff = (touch_state->position * 3 - 4 * touch_state->pos_hist1
      + touch_state->pos_hist2) / 0.002;  //(units)/s, 2nd order backward dif
  touch_state->velocity = (.2196 * (vel_buff + touch_state->inp_vel3)
      + .6588 * (touch_state->inp_vel1 + touch_state->inp_vel2)) / 1000.0
      - (-2.7488 * touch_state->out_vel1 + 2.5282 * touch_state->out_vel2
          - 0.7776 * touch_state->out_vel3);  //cutoff freq of 20 Hz
  touch_state->pos_hist2 = touch_state->pos_hist1;
  touch_state->pos_hist1 = touch_state->position;
  touch_state->inp_vel3 = touch_state->inp_vel2;
  touch_state->inp_vel2 = touch_state->inp_vel1;
  touch_state->inp_vel1 = vel_buff;
  touch_state->out_vel3 = touch_state->out_vel2;
  touch_state->out_vel2 = touch_state->out_vel1;
  touch_state->out_vel1 = touch_state->velocity;

  //~ // Set forces if locked
  //~ if (touch_state->lock == true) {
    //~ touch_state->force = 0.04 * touch_state->units_ratio * (touch_state->lock_pos - touch_state->position)
        //~ - 0.001 * touch_state->velocity;
  //~ }
  hduVector3Dd feedback;
  // Notice that we are changing Y <---> Z and inverting the Z-force_feedback
  feedback[0] = touch_state->force[0];
  feedback[1] = touch_state->force[2];
  feedback[2] = -touch_state->force[1];
  hdSetDoublev(HD_CURRENT_FORCE, feedback);

  //Get buttons
  int nButtons = 0;
  hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
  touch_state->buttons[0] = (nButtons & HD_DEVICE_BUTTON_1) ? 1 : 0;
  touch_state->buttons[1] = (nButtons & HD_DEVICE_BUTTON_2) ? 1 : 0;

  hdEndFrame(hdGetCurrentDevice());

  HDErrorInfo error;
  if (HD_DEVICE_ERROR(error = hdGetError())) {
    hduPrintError(stderr, &error, "Error during main scheduler callback");
    if (hduIsSchedulerError(&error))
      return HD_CALLBACK_DONE;
  }

  float t[7] = { 0.0f, static_cast<float>(touch_state->joints[0]), static_cast<float>(touch_state->joints[1]),
      static_cast<float>(touch_state->joints[2] - touch_state->joints[1]), static_cast<float>(gimbal_angles[0]),
      static_cast<float>(gimbal_angles[1]), static_cast<float>(gimbal_angles[2]) };
  for (int i = 0; i < 7; i++)
    touch_state->thetas[i] = t[i];
  return HD_CALLBACK_CONTINUE;
}

/*******************************************************************************
 Automatic Calibration of Touch Device - No character inputs
 *******************************************************************************/
void HHD_Auto_Calibration() {
  int supportedCalibrationStyles;
  HDErrorInfo error;

  hdGetIntegerv(HD_CALIBRATION_STYLE, &supportedCalibrationStyles);
  if (supportedCalibrationStyles & HD_CALIBRATION_ENCODER_RESET) {
    calibrationStyle = HD_CALIBRATION_ENCODER_RESET;
    printf("HD_CALIBRATION_ENCODER_RESET..\n");
  }
  if (supportedCalibrationStyles & HD_CALIBRATION_INKWELL) {
    calibrationStyle = HD_CALIBRATION_INKWELL;
    printf("HD_CALIBRATION_INKWELL..\n");
  }
  if (supportedCalibrationStyles & HD_CALIBRATION_AUTO) {
    calibrationStyle = HD_CALIBRATION_AUTO;
    printf("HD_CALIBRATION_AUTO..\n");
  }
  if (calibrationStyle == HD_CALIBRATION_ENCODER_RESET) {
    do {
      hdUpdateCalibration(calibrationStyle);
      printf("Calibrating.. (put stylus in well)\n");
      if (HD_DEVICE_ERROR(error = hdGetError())) {
        hduPrintError(stderr, &error, "Reset encoders reset failed.");
        break;
      }
    } while (hdCheckCalibration() != HD_CALIBRATION_OK);
    printf("Calibration complete.\n");
  }
  while(hdCheckCalibration() != HD_CALIBRATION_OK) {
    usleep(1e6);
    if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_MANUAL_INPUT)
      printf("Please place the device into the inkwell for calibration\n");
    else if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE) {
      printf("Calibration updated successfully\n");
      hdUpdateCalibration(calibrationStyle);
    }
    else
      printf("Unknown calibration status\n");
  }
}

// Signal handler for clean shutdown
void signal_handler(int signal) {
  if (signal == SIGINT) {
    printf("\nReceived SIGINT, shutting down gracefully...\n");
    g_shutdown_requested.store(true);
    
    // Stop ROS spinning
    rclcpp::shutdown();
  }
}

void *ros_publish(void *ptr) {
  std::shared_ptr<TouchROS> touch_ros = *static_cast<std::shared_ptr<TouchROS>*>(ptr);
  int publish_rate;
  touch_ros->declare_parameter("publish_rate", 1000);
  publish_rate = touch_ros->get_parameter("publish_rate").as_int();
  RCLCPP_INFO(touch_ros->get_logger(), "Publishing Touch state at [%d] Hz", publish_rate);
  rclcpp::Rate loop_rate(publish_rate);

  while (rclcpp::ok() && !g_shutdown_requested.load()) {
    touch_ros->publish_touch_state();
    rclcpp::spin_some(touch_ros);
    loop_rate.sleep();
  }
  return NULL;
}

// Clean shutdown function
void cleanup_resources() {
  printf("Cleaning up resources...\n");
  
  // Stop haptic device scheduler
  if (g_hHD != HD_INVALID_HANDLE) {
    hdStopScheduler();
    hdDisableDevice(g_hHD);
    printf("Haptic device stopped and disabled.\n");
  }
  
  // Shutdown ROS if not already done
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  
  printf("Cleanup completed.\n");
}

int main(int argc, char** argv) {
  ////////////////////////////////////////////////////////////////
  // Setup signal handler for clean shutdown
  ////////////////////////////////////////////////////////////////
  signal(SIGINT, signal_handler);
  
  ////////////////////////////////////////////////////////////////
  // Init ROS
  ////////////////////////////////////////////////////////////////
  rclcpp::init(argc, argv);
  TouchState state;
  auto touch_ros = std::make_shared<TouchROS>();

  ////////////////////////////////////////////////////////////////
  // Init Touch
  ////////////////////////////////////////////////////////////////
  HDErrorInfo error;
  std::string device_name;
  touch_ros->declare_parameter("device_name", "Default Device");
  device_name = touch_ros->get_parameter("device_name").as_string();
  HDstring target_dev = device_name.c_str();
  g_hHD = hdInitDevice(target_dev);
  if (HD_DEVICE_ERROR(error = hdGetError())) {
    RCLCPP_ERROR(touch_ros->get_logger(), "Failed to initialize haptic device");
    cleanup_resources();
    return -1;
  }
  RCLCPP_INFO(touch_ros->get_logger(), "Found %s.", hdGetString(HD_DEVICE_MODEL_TYPE));
  hdEnable(HD_FORCE_OUTPUT);
  hdStartScheduler();
  if (HD_DEVICE_ERROR(error = hdGetError())) {
    RCLCPP_ERROR(touch_ros->get_logger(), "Failed to start the scheduler");
    cleanup_resources();
    return -1;
  }
  HHD_Auto_Calibration();

  touch_ros->init(&state);
  hdScheduleAsynchronous(touch_state_callback, &state,
      HD_MAX_SCHEDULER_PRIORITY);

  
  ////////////////////////////////////////////////////////////////
  // Loop and publish
  ////////////////////////////////////////////////////////////////
  pthread_t publish_thread;
  pthread_create(&publish_thread, NULL, ros_publish, (void*) &touch_ros);
  
  // Wait for shutdown signal or thread completion
  pthread_join(publish_thread, NULL);

  RCLCPP_INFO(touch_ros->get_logger(), "Ending Session....");
  
  ////////////////////////////////////////////////////////////////
  // Clean shutdown
  ////////////////////////////////////////////////////////////////
  cleanup_resources();
  
  return 0;
}