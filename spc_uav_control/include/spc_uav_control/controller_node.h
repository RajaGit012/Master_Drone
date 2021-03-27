/*
 * controller_node.h
 *
 *  Created on: Mar 7, 2018
 *      Author: ramy
 *  Refactored on: May 2018
 *      Author: Asem
 *
 * The purpose of this class is to provide the commonly needed functionality
 * of a controller. This includes communicating with the GUI, converting to PX4
 * or RotorS set points and loading commonly needed parameters.
 *
 * Should NOT know anything about any specific controller.
 */

#ifndef INCLUDE_SPC_UAV_CONTROL_CONTROLLER_NODE_H_
#define INCLUDE_SPC_UAV_CONTROL_CONTROLLER_NODE_H_

#include <ros/ros.h>

#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>

#include "tf/tf.h"
#include "tf/transform_datatypes.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <iostream>
#include <string>

#include "actionlib/server/simple_action_server.h"
#include "spc_uav_comm/LearnParamAction.h"

//TODO: Move all these definitions in one header file
#define FREEFLIGHTMODE 0.0
#define HORZCIRCLEMODE 1.0
#define VERTCIRCLEMODE 2.0
#define CUSTOMTRAJMODE 3.0

#define MODE_ON 1.0
#define MODE_OFF 2.0

#define M_PI           3.14159265358979323846

class ControllerNode {

 public:
  /**
   * Constructor
   */
  ControllerNode();
  /**
   * Destructor
   */
  ~ControllerNode() {
    delete tfl;
  }
  /**
   * Load parameters from ROS Parameter Server
   */
  void loadControllerROSParameters();
  double loadSingleROSParameter(ros::NodeHandle &, const std::string&);
  /**
   * Unpause Gazebo Simulation
   * @param
   */
  void unpauseGazebo();
  /**
   * Callback for px4 state changes
   * @param
   */
  void statusCallback(const mavros_msgs::State::ConstPtr&);
  /**
   * Callback function for messages from Aeroworks Setpoint GUI
   * @param
   */
  void guiCommanderCallback(const std_msgs::Float32MultiArray::ConstPtr&);
  /**
   * Odometry Callback FUnction
   */
  void odomCallback(const nav_msgs::Odometry::ConstPtr&);
  /**
   * Update Setpoint from TF server
   */
  void updateGUISetpoint();
  /**
   * Convert desired trajectory message to H-matrix and Twist
   * @param desired_trajectory
   */
  void convertDesiredTrajectory(mav_msgs::EigenTrajectoryPoint desired_trajectory);
  /**
   * Convert actual odometry message to H-matrix and Twist
   * @param odometry
   */
  void convertOdometry(mav_msgs::EigenOdometry odometry);
  /**
   * Compute the desired trajectory from the setpoint frame
   */
  bool computeDesiredTrajectory(tf2_ros::Buffer& tfBuffer, mav_msgs::EigenTrajectoryPoint& command_trajectory);
  /**
   * Goal Callback for Learning Action Server
   */
  void learningGoalCB();
  /**
   * Preemt (Cancel) Callback for Learning Action Server
   */
  void learningPreemptCB();
  /**
   * Command trajectory to draw a circle once
   */
  void customCircle(mav_msgs::EigenTrajectoryPoint& desired_trajectory);
  /**
   * Command trajectory to draw a square once
   */
  void customSquare(mav_msgs::EigenTrajectoryPoint& desired_trajectory);
  /**
   *  Function that initializes the circle or square trajectory at the start configuration
   */
  void initCustomTrajectory(mav_msgs::EigenTrajectoryPoint& desired_trajectory);
  /**
   * Function that ends a custom trajectory called at the end of action
   */
  void finishCustomTrajectory();
  /**
   * Function that sets the desired start configuration of a custom trajectory
   */
  void captureDesiredStartConfiguration();
/**
 * Publish Custom Desired Trajectory used in an episode for learning
 * @param desired_trajectory
 */
  void publishCustomDesiredTrajectory(mav_msgs::EigenTrajectoryPoint& desired_trajectory);

  /**
   * Adapte the odometry message for the interface used (SITL, ROTORS, or Experiment)
   * @param odometry
   */
  void checkOdometryMessage(mav_msgs::EigenOdometry& odometry);
  /**
   * compute the elapsed sampling time in seconds
   * @return
   */
  double computeElapsedTime();
  /**
   * Used to interface controller with a px4 firmware through mavros
   * TODO: SHOULD BE MODIFIED TO INTERFACE TO NEW MC_ATT_CONTROL MODULE
   * @param command_wrench: control thrust force and torques for underactuated UAVs
   * @param px4_setpoints: normalized thrust and desired angular rates
   */
  void transformWrenchToAngRateSetPoints(Eigen::VectorXd& command_wrench, Eigen::VectorXd* px4_setpoints) const;
  /**
   * compute the control allocation matrix, mapping command wrench to rotor velocities squared
   * @param airframeType
   */
  void computeControlAllocation(std::string& airframeType);
  /**
   *
   * @param command_wrench: control thrust force and torques for underactuated UAVs
   * @param rotor_velocities: individual rotor angular velocities in [rad/s]
   */
  void calculateRotorVelocities(Eigen::VectorXd& command_wrench, Eigen::VectorXd* rotor_velocities,
                                std::string& airframeType) const;

  /**
   * publish the px4 setpoints
   * @param px4_setpoints
   */
  void publishPX4setpoints(Eigen::VectorXd& px4_setpoints);
  /**
   * publish the rotorS setpoints
   * @param ref_rotor_velocities
   */
  void publishRotorSsetpoints(Eigen::VectorXd& ref_rotor_velocities);
  /**
   * Publishes set points either to rotorS or PX4
   */
  void publishSetPoints(Eigen::VectorXd& command_wrench);
  /**
   * publish the command wrench
   * TODO: add to these additional internal signals of interest e.g., disturbance observer estimate, ...
   * @param command_wrench
   */
  void publishControlSignals(Eigen::VectorXd& command_wrench);
  /**
   * Set the command limits used for normalizing command wrench sent to px4
   * @param airframeType
   */
  void setCmdLimits(std::string& airframeType);

  /***************************************************************************************************************************************/
  /***************************************************** Setters ***************************************************************/

  void setPx4Gains(const Eigen::Vector3d& px4Gains) {
    _kPx4Gains = px4Gains;
  }

  void setAirframeType(std::string& airframeType) {
    computeControlAllocation(airframeType);
    setCmdLimits(airframeType);
  }

  void setThrustConstant(double thrustConstant) {
    _kThrust_constant = thrustConstant;
  }

  void setArmLength(double armLength) {
    _kArmLength = armLength;
  }

  void setMomentConstant(double momentConstant) {
    _kMomentConstant = momentConstant;
  }

  void setMaxRotorSpeed(double maxRotorSpeed) {
    _kMaxRotorSpeed = maxRotorSpeed;
  }

  void setMaxThrust(double maxThrust) {
    _kMaxThrust = maxThrust;
  }

  void setPosXEndEffector(double posXEndEffector) {
    _H_B_E.topRightCorner(3, 1) << posXEndEffector, 0.0, 0.0;
  }

  void setCustTrajStablznIterations(int custTrajStablznIterations) {
    cust_traj_stablzn_iterations = custTrajStablznIterations;
  }

  void setCustTrajStablznThreshold(float custTrajStablznThreshold) {
    cust_traj_stablzn_threshold = custTrajStablznThreshold;
  }
  /***************************************************************************************************************************************/
  /***************************************************** Getters ***************************************************************/

  bool isControllerActive() {
    return controller_active_;
  }

  double getCmdPitchMomentLimit() const {
    return _cmd_pitchMoment_limit;
  }

  double getCmdRollMomentLimit() const {
    return _cmd_rollMoment_limit;
  }

  double getCmdYawMomentLimit() const {
    return _cmd_yawMoment_limit;
  }

  bool isUseMavlinkInterface() const {
    return use_mavlink_interface;
  }

  double getPosXEndEffector() {
    return _H_B_E(0, 3);
  }

  bool isEnableDistObserver() const {
    return enable_dist_observer;
  }

  bool isRejectDisturbances() const {
    return reject_disturbances;
  }

  bool isTrackEndEffector() const {
    return track_end_effector;
  }

  ros::NodeHandle nh_;

  /***************************************************************************************************************************************/
  /***************************************************************************************************************************************/

 protected:
// Learning
  actionlib::SimpleActionServer<spc_uav_comm::LearnParamAction> act_server_;
  spc_uav_comm::LearnParamFeedback feedback_;
  spc_uav_comm::LearnParamResult result_;
  double action_radius;
  double action_speed;
  double action_depth;
  Eigen::Matrix<double, 4, 4> _H_I_E_init;
  bool custom_trajectory_running_;
  bool pos_initialized_;
  int action_type;
  bool action_running_;
  bool action_initialized_;
  float cust_traj_stablzn_threshold;
  int cust_traj_stablzn_iterations;

  // Controller's state
  bool controller_active_;

  // Homogeneous matrices
  Eigen::Matrix<double, 4, 4> _H_B_E;  // End Effector Frame to Body Frame
  Eigen::Matrix<double, 4, 4> _H_I_B;  // Body Frame to Inertial Frame
  Eigen::Matrix<double, 4, 4> _H_I_D;  // Desired Frame to Inertial Frame
// Twists
  Eigen::Matrix<double, 6, 1> _T_B;  // Body Frame to Inertial Frame expressed in Body Frame
  Eigen::Matrix<double, 6, 1> _T_D;  // Desired Frame to Inertial Frame expressed in Body Frame
  Eigen::Matrix<double, 6, 1> _Tdot_D;  // Desired Frame to Inertial Frame expressed in Body Fram;
  // TF Buffer
  tf2_ros::Buffer tfBuffer;
  // TF listener
  tf2_ros::TransformListener* tfl;

  // Publisher of control signals
  ros::Publisher control_signals_pub;

 private:
  // Flag to interface SITL [TRUE] or RotorS [FALSE]
  bool use_mavlink_interface;
  // Flag to interface Simulation or Experiment
  bool use_experiment_interface;
  //  Flight mode type:
  float flightMode;
  // Track End Effector Mode
  bool track_end_effector;
  // Disturbance Observer Mode
  bool enable_dist_observer;
  // Disturbance Rejection Mode
  bool reject_disturbances;

  // Names of setpoint Frame IDs
  std::string setpoint_frame_id, setpoint_child_frame_id;
  // Type of Airframe (quadrotor, hexarotor, tilt_hexarotor, omni_hexarotor)
  std::string uav_type;

  //Trajectory tracking parameters
  ros::Time traj_begin_time;

  // last ROS Time in which the control loop was executed
  ros::Time last_time;

  // Subscriber to px4 status
  ros::Subscriber status_sub;
  // Subscriber to topics from Aeroworks GUI
  ros::Subscriber commander_sub;
  // Subscriber to odometry topic
  ros::Subscriber odom_sub;

  // Publisher of GUI setpoint odometry message
  ros::Publisher gui_setpoint_pub;
  // Publisher of px4 setpoints used in SITL simulation
  ros::Publisher px4_setpoints_pub;

  // Publisher of rotor velocities used in RotorS simulation
  ros::Publisher rotor_velocity_command_pub;
  // Publisher of px4 acceleration setpoints used in SITL simulation
  ros::Publisher px4_acc_setpoints_pub;

  // An identity unit quaternion
  geometry_msgs::Quaternion unit_quaternion;
  // desired configration of UAV sent from Aeroworks GUI
  geometry_msgs::TransformStamped setpoint;
  // MAVROS message of px4 setpoints
  mavros_msgs::AttitudeTarget command_msg;
  // MAVROS message of px4 acceleration setpoints
  geometry_msgs::Vector3Stamped command_acc_msg;
  // A header used in the MAVROS mesage to bypass onboard attitude controller
  const uint8_t ignore_attitude = 0x80;

  // Control Allocation Constants
  double _kThrust_constant;
  double _kArmLength;
  double _kMomentConstant;
  double _kMaxRotorSpeed;
  double _kMaxThrust;

  // number of propellers
  int num_props_;
  //Maximum commanded thrust
  double _cmd_thrust_limit;
  // Maximum commanded thrust in x direction
  double _cmd_xthrust_limit;
  // Maximum commanded thrust in y direction
  double _cmd_ythrust_limit;
  // Maximum commanded roll moment
  double _cmd_rollMoment_limit;
  // Maximum commanded pitch moment
  double _cmd_pitchMoment_limit;
  // Maximum commanded yaw moment
  double _cmd_yawMoment_limit;

  // PX4 onboard controller gains
  Eigen::Vector3d _kPx4Gains;
  // Control Allocation matrix mapping command wrench to rotor velocities squared
  Eigen::MatrixXd control_allocation_matrix;
};

inline double limit(double value, double limit) {
  if (value > limit) {
    value = limit;
  }
  if (value < -limit) {
    value = -limit;
  }
  return value;
}

#endif /* INCLUDE_SPC_UAV_CONTROL_CONTROLLER_NODE_H_ */
