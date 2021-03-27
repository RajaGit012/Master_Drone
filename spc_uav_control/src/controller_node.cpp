/*
 * controller_node.cpp
 *
 *  Created on: Mar 7, 2018
 *      Author: ramy
 *  Refactored on: May 2018
 *      Author: Asem
 *
 * The purpose of this class is to provide the commonly needed functionality
 * of a controller. This includes communicating with the GUI, converting to PX4
 * or RotorS set points and loading commonly needed parameters.
 */

#include "spc_uav_control/controller_node.h"

ControllerNode::ControllerNode()
    : act_server_(nh_, "learning", false) {
  // Look up ROS Parameter Server
  ros::NodeHandle node_priv("~");
  node_priv.param<std::string>("setpoint_frame_id", setpoint_frame_id, "world");
  node_priv.param<std::string>("setpoint_child_frame_id", setpoint_child_frame_id, "setpoint");
  node_priv.param<bool>("use_mavlink_interface", use_mavlink_interface, false);
  node_priv.param<bool>("use_experiment_interface", use_experiment_interface, false);

  // Initialize subscribers
  status_sub = nh_.subscribe("mavros/state", 10, &ControllerNode::statusCallback, this);
  commander_sub = nh_.subscribe("gui/commander", 10, &ControllerNode::guiCommanderCallback, this);

  // Initialize publishers
  control_signals_pub = nh_.advertise<geometry_msgs::WrenchStamped>("control_signals", 1);
  rotor_velocity_command_pub = nh_.advertise<mav_msgs::Actuators>("/command/motor_speed", 1);  // The topic on which motor commands go
  px4_acc_setpoints_pub = nh_.advertise<geometry_msgs::Vector3Stamped>("mavros/setpoint_accel/accel", 10);
  px4_setpoints_pub = nh_.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 10);
  // This publisher is used currrently for the Learning Module since the custom trajectory sub-module is implemented
  // in this ROS node and not in the GUI.
  // TODO: move the custom trajectory sub-module to GUI
  gui_setpoint_pub = nh_.advertise<nav_msgs::Odometry>("gui_setpoint_odometry", 1);

  // Initialize subscribers
  odom_sub = nh_.subscribe("/ground_truth/odom", 10, &ControllerNode::odomCallback, this);
  // Initialize Transform Listener
  tfl = new tf2_ros::TransformListener(tfBuffer);

  loadControllerROSParameters();

  // Initialize Identity Unit Quaternion
  unit_quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);

  last_time = ros::Time::now();
  flightMode = FREEFLIGHTMODE;

  _kThrust_constant = 1.49e-5;
  _kArmLength = 0.3;
  _kMomentConstant = 0.06;
  _kMaxRotorSpeed = 800;

  controller_active_ = false;

  _H_I_B.setIdentity();
  _T_B.setZero();
  _H_I_D.setIdentity();
  _T_D.setZero();
  _Tdot_D.setZero();
  _H_B_E.setIdentity();
  _H_B_E.topRightCorner(3, 1) << 0.0, 0.0, 0.0;

  //register the goal and feeback callbacks for the learning action
  act_server_.registerGoalCallback(boost::bind(&ControllerNode::learningGoalCB, this));
  act_server_.registerPreemptCallback(boost::bind(&ControllerNode::learningPreemptCB, this));
  act_server_.start();
  _H_I_E_init.setIdentity();
  action_radius = 0.5;
  action_speed = 0.5;
  action_depth = 0.2;
  action_type = 0;
  action_running_ = false;
  action_initialized_ = false;

  if (!use_experiment_interface) {
    // Unpause GAZEBO Environment
    unpauseGazebo();
  }

}

void ControllerNode::loadControllerROSParameters() {
  ros::NodeHandle node_priv("~");

  setThrustConstant(loadSingleROSParameter(node_priv,"thrust_constant"));

  setArmLength(loadSingleROSParameter(node_priv,"arm_length"));

  setMomentConstant(loadSingleROSParameter(node_priv,"moment_constant"));

  setMaxRotorSpeed(loadSingleROSParameter(node_priv,"max_rotor_speed"));

  setMaxThrust(loadSingleROSParameter(node_priv,"max_thrust"));

  // Changed as dynamic parameters
  //setCustTrajStablznThreshold(loadSingleROSParameter(node_priv,"cust_traj_stablzn_threshold"));
  //setCustTrajStablznIterations((int)loadSingleROSParameter(node_priv,"cust_traj_stablzn_iterations"));

  node_priv.param<std::string>("uav_type", uav_type, "tilt_hexarotor");

  setAirframeType(uav_type);

}

double ControllerNode::loadSingleROSParameter(ros::NodeHandle & priv_node, const std::string& parName) {
  double dummy;
  if (!priv_node.getParam(parName, dummy)) {
    ROS_ERROR( "Could not find %s parameter!",parName.c_str());
    return -1.0;
  } else {
    return dummy;
  }
}

void ControllerNode::unpauseGazebo() {

  std_srvs::Empty srv;
  bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  unsigned int i = 0;

  // Trying to unpause Gazebo for 10 seconds.
  while (i <= 10 && !unpaused) {
    ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
    ros::Duration(1.0).sleep();
    unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    ++i;
  }

  if (!unpaused) {
    ROS_FATAL("Could not wake up Gazebo.");
  } else {
    ROS_INFO("Unpaused the Gazebo simulation.");
  }

  // Wait for 5 seconds to let the Gazebo GUI show up.
  ros::Duration(3.0).sleep();

}

void ControllerNode::statusCallback(const mavros_msgs::State::ConstPtr& msg) {
  if (not (msg->mode.find("OFFBOARD") != std::string::npos)) {
    ROS_INFO("Waiting for Offboard Mode to be set!");
  }
}

void ControllerNode::guiCommanderCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  flightMode = msg->data[0];
  track_end_effector = ((msg->data[1]) == MODE_ON);
  enable_dist_observer = ((msg->data[2]) == MODE_ON);
  reject_disturbances = ((msg->data[3]) == MODE_ON);
}

void ControllerNode::updateGUISetpoint() {
  mav_msgs::EigenTrajectoryPoint desired_trajectory;

  if (computeDesiredTrajectory(tfBuffer, desired_trajectory)) {
    // Set controller's trajectory point and activate controller
    convertDesiredTrajectory(desired_trajectory);
  }
}

void ControllerNode::convertDesiredTrajectory(mav_msgs::EigenTrajectoryPoint desired_trajectory) {
  //Homogenous matrix
  _H_I_D.topLeftCorner(3, 3) = desired_trajectory.orientation_W_B.toRotationMatrix();
  _H_I_D.topRightCorner(3, 1) = desired_trajectory.position_W;

  // the W subscript is just how the message is defined.
  // Twist
  _T_D.topLeftCorner(3, 1) = desired_trajectory.angular_velocity_W;
  _T_D.bottomLeftCorner(3, 1) = desired_trajectory.velocity_W;

  // Derivative of Twist
  _Tdot_D.topLeftCorner(3, 1) = desired_trajectory.angular_acceleration_W;
  _Tdot_D.bottomLeftCorner(3, 1) = desired_trajectory.acceleration_W;
}

void ControllerNode::convertOdometry(mav_msgs::EigenOdometry odometry) {
  //Homogenous matrix
  _H_I_B.topLeftCorner(3, 3) = odometry.orientation_W_B.toRotationMatrix();
  _H_I_B.topRightCorner(3, 1) = odometry.position_W;

  //Twist already expressed in body-fixed frame
  _T_B.topLeftCorner(3, 1) = odometry.angular_velocity_B;
  _T_B.bottomLeftCorner(3, 1) = odometry.velocity_B;
}

void ControllerNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  // Transform odometry message to Eigen Odometry
  mav_msgs::EigenOdometry odometry;
  mav_msgs::eigenOdometryFromMsg(*msg, &odometry);

  // Check for type of interface (SITL, ROTORS, or Experiment)
  checkOdometryMessage(odometry);

  // Set Odometry Message
  convertOdometry(odometry);

}

bool ControllerNode::computeDesiredTrajectory(tf2_ros::Buffer& tfBuffer,
                                              mav_msgs::EigenTrajectoryPoint& desired_trajectory) {

  try {

    // Look Up Desired Configuration
    setpoint = tfBuffer.lookupTransform(setpoint_frame_id, setpoint_child_frame_id, ros::Time(0));
    mav_msgs::eigenTrajectoryPointFromTransformMsg(setpoint, &desired_trajectory);

    /* Learning Module - Custom Trajectory hardcoded*/
    if (action_running_) {
      if (action_initialized_) {
        if (action_type == 0)
          customCircle(desired_trajectory);
        else if (action_type == 1)
          customSquare(desired_trajectory);
        else
          finishCustomTrajectory();
      } else
        initCustomTrajectory(desired_trajectory);
    }

    // Publish the custom setpoint trajectory message
    // TODO: move implementation to GUI node so that the setpoint frame is also changed
    publishCustomDesiredTrajectory(desired_trajectory);

    controller_active_ = true;
    return true;
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();  // TODO: What is the purpose of this line ?
    // Deactivate controller
    controller_active_ = false;
    return false;
  }

}

void ControllerNode::publishCustomDesiredTrajectory(mav_msgs::EigenTrajectoryPoint& desired_trajectory){
  nav_msgs::Odometry desired_trajectory_msg;

  desired_trajectory_msg.child_frame_id = setpoint.child_frame_id;
  desired_trajectory_msg.header = setpoint.header;
  desired_trajectory_msg.pose.pose.position.x = desired_trajectory.position_W.x();
  desired_trajectory_msg.pose.pose.position.y = desired_trajectory.position_W.y();
  desired_trajectory_msg.pose.pose.position.z = desired_trajectory.position_W.z();

  desired_trajectory_msg.pose.pose.orientation.x = desired_trajectory.orientation_W_B.x();
  desired_trajectory_msg.pose.pose.orientation.y = desired_trajectory.orientation_W_B.y();
  desired_trajectory_msg.pose.pose.orientation.z = desired_trajectory.orientation_W_B.z();
  desired_trajectory_msg.pose.pose.orientation.w = desired_trajectory.orientation_W_B.w();

  desired_trajectory_msg.twist.twist.linear.x = desired_trajectory.velocity_W.x();
  desired_trajectory_msg.twist.twist.linear.y = desired_trajectory.velocity_W.y();
  desired_trajectory_msg.twist.twist.linear.z = desired_trajectory.velocity_W.z();

  desired_trajectory_msg.twist.twist.angular.x = desired_trajectory.angular_velocity_W.x();
  desired_trajectory_msg.twist.twist.angular.y = desired_trajectory.angular_velocity_W.y();
  desired_trajectory_msg.twist.twist.angular.z = desired_trajectory.angular_velocity_W.z();

  gui_setpoint_pub.publish(desired_trajectory_msg);
}

void ControllerNode::learningGoalCB() {
  const spc_uav_comm::LearnParamGoalConstPtr &goal = act_server_.acceptNewGoal();
  if (goal->start) {
    // get information from the received goal
    action_depth = goal->depth;
    action_radius = goal->radius;
    action_speed = goal->speed;
    action_type = goal->atype;
    if (!action_running_) {
      action_running_ = true;
      captureDesiredStartConfiguration();
    }
    feedback_.started = false;
    action_initialized_ = false;
    traj_begin_time = ros::Time::now();
  } else {
    // if start == false don't do anything
    result_.finished = false;
    act_server_.setAborted(result_);
  }
}

void ControllerNode::learningPreemptCB() {
  // stop following the trajectory and follow the gui set point
  action_running_ = false;
  result_.finished = false;
  act_server_.setPreempted(result_);
}

void ControllerNode::initCustomTrajectory(mav_msgs::EigenTrajectoryPoint& desired_trajectory) {
  static Eigen::Vector3d last_position(0.0, 0.0, 0.0);
  static int iterations = 0;

  // command the end effector to go to the start position
  desired_trajectory.position_W = _H_I_E_init.topRightCorner(3, 1) + Eigen::Vector3d(0, action_radius, 0);
  //desired_trajectory.orientation_W_B = Eigen::Quaterniond((Eigen::Matrix3d)_H_I_E_init.topLeftCorner(3,3));
  desired_trajectory.velocity_W.setZero();
  desired_trajectory.acceleration_W.setZero();

  if (!feedback_.started) {
    // compare the current position with the position seen before N iterations
    // this necessary to ensure stability
    // N := cust_traj_stablzn_iterations
    while (iterations++ < cust_traj_stablzn_iterations)
      return;  // do nothing for N iterations
    iterations = 0;

    // compare the current position with the position seen before N iterations
    // Units of cust_traj_stablzn_threshold = [m]
    if ((((_H_I_B * _H_B_E).topRightCorner(3, 1) - last_position).norm() < cust_traj_stablzn_threshold)) {
      // if the difference is small enough, positioning is done
      last_position.setZero();
      feedback_.started = true;
    } else {
      // save the current position (will be compared with the position after N iterations)
      last_position = (_H_I_B * _H_B_E).topRightCorner(3, 1);
    }
  } else {
    // if positioning is done, press against the wall for N iterations before starting the trajectory
    // do this for N iterations is necessary to ensure the trajectory starts AFTER pressing
    // the trajectory will continue pressing after it starts.
    desired_trajectory.position_W += Eigen::Vector3d(action_depth, 0, 0);
    while (iterations++ < cust_traj_stablzn_iterations)
      return;
    iterations = 0;
    traj_begin_time = ros::Time::now();
    // after this finishes, the complete initialization is done
    action_initialized_ = true;
    act_server_.publishFeedback(feedback_);
  }
}

void ControllerNode::captureDesiredStartConfiguration() {
  _H_I_E_init = _H_I_D;  // Set to current configuration
  _H_I_E_init(1, 3) -= action_radius;  // Set y-coord because it will be initialized later with action radius
}

void ControllerNode::customCircle(mav_msgs::EigenTrajectoryPoint& desired_trajectory) {
  Eigen::Matrix3d R_I_D = desired_trajectory.orientation_W_B.toRotationMatrix().transpose();
  ros::Time current_time = ros::Time::now();
  ros::Duration diff = ros::Time::now() - traj_begin_time;
  double t_i = diff.toSec();
  if (action_speed * t_i > 2 * M_PI * action_radius) {
    finishCustomTrajectory();
    return;
  }

  desired_trajectory.position_W = _H_I_E_init.topRightCorner(3, 1) + Eigen::Vector3d(action_depth, 0, 0);
  //desired_trajectory.orientation_W_B = Eigen::Quaterniond((Eigen::Matrix3d)_H_I_E_init.topLeftCorner(3,3));
  desired_trajectory.velocity_W.setZero();
  desired_trajectory.acceleration_W.setZero();

  desired_trajectory.position_W(1) += action_radius * std::cos(action_speed * t_i / action_radius);
  desired_trajectory.position_W(2) += action_radius * std::sin(action_speed * t_i / action_radius);

  desired_trajectory.velocity_W(1) += -action_speed * std::sin(action_speed * t_i / action_radius);
  desired_trajectory.velocity_W(2) += action_speed * std::cos(action_speed * t_i / action_radius);

  desired_trajectory.velocity_W = R_I_D * desired_trajectory.velocity_W;

  desired_trajectory.acceleration_W(1) += -action_speed * action_speed / action_radius
      * std::cos(action_speed * t_i / action_radius);
  desired_trajectory.acceleration_W(2) += -action_speed * action_speed / action_radius
      * std::sin(action_speed * t_i / action_radius);

  desired_trajectory.acceleration_W = R_I_D * desired_trajectory.acceleration_W;
}

void ControllerNode::customSquare(mav_msgs::EigenTrajectoryPoint& desired_trajectory) {
  Eigen::Matrix3d R_I_D = desired_trajectory.orientation_W_B.toRotationMatrix().transpose();
  ros::Time current_time = ros::Time::now();
  ros::Duration diff = ros::Time::now() - traj_begin_time;
  double t_i = diff.toSec();
  if (action_speed * t_i > 8.0 * action_radius) {
    finishCustomTrajectory();
    return;
  }

  desired_trajectory.position_W = _H_I_E_init.topRightCorner(3, 1) + Eigen::Vector3d(action_depth, 0, 0);
  //desired_trajectory.orientation_W_B = Eigen::Quaterniond((Eigen::Matrix3d)_H_I_E_init.topLeftCorner(3,3));
  desired_trajectory.velocity_W.setZero();
  desired_trajectory.acceleration_W.setZero();

  if (action_speed * t_i > 7.0 * action_radius) {
    desired_trajectory.position_W(1) += action_radius;
    desired_trajectory.position_W(2) += action_speed * t_i - 8.0 * action_radius;
    desired_trajectory.velocity_W(2) += action_speed;
  } else if (action_speed * t_i > 5.0 * action_radius) {
    desired_trajectory.position_W(1) += action_speed * t_i - 6.0 * action_radius;
    desired_trajectory.position_W(2) -= action_radius;
    desired_trajectory.velocity_W(1) += action_speed;
  } else if (action_speed * t_i > 3.0 * action_radius) {
    desired_trajectory.position_W(1) -= action_radius;
    desired_trajectory.position_W(2) += 4.0 * action_radius - action_speed * t_i;
    desired_trajectory.velocity_W(2) -= action_speed;
  } else if (action_speed * t_i > action_radius) {
    desired_trajectory.position_W(1) += 2.0 * action_radius - action_speed * t_i;
    desired_trajectory.position_W(2) += action_radius;
    desired_trajectory.velocity_W(1) -= action_speed;
  } else {
    desired_trajectory.position_W(1) += action_radius;
    desired_trajectory.position_W(2) += action_speed * t_i;
    desired_trajectory.velocity_W(2) += action_speed;
  }

  desired_trajectory.velocity_W = R_I_D * desired_trajectory.velocity_W;
  desired_trajectory.acceleration_W = R_I_D * desired_trajectory.acceleration_W;

}

void ControllerNode::finishCustomTrajectory() {
  // switch to the gui setpoint
  action_running_ = false;
  result_.finished = true;
  act_server_.setSucceeded(result_);
}

void ControllerNode::checkOdometryMessage(mav_msgs::EigenOdometry& odometry) {
  ROS_INFO_ONCE("Motion_Controller got first odometry message.");
  // Gazebo feedbacks body velocities expressed in inertial frame, so they must be transformed to the body frame
  if (use_mavlink_interface && !use_experiment_interface) {
    Eigen::Matrix3d R_B_I = odometry.orientation_W_B.toRotationMatrix().transpose();
    odometry.angular_velocity_B = R_B_I * odometry.angular_velocity_B;
    odometry.velocity_B = R_B_I * odometry.velocity_B;
  }

}

double ControllerNode::computeElapsedTime() {
  ros::Time current_time = ros::Time::now();
  ros::Duration diff = current_time - last_time;
  last_time = ros::Time::now();
  return diff.toSec();

}

void ControllerNode::transformWrenchToAngRateSetPoints(Eigen::VectorXd& command_wrench,
                                                       Eigen::VectorXd* px4_setpoints) const {
  assert(px4_setpoints);
  px4_setpoints->resize(6);

  // Extract commanded torques
  Eigen::Vector3d tau = command_wrench.head(3);
  // Normalize torques
  tau(0) = tau(0) / _cmd_rollMoment_limit;
  tau(1) = tau(1) / _cmd_pitchMoment_limit;
  tau(2) = tau(2) / _cmd_yawMoment_limit;

// Compute desired angular rates to PX4 onboard controller
  Eigen::Vector3d desAngRates;  // [rad/s]
  desAngRates = tau.cwiseQuotient(_kPx4Gains);

// Calculate the roll rate, pitch rate and yaw rate
  double command_rollrate, command_pitchrate, command_yawrate, command_xthrust, command_ythrust, command_thrust;

  command_rollrate = desAngRates(0);
  command_pitchrate = -desAngRates(1);
  command_yawrate = -desAngRates(2);

  // Debugging whether command set points reach px4 correctly or not
  //std::string sep = "\n----------------------------------------\n";
  //std::cout <<sep<< command_rollrate<<", "<< command_pitchrate<<", "<< command_yawrate << sep;

  // Normalize commanded thrust
  // Extract commanded forces
  Eigen::Vector3d force = command_wrench.tail(3);
  // Normalize forces
  command_xthrust = force(0) / _cmd_xthrust_limit;
  command_ythrust = force(1) / _cmd_ythrust_limit;
  command_thrust = force(2) / _cmd_thrust_limit;

  command_xthrust = limit(command_xthrust, 1.0);
  command_ythrust = limit(command_ythrust, 1.0);
  command_thrust = limit(command_thrust, 1.0);

  // Write commanded signals
  *px4_setpoints << command_rollrate, command_pitchrate, command_yawrate, command_xthrust, command_ythrust, command_thrust;
}

void ControllerNode::computeControlAllocation(std::string& airframeType) {
  const double kDegToRad = M_PI / 180.0;
  std::cout << "Start to compute Control Allocation !! \n";
  // Map from rotor velocities squared to control wrench
  Eigen::MatrixXd A;

  // Check Airframe Type

  if (airframeType.compare("tilt_hexarotor") == 0 || airframeType.compare("omni_hexarotor") == 0) {
    A.resize(6, 6);
    A.setZero();

    // Precompute constants
    const double kSa = std::sin(47 * kDegToRad);
    const double kCa = std::cos(47 * kDegToRad);
    const double kC = std::cos(30 * kDegToRad);
    const double kG = _kMomentConstant;
    const double kL = _kArmLength;

    // Compute column vectors of U and T matrices
    Eigen::Vector3d t12, t34, t56, u12, u34, u56;

    u12 << -kSa, 0, kCa;
    u34 << 0.5 * kSa, -kC * kSa, kCa;
    u56 << 0.5 * kSa, kC * kSa, kCa;

    t12 << (-kL * kCa - kG * kSa), 0, (-kL * kSa + kG * kCa);
    t34 << 0.5 * (kL * kCa + kG * kSa), -kC * (kL * kCa + kG * kSa), (-kL * kSa + kG * kCa);
    t56 << -0.5 * (kL * kCa + kG * kSa), -kC * (kL * kCa + kG * kSa), (kL * kSa - kG * kCa);

    // Concatenate U and T matrices
    A.col(0).head(3) << t12;
    A.col(1).head(3) << -t12;
    A.col(0).tail(3) << u12;
    A.col(1).tail(3) << u12;

    A.col(2).head(3) << t34;
    A.col(3).head(3) << -t34;
    A.col(2).tail(3) << u34;
    A.col(3).tail(3) << u34;

    A.col(4).head(3) << t56;
    A.col(5).head(3) << -t56;
    A.col(4).tail(3) << u56;
    A.col(5).tail(3) << u56;

    // Multiply by thrust constant
    A = _kThrust_constant * A;

    // Compute the control allocation matrix, mapping command wrench to rotor velocities squared
    control_allocation_matrix.resize(6, 6);
    control_allocation_matrix = A.inverse();

    // Set number of rotors
    num_props_ = 6;
  }

  if (airframeType.compare("hexarotor") == 0) {
    Eigen::Vector4d k;  // Helper diagonal matrix.
    k << _kThrust_constant, _kThrust_constant * _kArmLength, _kThrust_constant * _kArmLength, _kMomentConstant
        * _kThrust_constant;
    const double kS = std::sin(30 * kDegToRad);
    const double kC = std::cos(30 * kDegToRad);
    // Map from rotor velocities squared to control wrench
    A.resize(4, 6);
    A << 1, 1, 1, 1, 1, 1, -1, 1, kS, -kS, -kS, kS, 0, 0, -kC, kC, -kC, kC, 1, -1, 1, -1, -1, 1;
    A = k.asDiagonal() * A;

    // Compute the control allocation matrix, mapping command wrench to rotor velocities squared
    control_allocation_matrix.resize(6, 4);
    control_allocation_matrix = A.transpose() * (A * A.transpose()).inverse();

    // Set number of rotors
    num_props_ = 6;
  }

  if (airframeType.compare("quadrotor") == 0) {
    Eigen::Vector4d k;  // Helper diagonal matrix.
    const double kSq = 1.0 / std::sqrt(2.0);
    k << _kThrust_constant, _kThrust_constant * _kArmLength * kSq, _kThrust_constant * _kArmLength * kSq, _kMomentConstant
        * _kThrust_constant;

    // Map from rotor velocities squared to control wrench
    A.resize(4, 4);
    A << 1, 1, 1, 1, -1, 1, 1, -1, -1, 1, -1, 1, -1, -1, 1, 1;
    A = k.asDiagonal() * A;

    // Map from control wrench to rotor velocities squared
    control_allocation_matrix.resize(4, 4);
    control_allocation_matrix = A.inverse();
    num_props_ = 4;
  }
  // Print Control Allocation Matrix
  std::string sep = "\n----------------------------------------\n";
  std::cout << A << sep;
  std::cout << control_allocation_matrix << sep;
  std::cout << "Finished Control Allocation !! \n";

  /* DEBUGGING Incorrect Control Allocation Normalization
   const double kSa2 = std::sin(47 * kDegToRad);
   const double kCa2 = std::cos(47 * kDegToRad);
   const double kC2 = std::cos(30 * kDegToRad);
   const double kL2 = _kArmLength;

   Eigen::MatrixXd control_allocation_matrix2;
   Eigen::MatrixXd control_allocation_matrix2_inv;
   Eigen::MatrixXd control_allocation_matrix2_inv2;


   control_allocation_matrix2.resize(6,6);
   control_allocation_matrix2_inv.resize(6,6);
   control_allocation_matrix2_inv2.resize(6,6);

   control_allocation_matrix2.row(0) = A.row(0)/(0.5*_kThrust_constant * kL2*kCa2);
   control_allocation_matrix2.row(1) = A.row(1)/(kC2*_kThrust_constant * kL2*kCa2);
   control_allocation_matrix2.row(2) = A.row(2)/(_kThrust_constant * kL2*kSa2);

   control_allocation_matrix2.row(3) = A.row(3)/(0.5*_kThrust_constant *kSa2);
   control_allocation_matrix2.row(4) = A.row(4)/(kC2*_kThrust_constant *kSa2);
   control_allocation_matrix2.row(5) = A.row(5)/(_kThrust_constant *kCa2);

   control_allocation_matrix2_inv = 6.0*control_allocation_matrix2.inverse();

   control_allocation_matrix2_inv2.col(0) = control_allocation_matrix.col(0)*(0.5*_kThrust_constant * kL2*kCa2);
   control_allocation_matrix2_inv2.col(1) = control_allocation_matrix.col(1)*(kC2*_kThrust_constant * kL2*kCa2);
   control_allocation_matrix2_inv2.col(2) = control_allocation_matrix.col(2)*(_kThrust_constant * kL2*kSa2);

   control_allocation_matrix2_inv2.col(3) = control_allocation_matrix.col(3)*(0.5*_kThrust_constant *kSa2);
   control_allocation_matrix2_inv2.col(4) = control_allocation_matrix.col(4)*(kC2*_kThrust_constant *kSa2);
   control_allocation_matrix2_inv2.col(5) = control_allocation_matrix.col(5)*(_kThrust_constant *kCa2);
   control_allocation_matrix2_inv2 = 6.0*control_allocation_matrix2_inv2;

   // Print Control Allocation Matrix
   sep = "\n----------------------------------------\n";
   std::cout << control_allocation_matrix2 << sep;
   std::cout << control_allocation_matrix2_inv << sep;
   std::cout << control_allocation_matrix2_inv2 << sep;
   */
}

void ControllerNode::calculateRotorVelocities(Eigen::VectorXd& command_wrench, Eigen::VectorXd* rotor_velocities,
                                              std::string& airframeType) const {
  assert(rotor_velocities);
  rotor_velocities->resize(num_props_);
  // Return 0 velocities on all rotors, until the first command is received.
  if (!controller_active_) {
    rotor_velocities->setConstant(100.0);  // Just to indicate arming of propellers in simulation
    return;
  }
  // Map command wrench to rotor velocties
  *rotor_velocities = control_allocation_matrix * command_wrench;

  // Compute signed square root of rotor velocities
  for (int i = 0; i < num_props_; i++) {
    double w = (*rotor_velocities)(i);
    if (w >= 0) {
      w = std::sqrt(w);
    } else {
      if (airframeType.compare("omni_hexarotor") == 0) {
        w = -1.0 * std::sqrt(-1.0 * w);
      } else {
        w = 0.0;
      }
    }
    //std::cout << w << sep;
    (*rotor_velocities)(i) = w;
  }

}

void ControllerNode::publishPX4setpoints(Eigen::VectorXd& px4_setpoints) {

  // Construct the message to send to the Pixhawk
  command_msg.type_mask = ignore_attitude;
  command_msg.orientation = unit_quaternion;
  command_msg.body_rate.x = px4_setpoints(0);
  command_msg.body_rate.y = px4_setpoints(1);
  command_msg.body_rate.z = px4_setpoints(2);
  command_msg.thrust = px4_setpoints(5);
  command_msg.header.frame_id = "base_link";
  command_msg.header.stamp = ros::Time::now();

  px4_setpoints_pub.publish(command_msg);

  command_acc_msg.header.frame_id = "base_link";
  command_acc_msg.header.stamp = ros::Time::now();
  command_acc_msg.vector.x = px4_setpoints(3);
  command_acc_msg.vector.y = px4_setpoints(4);
  command_acc_msg.vector.z = px4_setpoints(5);

  // Warning!  Accelerations are defined here in ENU (xyz) frame.
  // In mavros pkg, they will be converted to NED
  // In firmware, we will handle reconversion from NED to ENU

  px4_acc_setpoints_pub.publish(command_acc_msg);

}

void ControllerNode::publishRotorSsetpoints(Eigen::VectorXd& ref_rotor_velocities) {

  // Construct the RotorS message
  mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

  actuator_msg->angular_velocities.clear();
  for (int i = 0; i < ref_rotor_velocities.size(); i++) {
    actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
  }
  actuator_msg->header.stamp = ros::Time::now();
  actuator_msg->header.frame_id = "base_link";

  //ROS_INFO("Published Rotor Velocities %f, %f, %f, %f",ref_rotor_velocities[0],ref_rotor_velocities[1],ref_rotor_velocities[2],ref_rotor_velocities[3]); // For Debugging
  // TODO: capture this published message in GUI to display motor speeds
  rotor_velocity_command_pub.publish(actuator_msg);
}

void ControllerNode::publishSetPoints(Eigen::VectorXd& command_wrench) {
  // For Debugging
  if (!isControllerActive()) {
    ROS_INFO("Controller is NOT ACTIVE !!");
  }

  // Either Use SITL interface OR RotorS interface
  if (use_mavlink_interface) {

    Eigen::VectorXd px4_setpoints;
    transformWrenchToAngRateSetPoints(command_wrench, &px4_setpoints);

    publishPX4setpoints(px4_setpoints);

  } else {
    Eigen::VectorXd ref_rotor_velocities;
    calculateRotorVelocities(command_wrench, &ref_rotor_velocities, uav_type);

    publishRotorSsetpoints(ref_rotor_velocities);
  }
}

void ControllerNode::publishControlSignals(Eigen::VectorXd& command_wrench) {
  geometry_msgs::WrenchStamped wrench_msg;
  wrench_msg.wrench.torque.x = command_wrench(0);
  wrench_msg.wrench.torque.y = command_wrench(1);
  wrench_msg.wrench.torque.z = command_wrench(2);
  wrench_msg.wrench.force.x = command_wrench(3);
  wrench_msg.wrench.force.y = command_wrench(4);
  wrench_msg.wrench.force.z = command_wrench(5);
  wrench_msg.header.stamp = ros::Time::now();
  wrench_msg.header.frame_id = "base_link";

  control_signals_pub.publish(wrench_msg);
}

void ControllerNode::setCmdLimits(std::string& airframeType) {
  const double kDegToRad = M_PI / 180.0;

  if (airframeType.compare("tilt_hexarotor") == 0 || airframeType.compare("omni_hexarotor") == 0) {
    const double kSq = std::sqrt(3.0);
    const double kSa = std::sin(47 * kDegToRad);
    const double kCa = std::cos(47 * kDegToRad);

    _cmd_rollMoment_limit = 3 * kCa * _kArmLength * _kMaxThrust;
    _cmd_pitchMoment_limit = 3 * kSq * kCa * _kArmLength * _kMaxThrust;
    _cmd_yawMoment_limit = 6 * _kArmLength * kSa * _kMaxThrust;
    _cmd_xthrust_limit = 3 * kSa * _kMaxThrust;
    _cmd_ythrust_limit = 3 * kSq * kSa * _kMaxThrust;
    _cmd_thrust_limit = 6 * kCa * _kMaxThrust;
  }

  if (airframeType.compare("quadrotor") == 0) {
    const double kSq = 1.0 / std::sqrt(2.0);

    _cmd_xthrust_limit = 10000;
    _cmd_ythrust_limit = 10000;

    _cmd_rollMoment_limit = 4.0 * kSq * _kArmLength * _kMaxThrust;
    _cmd_pitchMoment_limit = 4.0 * kSq * _kArmLength * _kMaxThrust;
    _cmd_yawMoment_limit = 4 * _kMomentConstant * _kMaxThrust;
    _cmd_thrust_limit = 4 * _kMaxThrust;
  }

  if (airframeType.compare("hexarotor") == 0) {
    const double kSq = std::sqrt(3.0);

    _cmd_xthrust_limit = 10000;
    _cmd_ythrust_limit = 10000;

    _cmd_rollMoment_limit = 3 * _kArmLength * _kMaxThrust;
    _cmd_pitchMoment_limit = 3 * kSq * _kArmLength * _kMaxThrust;
    _cmd_yawMoment_limit = 6 * _kMomentConstant * _kMaxThrust;
    _cmd_thrust_limit = 6 * _kMaxThrust;
  }

  //std::string sep = "\n----------PX4 Limits-----------\n";
  //std::cout << _cmd_rollMoment_limit << "\t" << _cmd_pitchMoment_limit << "\t" << _cmd_yawMoment_limit << "\t"
  //         << _cmd_xthrust_limit << "\t" << _cmd_ythrust_limit << "\t" << _cmd_thrust_limit << sep;

}
