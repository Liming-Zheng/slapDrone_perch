
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <traj_opt/traj_opt.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/TrajectoryPoint.h>


#include <Eigen/Core>
#include <atomic>
#include <thread>
#include <cmath>
#include <vis_utils/vis_utils.hpp>
#include <std_msgs/String.h>

namespace planning {

Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " << ", ";");

class Nodelet : public nodelet::Nodelet {
 private:
  std::thread initThread_;
  ros::Subscriber triger_sub_;
  ros::Timer plan_timer_;

  // dawn   
  ros::Publisher slap_odom_pub;
  ros::Publisher slap_rpg_odom_pub;
  ros::Publisher slap_grasper_command_pub;
  ros::Subscriber now_position_sub;
  ros::Subscriber branch_pose_sub;
  ros::Time time_get_pub;
  ros::Time time_pub_cmd;
  bool pub_time = true;
  nav_msgs::Odometry now_position;
  nav_msgs::Odometry branch_pose;
  bool now_position_enable = true;
  bool rpg_cmd_;
  double yaw_right_distance_;
  bool using_grasper;
  std_msgs::String grasper_command;
  

  // dawn pub the cmd topic to slapDrone_base
  quadrotor_msgs::PositionCommand slap_odom;
  quadrotor_msgs::TrajectoryPoint slap_rpg_odom;

  std::shared_ptr<vis_utils::VisUtils> visPtr_;
  std::shared_ptr<traj_opt::TrajOpt> trajOptPtr_;

  // NOTE planning or fake target
  bool target_ = false;
  Eigen::Vector3d goal_;

  // NOTE just for debug
  bool debug_ = false;
  bool once_ = false;
  bool debug_replan_ = false;

  double tracking_dur_, tracking_dist_, tolerance_d_;
  Eigen::Vector3d perching_p_, perching_v_, perching_axis_;
  double perching_theta_;

  Trajectory traj_poly_;
  ros::Time replan_stamp_;
  int traj_id_ = 0;
  bool wait_hover_ = true;
  bool force_hover_ = true;

  int plan_hz_ = 10;

  std::atomic_bool triger_received_ = ATOMIC_VAR_INIT(false);

  // calculate target yaw
  double cal_terminal_yaw()
  {
      //TODO: need to know how to calculate the terminal yaw and quaternion

    return 0.0;
  }


  // get the heading of Drone
  double cal_slap_yaw(Eigen::Vector3d target_p, Eigen::Vector3d position_now, double target_yaw)
  {
    double d_x = position_now.x()-target_p.x();
    double d_y = position_now.y()-target_p.y();
    double distance = sqrt(d_x*d_x + d_y*d_y);
    if (distance > yaw_right_distance_)
    {
      return atan2(target_p.x()-position_now.x(), target_p.y()-position_now.y());
    }
    else{
      return target_yaw;
    }
  }

  void triger_callback(const geometry_msgs::PoseStampedConstPtr& msgPtr) {
    goal_ << msgPtr->pose.position.x, msgPtr->pose.position.y, 1.0;
    triger_received_ = true;
    time_get_pub = ros::Time::now();
  }

  void now_position_cb(const nav_msgs::OdometryConstPtr& msg){
    if (now_position_enable)
    {
      now_position.pose.pose.position.x = msg->pose.pose.position.x;
      now_position.pose.pose.position.y = msg->pose.pose.position.y;
      now_position.pose.pose.position.z = msg->pose.pose.position.z;
      now_position_enable = false;
    }
  }

  void branch_pose_cb(const nav_msgs::OdometryConstPtr& msg){
   
    branch_pose = *msg;
  }

  void debug_timer_callback(const ros::TimerEvent& event) {
    if (!triger_received_) {
      return;
    }
    Eigen::MatrixXd iniState;
    iniState.setZero(3, 4);
    bool generate_new_traj_success = false;
    Trajectory traj;
    Eigen::Vector3d target_p, target_v;
    Eigen::Quaterniond target_q;
    Eigen::Quaterniond land_q(1, 0, 0, 0);

    iniState.setZero();
    iniState.col(0).x() = now_position.pose.pose.position.x;
    iniState.col(0).y() = now_position.pose.pose.position.y;
    iniState.col(0).z() = now_position.pose.pose.position.z;
    iniState.col(1) = perching_v_;
    target_p = perching_p_;
    target_v = perching_v_;
    target_q.x() = 0.0;
    target_q.y() = 0.0;
    target_q.z() = 0.0;
    target_q.w() = 1.0;

    Eigen::Vector3d axis = perching_axis_.normalized();
    double theta = perching_theta_ * 0.5;
    land_q.w() = cos(theta);
    land_q.x() = axis.x() * sin(theta);
    land_q.y() = axis.y() * sin(theta);
    land_q.z() = axis.z() * sin(theta);
    land_q = target_q * land_q;

    std::cout << "iniState: \n"
              << iniState << std::endl;
    std::cout << "target_p: " << target_p.transpose() << std::endl;
    std::cout << "target_v: " << target_v.transpose() << std::endl;
    std::cout << "land_q: "
              << land_q.w() << ","
              << land_q.x() << ","
              << land_q.y() << ","
              << land_q.z() << "," << std::endl;
    // 产生新的轨迹，传入了初始化的状态，目标位置，目标速递，降落的角度， 分了几段，还有轨迹的引用
    generate_new_traj_success = trajOptPtr_->generate_traj(iniState, target_p, target_v, land_q, 10, traj);
    if (generate_new_traj_success) {
      visPtr_->visualize_traj(traj, "traj");

      Eigen::Vector3d tail_pos = traj.getPos(traj.getTotalDuration());
      Eigen::Vector3d tail_vel = traj.getVel(traj.getTotalDuration());
      visPtr_->visualize_arrow(tail_pos, tail_pos + 0.5 * tail_vel, "tail_vel");
    }
    if (!generate_new_traj_success) {
      triger_received_ = false;
      return;
      // assert(false);
    }

    // NOTE run vis
    // hopf fiberation
    auto v2q = [](const Eigen::Vector3d& v, Eigen::Quaterniond& q) -> bool {
      double a = v.x();
      double b = v.y();
      double c = v.z();
      if (c == -1) {
        return false;
      }
      double d = 1.0 / sqrt(2.0 * (1 + c));
      q.w() = (1 + c) * d;
      q.x() = -b * d;
      q.y() = a * d;
      q.z() = 0;
      return true;
    };

    auto f_DN = [](const Eigen::Vector3d& x) {
      double x_norm_2 = x.squaredNorm();
      return (Eigen::MatrixXd::Identity(3, 3) - x * x.transpose() / x_norm_2) / sqrt(x_norm_2);
    };

    nav_msgs::Odometry msg;
    msg.header.frame_id = "world";
    double dt = 0.01;
    Eigen::Quaterniond q_last;
    double max_omega = 0;
    bool pub_slap_trigger_once = true;
    for (double t = 0; t <= traj.getTotalDuration(); t += dt) {
      ros::Duration(dt).sleep();
      // drone
      Eigen::Vector3d p = traj.getPos(t);
      Eigen::Vector3d dawn_v = traj.getVel(t);
      Eigen::Vector3d a = traj.getAcc(t);
      Eigen::Vector3d j = traj.getJer(t);
      Eigen::Vector3d g(0, 0, -9.8);
      Eigen::Vector3d thrust = a - g;

      //dawn publish the trajectory msg to px4control
      if (!rpg_cmd_)
      {
        slap_odom.header.stamp = ros::Time::now();
        slap_odom.header.frame_id = "world";

        slap_odom.position.x = p.x();
        slap_odom.position.y = p.y();
        slap_odom.position.z = p.z();
        slap_odom.velocity.x = dawn_v.x();
        slap_odom.velocity.y = dawn_v.y();
        slap_odom.velocity.z = dawn_v.z();
        slap_odom.acceleration.x = a.x();
        slap_odom.acceleration.y = a.y();
        slap_odom.acceleration.z = a.z();
        slap_odom.jerk.x = j.x();
        slap_odom.jerk.y = j.y();
        slap_odom.jerk.z = j.z();

        // Here pub the trajectory to px4cotl, and then send the attitude command to px4. This method always delay for the target position.
        slap_odom_pub.publish(slap_odom);
      }
      else{
        ROS_INFO("##### test rpg #########################");
        ros::Time time_now = ros::Time::now();
        double t_cur = (time_now - time_get_pub).toSec();

        static ros::Time time_last = ros::Time::now();
        Eigen::AngleAxisd R_ego_ab = Eigen::AngleAxisd(0.0 * M_PI, Eigen::Vector3d::UnitZ());

        slap_rpg_odom.header.stamp = time_now;
        slap_rpg_odom.header.frame_id = "world";

        slap_rpg_odom.pose.position.x = p.x();
        slap_rpg_odom.pose.position.y = p.y();
        slap_rpg_odom.pose.position.z = p.z();
        slap_rpg_odom.velocity.linear.x = dawn_v.x();
        slap_rpg_odom.velocity.linear.y = dawn_v.y();
        slap_rpg_odom.velocity.linear.z = dawn_v.z();
        slap_rpg_odom.acceleration.linear.x = a.x();
        slap_rpg_odom.acceleration.linear.y = a.y();
        slap_rpg_odom.acceleration.linear.z = a.z();
        slap_rpg_odom.jerk.linear.x = j.x();
        slap_rpg_odom.jerk.linear.y = j.y();
        slap_rpg_odom.jerk.linear.z = j.z();

        slap_rpg_odom.heading = cal_slap_yaw(target_p, p, cal_terminal_yaw());

        Eigen::Vector3d I_eZ_I(0.0, 0.0, 1.0);
        Eigen::Quaterniond quatDes = Eigen::Quaterniond::FromTwoVectors(
          I_eZ_I, a + Eigen::Vector3d(0.0, 0.0, 9.81));
        Eigen::Quaternion<double> q_heading =
          Eigen::Quaternion<double>(Eigen::AngleAxis<double>( slap_rpg_odom.heading, Eigen::Matrix<double, 3, 1>::UnitZ()));
        Eigen::Quaternion<double> q_orientation = quatDes * q_heading;
        slap_rpg_odom.pose.orientation.x = q_orientation.x();
        slap_rpg_odom.pose.orientation.y = q_orientation.y();
        slap_rpg_odom.pose.orientation.z = q_orientation.z();
        slap_rpg_odom.pose.orientation.w = q_orientation.w();

        double time_step = 0.01;
        Eigen::Vector3d acc1 = a + Eigen::Vector3d(0.0, 0.0, 9.81);
        Eigen::Vector3d acc2 = a + Eigen::Vector3d(0.0, 0.0, 9.81) +
        time_step * j;  // should be acceleration at next trajectory point
        acc1.normalize();
        acc2.normalize();

        Eigen::Vector3d crossProd = acc1.cross(acc2);  // direction of omega, in inertial axes
        Eigen::Vector3d bodyrates_wf = Eigen::Vector3d(0, 0, 0);
        if (crossProd.norm() > 0.0) {
          bodyrates_wf = std::acos(acc1.dot(acc2)) / time_step * crossProd / crossProd.norm();
        }
        // rotate angular rates to bodyframe
        Eigen::Vector3d angular_rate = q_orientation.inverse() * bodyrates_wf;
        slap_rpg_odom.velocity.angular.x = angular_rate.x();
        slap_rpg_odom.velocity.angular.y = angular_rate.y();
        slap_rpg_odom.velocity.angular.z = angular_rate.z();
        // std::cout << "&&&&&&& angular.x $$$$$$$:" << angular_rate.x() << std::endl;

        // Here pub the trajectory to rgp control node. And then pub body_rate command to px4.
        slap_rpg_odom_pub.publish(slap_rpg_odom);
        // ROS_INFO("--------------rpg rpg-----------------");
      }

      
      if(pub_time)
      {
        time_pub_cmd = ros::Time::now();
         ROS_INFO("time needed from pub the trigger:%.f", time_pub_cmd.toSec()-time_get_pub.toSec());
         pub_time = false;
      }

      Eigen::Vector3d zb = thrust.normalized();
      {
    
        Eigen::Vector3d zb_dot = f_DN(thrust) * j;
        double omega12 = zb_dot.norm();
     
        if (omega12 > max_omega) {
          max_omega = omega12;
        }
      }

      Eigen::Quaterniond q;
      bool no_singlarity = v2q(zb, q);
      Eigen::MatrixXd R_dot = (q.toRotationMatrix() - q_last.toRotationMatrix()) / dt;
      Eigen::MatrixXd omega_M = q.toRotationMatrix().transpose() * R_dot;
      // std::cout << "omega_M: \n" << omega_M << std::endl;
      Eigen::Vector3d omega_real;
      omega_real.x() = -omega_M(1, 2);
      omega_real.y() = omega_M(0, 2);
      omega_real.z() = -omega_M(0, 1);
      // std::cout << "omega_real: " << omega_real.transpose() << std::endl;
      q_last = q;
      if (no_singlarity) {
        msg.pose.pose.position.x = p.x();
        msg.pose.pose.position.y = p.y();
        msg.pose.pose.position.z = p.z();
        msg.pose.pose.orientation.w = q.w();
        msg.pose.pose.orientation.x = q.x();
        msg.pose.pose.orientation.y = q.y();
        msg.pose.pose.orientation.z = q.z();
        msg.header.stamp = ros::Time::now();
        visPtr_->visualize_traj(traj, "traj");
        visPtr_->pub_msg(msg, "odom");
      }
      // target
      // Eigen::Vector3d fake_target_v = target_v * (1.0 + 0.5 * sin(1e6 * t));
      // target_p = target_p + fake_target_v * dt;
      // target_v *= 1.0001;
      target_p = target_p + target_v * dt;
      msg.pose.pose.position.x = target_p.x();
      msg.pose.pose.position.y = target_p.y();
      msg.pose.pose.position.z = target_p.z();
      msg.pose.pose.orientation.w = land_q.w();
      msg.pose.pose.orientation.x = land_q.x();
      msg.pose.pose.orientation.y = land_q.y();
      msg.pose.pose.orientation.z = land_q.z();
      msg.header.stamp = ros::Time::now();
      visPtr_->pub_msg(msg, "target_odom");
      if (trajOptPtr_->check_collilsion(p, a, target_p)) {
        std::cout << "collide!  t: " << t << std::endl;
      }
      // TODO replan
      if (debug_replan_ && t > 1.0 / plan_hz_ && traj.getTotalDuration() > 0.5) {
        // ros::Duration(3.0).sleep();

        iniState.col(0) = traj.getPos(t);
        iniState.col(1) = traj.getVel(t);
        iniState.col(2) = traj.getAcc(t);
        iniState.col(3) = traj.getJer(t);
        std::cout << "iniState: \n"
                  << iniState << std::endl;
        trajOptPtr_->generate_traj(iniState, target_p, target_v, land_q, 10, traj, t);
        visPtr_->visualize_traj(traj, "traj");
        t = 0;
        std::cout << "max omega: " << max_omega << std::endl;
      }
    }

    if (using_grasper && pub_slap_trigger_once)
    {
        grasper_command.data = "perching";
        slap_grasper_command_pub.publish(grasper_command);
        pub_slap_trigger_once = false;
        std::cout << ">>>>>>> grasper is working for perching <<<<<<<<< " << std::endl;
    }

    std::cout << "tailV: " << traj.getVel(traj.getTotalDuration()).transpose() << std::endl;
    std::cout << "max thrust: " << traj.getMaxThrust() << std::endl;
    std::cout << "max omega: " << max_omega << std::endl;

    triger_received_ = false;
  }

  void init(ros::NodeHandle& nh) {
    // set parameters of planning
    nh.getParam("replan", debug_replan_);

    // NOTE once
    nh.getParam("perching_px", perching_p_.x());
    nh.getParam("perching_py", perching_p_.y());
    nh.getParam("perching_pz", perching_p_.z());
    nh.getParam("perching_vx", perching_v_.x());
    nh.getParam("perching_vy", perching_v_.y());
    nh.getParam("perching_vz", perching_v_.z());
    nh.getParam("perching_axis_x", perching_axis_.x());
    nh.getParam("perching_axis_y", perching_axis_.y());
    nh.getParam("perching_axis_z", perching_axis_.z());
    nh.getParam("perching_theta", perching_theta_);
    nh.getParam("rpg_cmd", rpg_cmd_);
    nh.getParam("yaw_right_distance", yaw_right_distance_);
    nh.getParam("using_grasper", using_grasper);

    visPtr_ = std::make_shared<vis_utils::VisUtils>(nh);
    trajOptPtr_ = std::make_shared<traj_opt::TrajOpt>(nh);

    plan_timer_ = nh.createTimer(ros::Duration(1.0 / plan_hz_), &Nodelet::debug_timer_callback, this);

    triger_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("triger", 10, &Nodelet::triger_callback, this, ros::TransportHints().tcpNoDelay());

    now_position_sub = nh.subscribe<nav_msgs::Odometry>("/mocap/slapDrone", 10, &Nodelet::now_position_cb, this, ros::TransportHints().tcpNoDelay());

    branch_pose_sub = nh.subscribe<nav_msgs::Odometry>("/mocap/realBranch", 10, &Nodelet::branch_pose_cb, this, ros::TransportHints().tcpNoDelay());

    // dawn
    slap_odom_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/drone_commander/onboard_command", 10);
    
    // slap_rpg_odom_pub = nh.advertise<quadrotor_msgs::TrajectoryPoint>("/slapDrone/autopilot/reference_state", 10);
    slap_rpg_odom_pub = nh.advertise<quadrotor_msgs::TrajectoryPoint>("/rpg/command", 10);

    slap_grasper_command_pub = nh.advertise<std_msgs::String>("/slap_grasper_command", 1);

    ROS_WARN("Planning node initialized!");
  }

 public:
  void onInit(void) {
    ros::NodeHandle nh(getMTPrivateNodeHandle());
    initThread_ = std::thread(std::bind(&Nodelet::init, this, nh));
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(planning::Nodelet, nodelet::Nodelet);