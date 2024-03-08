/* path_publisher_node.cpp

 * Copyright (C) 2023 SS47816

 * ROS Node for publishing short term paths

**/

#include "me5413_world/path_publisher_node.hpp"

namespace me5413_world
{

PathPublisherNode::PathPublisherNode() : tf2_listener_(tf2_buffer_)
{
  this->timer_ = nh_.createTimer(ros::Duration(0.2), &PathPublisherNode::timerCallback, this);
  this->sub_robot_odom_ = nh_.subscribe("/gazebo/ground_truth/state", 1, &PathPublisherNode::robotOdomCallback, this);
  this->pub_global_path_ = nh_.advertise<nav_msgs::Path>("/me5413_world/planning/global_path", 1);
  this->pub_local_path_ = nh_.advertise<nav_msgs::Path>("/me5413_world/planning/local_path", 1);
  this->pub_absolute_position_error_ = nh_.advertise<std_msgs::Float32>("/me5413_world/planning/position_error", 1);
  this->pub_absolute_heading_error_ = nh_.advertise<std_msgs::Float32>("/me5413_world/planning/heading_error", 1);

  // Initialization
  this->robot_frame_ = "base_link";
  this->map_frame_ = "map";
  this->world_frame_ = "world";

  this->global_path_msg_.header.frame_id = this->world_frame_;
  this->local_path_msg_.header.frame_id = this->world_frame_;

  this->absolute_position_error_.data = 0.0;
  this->absolute_heading_error_.data = 0.0;
  this->relative_position_error_.data = 0.0;
  this->relative_heading_error_.data = 0.0;
};

void PathPublisherNode::timerCallback(const ros::TimerEvent &)
{
  // Calculate absolute errors (wrt to world frame)
  const std::pair<double, double> error_absolute = calculatePoseError(this->pose_world_robot_, this->pose_world_goal_);
  this->absolute_position_error_.data = error_absolute.first;
  this->absolute_heading_error_.data = error_absolute.second;

  // Publish errors
  this->pub_absolute_position_error_.publish(this->absolute_position_error_);
  this->pub_absolute_heading_error_.publish(this->absolute_heading_error_);

  // Publish paths
  publishGlobalPath(10, 10, 0.001);
  publishLocalPath(this->pose_world_robot_, 10, 90);

  return;
};

void PathPublisherNode::robotOdomCallback(const nav_msgs::Odometry::ConstPtr &odom)
{
  this->world_frame_ = odom->header.frame_id;
  this->robot_frame_ = odom->child_frame_id;
  this->pose_world_robot_ = odom->pose.pose;

  const tf2::Transform T_world_robot = convertPoseToTransform(this->pose_world_robot_);
  const tf2::Transform T_robot_world = T_world_robot.inverse();

  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = this->robot_frame_;
  transformStamped.child_frame_id = this->world_frame_;
  transformStamped.transform.translation = tf2::toMsg(T_robot_world.getOrigin());
  // transformStamped.transform.translation.z = 0.0;
  transformStamped.transform.rotation = tf2::toMsg(T_robot_world.getRotation());
  this->tf2_bcaster_.sendTransform(transformStamped);

  return;
};

void PathPublisherNode::publishGlobalPath(const double A, const double B, const double t_res)
{
  std::vector<geometry_msgs::PoseStamped> poses;
  const double t_increament = t_res * 2 * M_PI;

  // Calculate the positions
  for (double t = 0.0; t <= 2 * M_PI; t += t_increament)
  {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = A * std::sin(t);
    pose.pose.position.y = B * std::sin(t) * std::cos(t);
    poses.push_back(pose);
  }

  // Calcuate the orientations
  tf2::Quaternion q;
  for (size_t i = 0; i < poses.size(); i++)
  {
    const double x_d = poses[i + 1].pose.position.x - poses[i].pose.position.x;
    const double y_d = poses[i + 1].pose.position.y - poses[i].pose.position.y;
    const double yaw = std::atan2(y_d, x_d);

    q.setRPY(0.0, 0.0, yaw);
    q.normalize();
    poses[i].pose.orientation = tf2::toMsg(q);
  }
  poses.back().pose.orientation = tf2::toMsg(q);

  // Update the message
  this->global_path_msg_.header.stamp = ros::Time::now();
  this->global_path_msg_.poses = poses;
  this->pub_global_path_.publish(this->global_path_msg_);
}

void PathPublisherNode::publishLocalPath(const geometry_msgs::Pose &robot_pose, const size_t n_wp_prev, const size_t n_wp_post)
{
  size_t id_next = nextWaypoint(robot_pose, this->global_path_msg_, 0);
  size_t id_start = std::max(id_next - n_wp_prev, size_t(0));
  size_t id_end = std::min(id_next + n_wp_post, this->global_path_msg_.poses.size() - 1);

  std::vector<geometry_msgs::PoseStamped>::const_iterator start = this->global_path_msg_.poses.begin() + id_start;
  std::vector<geometry_msgs::PoseStamped>::const_iterator end = this->global_path_msg_.poses.begin() + id_end;

  // Update the message
  this->local_path_msg_.header.stamp = ros::Time::now();
  this->local_path_msg_.poses = std::vector<geometry_msgs::PoseStamped>(start, end);
  this->pub_local_path_.publish(this->local_path_msg_);
}

size_t PathPublisherNode::closestWaypoint(const geometry_msgs::Pose &robot_pose, const nav_msgs::Path &path, const size_t id_start = 0)
{
  const double yaw_robot = getYawFromOrientation(robot_pose.orientation);

  double min_dist = DBL_MAX;
  size_t id_closest = id_start;
  for (size_t i = id_start; i < path.poses.size(); i++)
  {
    const double dist = std::hypot(robot_pose.position.x - path.poses[i].pose.position.x, robot_pose.position.y - path.poses[i].pose.position.y);

    if (dist <= min_dist)
    {
      const double yaw_wp = getYawFromOrientation(path.poses[i].pose.orientation);
      const double yaw_diff = (yaw_robot - yaw_wp) / M_PI * 180.0;
      if (std::fabs(yaw_diff) <= 90.0)
      {
        min_dist = dist;
        id_closest = i;
      }
    }
  }

  return id_closest;
}

size_t PathPublisherNode::nextWaypoint(const geometry_msgs::Pose &robot_pose, const nav_msgs::Path &path, const size_t id_start = 0)
{
  size_t id_closest = closestWaypoint(robot_pose, path, id_start);
  double yaw_T_robot_wp = atan2((path.poses[id_closest].pose.position.y - robot_pose.position.y),
                                (path.poses[id_closest].pose.position.x - robot_pose.position.x));

  const double yaw_robot = getYawFromOrientation(robot_pose.orientation);
  const double angle = std::fabs(yaw_robot - yaw_T_robot_wp);
  const double angle_norm = std::min(2 * M_PI - angle, angle); // TODO: check if this is correct

  if (angle_norm > M_PI / 2)
  {
    id_closest++;
  }

  return id_closest;
}

double PathPublisherNode::getYawFromOrientation(const geometry_msgs::Quaternion &orientation)
{
  tf2::Quaternion q;
  tf2::fromMsg(orientation, q);
  const tf2::Matrix3x3 m = tf2::Matrix3x3(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  return yaw;
}

tf2::Transform PathPublisherNode::convertPoseToTransform(const geometry_msgs::Pose &pose)
{
  tf2::Transform T;
  T.setOrigin(tf2::Vector3(pose.position.x, pose.position.y, 0));
  tf2::Quaternion q;
  q.setValue(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  T.setRotation(q);

  return T;
};

std::pair<double, double> PathPublisherNode::calculatePoseError(const geometry_msgs::Pose &pose_robot, const geometry_msgs::Pose &pose_goal)
{
  // Positional Error
  const double position_error = std::sqrt(
      std::pow(pose_robot.position.x - pose_goal.position.x, 2) +
      std::pow(pose_robot.position.y - pose_goal.position.y, 2));

  // Heading Error
  tf2::Quaternion q_robot, q_wp;
  tf2::fromMsg(pose_robot.orientation, q_robot);
  tf2::fromMsg(pose_goal.orientation, q_wp);
  const tf2::Matrix3x3 m_robot = tf2::Matrix3x3(q_robot);
  const tf2::Matrix3x3 m_goal = tf2::Matrix3x3(q_wp);

  double roll, pitch, yaw_robot, yaw_wp;
  m_robot.getRPY(roll, pitch, yaw_robot);
  m_goal.getRPY(roll, pitch, yaw_wp);

  const double heading_error = (yaw_robot - yaw_wp) / M_PI * 180.0;

  return std::pair<double, double>(position_error, heading_error);
}

} // namespace me5413_world

int main(int argc, char **argv)
{
  ros::init(argc, argv, "path_publisher_node");
  me5413_world::PathPublisherNode path_publisher_node;
  ros::spin(); // spin the ros node.
  return 0;
}