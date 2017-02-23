/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include "fcu_sim_plugins/odometry_plugin.h"

namespace gazebo {

OdometryPlugin::~OdometryPlugin() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}


void OdometryPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model
  model_ = _model;
  world_ = model_->GetWorld();

  odometry_queue_.clear();

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_odometry_plugin] Please specify a robotNamespace.\n";
  node_handle_ = new ros::NodeHandle(namespace_);

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_odometry_plugin] Please specify a linkName.\n";
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_odometry_plugin] Couldn't find specified link \"" << link_name_ << "\".");

  getSdfParam<std::string>(_sdf, "poseTopic", pose_pub_topic_, "pose");
  getSdfParam<std::string>(_sdf, "poseWithCovarianceTopic", pose_with_covariance_pub_topic_, "pose_with_covariance");
  getSdfParam<std::string>(_sdf, "positionTopic", position_pub_topic_, "position");
  getSdfParam<std::string>(_sdf, "transformTopic", transform_pub_topic_, "transform");
  getSdfParam<std::string>(_sdf, "odometryTopic", odometry_pub_topic_, "odometry");
  getSdfParam<std::string>(_sdf, "parentFrameId", parent_frame_id_, "world");

  parent_link_ = world_->GetEntity(parent_frame_id_);
  if (parent_link_ == NULL && parent_frame_id_ != "world") {
    gzthrow("[gazebo_odometry_plugin] Couldn't find specified parent link \"" << parent_frame_id_ << "\".");
  }

  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&OdometryPlugin::OnUpdate, this, _1));
  pose_pub_ = node_handle_->advertise<geometry_msgs::PoseStamped>(pose_pub_topic_, 10);
  transform_pub_ = node_handle_->advertise<geometry_msgs::TransformStamped>(transform_pub_topic_, 10);
  odometry_pub_ = node_handle_->advertise<nav_msgs::Odometry>(odometry_pub_topic_, 10);
  euler_pub_ = node_handle_->advertise<geometry_msgs::Vector3Stamped>("euler", 1);
}

// This gets called by the world update start event.
void OdometryPlugin::OnUpdate(const common::UpdateInfo& _info) {
  // C denotes child frame, P parent frame, and W world frame.
  // Further C_pose_W_P denotes pose of P wrt. W expressed in C.
  math::Pose W_pose_W_C = link_->GetWorldCoGPose();
  math::Vector3 C_linear_velocity_W_C = link_->GetRelativeLinearVel();
  math::Vector3 C_angular_velocity_W_C = link_->GetRelativeAngularVel();

  math::Vector3 gazebo_linear_velocity = C_linear_velocity_W_C;
  math::Vector3 gazebo_angular_velocity = C_angular_velocity_W_C;
  math::Pose gazebo_pose = W_pose_W_C;

  if (parent_frame_id_ != "world") {
    math::Pose W_pose_W_P = parent_link_->GetWorldPose();
    math::Vector3 P_linear_velocity_W_P = parent_link_->GetRelativeLinearVel();
    math::Vector3 P_angular_velocity_W_P = parent_link_->GetRelativeAngularVel();
    math::Pose C_pose_P_C_ = W_pose_W_C - W_pose_W_P;
    math::Vector3 C_linear_velocity_P_C;
    C_linear_velocity_P_C = - C_pose_P_C_.rot.GetInverse()
        * P_angular_velocity_W_P.Cross(C_pose_P_C_.pos)
        + C_linear_velocity_W_C
        - C_pose_P_C_.rot.GetInverse() * P_linear_velocity_W_P;
    gazebo_angular_velocity = C_angular_velocity_W_C
        - C_pose_P_C_.rot.GetInverse() * P_angular_velocity_W_P;
    gazebo_linear_velocity = C_linear_velocity_P_C;
    gazebo_pose = C_pose_P_C_;
  }

  nav_msgs::Odometry odometry;
  odometry.header.frame_id = "NED";
  odometry.header.seq = odometry_sequence_++;
  odometry.header.stamp.sec = (world_->GetSimTime()).sec;
  odometry.header.stamp.nsec = (world_->GetSimTime()).nsec;
  odometry.child_frame_id = namespace_;
  copyPosition(gazebo_pose.pos, &odometry.pose.pose.position);

  // Convert from NWU (gazebo coordinates) to NED (MAV coordinates)
  odometry.pose.pose.position.y *= -1.0;
  odometry.pose.pose.position.z *= -1.0;
  odometry.pose.pose.orientation.w = gazebo_pose.rot.w;
  odometry.pose.pose.orientation.x = gazebo_pose.rot.x;
  odometry.pose.pose.orientation.y = -1.0*gazebo_pose.rot.y;
  odometry.pose.pose.orientation.z = -1.0*gazebo_pose.rot.z;
  odometry.twist.twist.linear.x = gazebo_linear_velocity.x;
  odometry.twist.twist.linear.y = -1.0*gazebo_linear_velocity.y;
  odometry.twist.twist.linear.z = -1.0*gazebo_linear_velocity.z;
  odometry.twist.twist.angular.x = gazebo_angular_velocity.x;
  odometry.twist.twist.angular.y = -1.0*gazebo_angular_velocity.y;
  odometry.twist.twist.angular.z = -1.0*gazebo_angular_velocity.z;

  // Publish all the topics, for which the topic name is specified.
  if (euler_pub_.getNumSubscribers() > 0) {
    geometry_msgs::Vector3Stamped euler;
    tf::Quaternion q;
    tf::quaternionMsgToTF(odometry.pose.pose.orientation, q);
    tf::Matrix3x3 R(q);
    double roll, pitch, yaw;
    R.getEulerYPR(yaw, pitch, roll);
    euler.header = odometry.header;
    euler.vector.x = roll;
    euler.vector.y = pitch;
    euler.vector.z = yaw;
    euler_pub_.publish(euler);
  }
  if (pose_pub_.getNumSubscribers() > 0) {
    geometry_msgs::PoseStampedPtr pose(new geometry_msgs::PoseStamped);
    pose->header = odometry.header;
    pose->pose = odometry.pose.pose;
    pose_pub_.publish(pose);
  }

  if (transform_pub_.getNumSubscribers() > 0) {
    geometry_msgs::TransformStampedPtr transform(new geometry_msgs::TransformStamped);
    transform->header = odometry.header;
    geometry_msgs::Vector3 translation;
    translation.x = odometry.pose.pose.position.x;
    translation.y = odometry.pose.pose.position.y;
    translation.z = odometry.pose.pose.position.z;
    transform->transform.translation = translation;
    transform->transform.rotation = odometry.pose.pose.orientation;
    transform_pub_.publish(transform);
  }
  if (odometry_pub_.getNumSubscribers() > 0) {
    odometry_pub_.publish(odometry);
  }
  tf::Quaternion tf_q;
  tf::quaternionMsgToTF(odometry.pose.pose.orientation, tf_q);
  tf::Vector3 tf_v(odometry.pose.pose.position.x, odometry.pose.pose.position.y, odometry.pose.pose.position.z);
  tf_ = tf::Transform(tf_q, tf_v);
  transform_broadcaster_.sendTransform(tf::StampedTransform(tf_, odometry.header.stamp, parent_frame_id_, namespace_));
}

GZ_REGISTER_MODEL_PLUGIN(OdometryPlugin);
}
