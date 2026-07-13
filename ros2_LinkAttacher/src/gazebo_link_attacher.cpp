/*
# ===================================== COPYRIGHT ===================================== #
#                                                                                       #
#  IFRA (Intelligent Flexible Robotics and Assembly) Group, CRANFIELD UNIVERSITY        #
#  Created on behalf of the IFRA Group at Cranfield University, United Kingdom          #
#  E-mail: IFRA@cranfield.ac.uk                                                         #
#                                                                                       #
#  Licensed under the Apache-2.0 License.                                               #
#  You may not use this file except in compliance with the License.                     #
#  You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0  #
#                                                                                       #
#  Unless required by applicable law or agreed to in writing, software distributed      #
#  under the License is distributed on an "as-is" basis, without warranties or          #
#  conditions of any kind, either express or implied. See the License for the specific  #
#  language governing permissions and limitations under the License.                    #
#                                                                                       #
#  IFRA Group - Cranfield University                                                    #
#  AUTHORS: Mikel Bueno Viso - Mikel.Bueno-Viso@cranfield.ac.uk                         #
#           Dr. Seemal Asif  - s.asif@cranfield.ac.uk                                   #
#           Prof. Phil Webb  - p.f.webb@cranfield.ac.uk                                 #
#                                                                                       #
#  Date: May, 2023.                                                                     #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #

# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# IFRA-Cranfield (2023) IFRA Gazebo-ROS2 Link Attacher. URL: https://github.com/IFRA-Cranfield/IFRA_LinkAttacher.
*/

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/Entity.hh>
#include <gazebo/physics/Light.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/SurfaceParams.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/PhysicsEngine.hh>

#include <gazebo_ros/node.hpp>
#include <algorithm>
#include <memory>
#include <vector>

#include "gazebo_ros/conversions/builtin_interfaces.hpp"
#include "gazebo_ros/conversions/geometry_msgs.hpp"

#include "ros2_linkattacher/gazebo_link_attacher.hpp"   // INCLUDE HADER FILE.
#include <linkattacher_msgs/srv/attach_link.hpp>        // INCLUDE ROS2 SERVICE.
#include <linkattacher_msgs/srv/detach_link.hpp>        // INCLUDE ROS2 SERVICE.

// GLOBAL VARIABLE:
std::vector<JointSTRUCT> GV_joints;

namespace gazebo_ros
{

class GazeboLinkAttacherPrivate
{
public:

  // ATTACH (ROS2 service):
  void Attach(
    linkattacher_msgs::srv::AttachLink::Request::SharedPtr _req,
    linkattacher_msgs::srv::AttachLink::Response::SharedPtr _res);

  // DETACH (ROS2 service):
  void Detach(
    linkattacher_msgs::srv::DetachLink::Request::SharedPtr _req,
    linkattacher_msgs::srv::DetachLink::Response::SharedPtr _res);

  // World pointer from Gazebo.
  gazebo::physics::WorldPtr world_;

  /// ROS node for communication, managed by gazebo_ros.
  gazebo_ros::Node::SharedPtr ros_node_;

  // ROS services to handle requests for attach/detach.
  rclcpp::Service<linkattacher_msgs::srv::AttachLink>::SharedPtr attach_link_service_;
  rclcpp::Service<linkattacher_msgs::srv::DetachLink>::SharedPtr detach_link_service_;

  // getJoint function:
  bool getJoint(std::string M1, std::string L1, std::string M2, std::string L2, JointSTRUCT &joint);

  // A physical link can belong to only one active attachment.  This allows
  // both hands to hold separate objects while preventing an object (or hand)
  // from being constrained by two attachment joints at once.
  bool isLinkAttached(const std::string &model, const std::string &link) const;

};

GazeboLinkAttacher::GazeboLinkAttacher()
: impl_(std::make_unique<GazeboLinkAttacherPrivate>())
{
}

GazeboLinkAttacher::~GazeboLinkAttacher()
{
}

void GazeboLinkAttacher::Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  
  // Gazebo WORLD:
  impl_->world_ = _world;

  // ROS2 NODE:
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  // ROS2 SERVICE SERVERS:
  impl_->attach_link_service_ =
    impl_->ros_node_->create_service<linkattacher_msgs::srv::AttachLink>(
    "ATTACHLINK", std::bind(
      &GazeboLinkAttacherPrivate::Attach, impl_.get(),
      std::placeholders::_1, std::placeholders::_2));
  impl_->detach_link_service_ =
    impl_->ros_node_->create_service<linkattacher_msgs::srv::DetachLink>(
    "DETACHLINK", std::bind(
      &GazeboLinkAttacherPrivate::Detach, impl_.get(),
      std::placeholders::_1, std::placeholders::_2));

}

void GazeboLinkAttacherPrivate::Attach(
  linkattacher_msgs::srv::AttachLink::Request::SharedPtr _req,
  linkattacher_msgs::srv::AttachLink::Response::SharedPtr _res)
{

  // Get the first link:
  gazebo::physics::ModelPtr model1 = world_->ModelByName(_req->model1_name);
  if (!model1) {
    _res->success = false;
    _res->message = "Failed to find model with name: " + _req->model1_name;
    return;
  }
  gazebo::physics::LinkPtr link1 = model1->GetLink(_req->link1_name);
  if (!link1) {
    _res->success = false;
    _res->message = "Failed to find link with name: " + _req->link1_name;
    return;
  }

  // Get the second link:
  gazebo::physics::ModelPtr model2 = world_->ModelByName(_req->model2_name);
  if (!model2) {
    _res->success = false;
    _res->message = "Failed to find model with name: " + _req->model2_name;
    return;
  }
  gazebo::physics::LinkPtr link2 = model2->GetLink(_req->link2_name);
  if (!link2) {
    _res->success = false;
    _res->message = "Failed to find link with name: " + _req->link2_name;
    return;
  }

  JointSTRUCT existing_joint;
  if (this->getJoint(
      _req->model1_name, _req->link1_name,
      _req->model2_name, _req->link2_name, existing_joint)) {
    _res->success = false;
    _res->message = "Both requested links are already attached to each other.";
    return;
  }
  if (this->isLinkAttached(_req->model1_name, _req->link1_name)) {
    _res->success = false;
    _res->message = "The first requested link is already attached to another link.";
    return;
  }
  if (this->isLinkAttached(_req->model2_name, _req->link2_name)) {
    _res->success = false;
    _res->message = "The second requested link is already attached to another link.";
    return;
  }

  // Create a fixed joint between the two links.
  JointSTRUCT joint_record;
  joint_record.model1 = _req->model1_name;
  joint_record.model2 = _req->model2_name;
  joint_record.link1 = _req->link1_name;
  joint_record.link2 = _req->link2_name;
  joint_record.m1 = model1;
  joint_record.m2 = model2;
  joint_record.l1 = link1;
  joint_record.l2 = link2;
  joint_record.joint_name = _req->model1_name + "_" + _req->link1_name + "_" +
    _req->model2_name + "_" + _req->link2_name + "_joint";

  gazebo::physics::JointPtr joint = model1->CreateJoint(
    joint_record.joint_name, "revolute", link1, link2);
  joint->Attach(link1, link2);
  joint->Load(link1, link2, ignition::math::Pose3d());
  joint->SetProvideFeedback(true);

  joint->SetAxis(0, ignition::math::Vector3d(1, 0, 0));
  joint->SetUpperLimit(0, 0);
  joint->SetLowerLimit(0, 0);
  joint->SetEffortLimit(0, 0);
  joint->SetDamping(1, 1.0);

  joint->Init();
  model1->Update();

  joint_record.joint = joint;

  // Remove all physical interaction for the grasped object's link while it is
  // attached. Creating the fixed joint alone does NOT disable collisions, so
  // if ATTACH happens while the gripper fingers are penetrating/pressing the
  // object, ODE has to satisfy the new rigid constraint AND the live contact
  // constraints simultaneously -> the object gets ejected / jitters. Setting
  // the object link's collide bitmask to 0 makes it collide with nothing (two
  // collisions interact only if their bitmasks intersect); the fixed joint
  // holds it rigidly to the gripper. The original bitmask is restored on DETACH
  // so the object interacts with the world again once released. Note: the
  // contact is actually between the object and the gripper FINGER links (not
  // link1, the wrist), so disabling the object's collisions outright is the
  // correct way to remove that interaction.
  for (const auto & collision : link2->GetCollisions()) {
    if (!collision) {
      continue;
    }
    gazebo::physics::SurfaceParamsPtr surface = collision->GetSurface();
    if (!surface) {
      continue;
    }
    joint_record.disabled_collisions.push_back(collision);
    joint_record.saved_collide_bitmask.push_back(surface->collideBitmask);
    surface->collideBitmask = 0u;
  }

  GV_joints.push_back(joint_record);

  _res->success = true;
  _res->message = "ATTACHED: {MODEL , LINK} -> {" + _req->model1_name + " , " + _req->link1_name + "} -- {" + _req->model2_name + " , " + _req->link2_name + "}.";
}

void GazeboLinkAttacherPrivate::Detach(
  linkattacher_msgs::srv::DetachLink::Request::SharedPtr _req,
  linkattacher_msgs::srv::DetachLink::Response::SharedPtr _res)
{

  // CHECK if -> Joint already exists in GV_joints:
  JointSTRUCT j;
  if (this->getJoint(_req->model1_name, _req->link1_name, _req->model2_name, _req->link2_name, j)){
    j.joint->Detach();

    // Restore the object link's original collide bitmask (zeroed on ATTACH)
    // so it physically interacts with the world again -- e.g. rests on the
    // table / placement area after being released.
    for (size_t i = 0; i < j.disabled_collisions.size(); ++i) {
      if (j.disabled_collisions[i]) {
        gazebo::physics::SurfaceParamsPtr surface = j.disabled_collisions[i]->GetSurface();
        if (surface) {
          surface->collideBitmask = j.saved_collide_bitmask[i];
        }
      }
    }

    _res->success = true;
    _res->message = "DETACHED: {MODEL , LINK} -> {" + _req->model1_name + " , " + _req->link1_name + "} -- {" + _req->model2_name + " , " + _req->link2_name + "}.";
    
    // Remove the specific Gazebo joint and its bookkeeping entry, so either
    // hand can later attach another object (or reattach this one).
    j.m1->RemoveJoint(j.joint_name);
    GV_joints.erase(
      std::remove_if(
        GV_joints.begin(), GV_joints.end(),
        [&_req](const JointSTRUCT &record) {
          return record.model1 == _req->model1_name &&
                 record.link1 == _req->link1_name &&
                 record.model2 == _req->model2_name &&
                 record.link2 == _req->link2_name;
        }),
      GV_joints.end());
    
    return;
  } else {
    _res->success = false;
    _res->message = "DETACHED -- ERROR (Joint does not exist!): {MODEL , LINK} -> {" + _req->model1_name + " , " + _req->link1_name + "} -- {" + _req->model2_name + " , " + _req->link2_name + "}.";
  }

}

bool GazeboLinkAttacherPrivate::getJoint(std::string M1, std::string L1, std::string M2, std::string L2, JointSTRUCT &joint)
  {
    JointSTRUCT j;
    for(std::vector<JointSTRUCT>::iterator it = GV_joints.begin(); it != GV_joints.end(); ++it){
      j = *it;
      if ((j.model1.compare(M1) == 0) && (j.model2.compare(M2) == 0) && (j.link1.compare(L1) == 0) && (j.link2.compare(L2) == 0)){
        joint = j;
        return true;
      }
    }
    return false;
  }

bool GazeboLinkAttacherPrivate::isLinkAttached(
  const std::string &model, const std::string &link) const
{
  return std::any_of(
    GV_joints.begin(), GV_joints.end(),
    [&model, &link](const JointSTRUCT &joint) {
      return (joint.model1 == model && joint.link1 == link) ||
             (joint.model2 == model && joint.link2 == link);
    });
}

GZ_REGISTER_WORLD_PLUGIN(GazeboLinkAttacher)

}  // namespace gazebo_ros
