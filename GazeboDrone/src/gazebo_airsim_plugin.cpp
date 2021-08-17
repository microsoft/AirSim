/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @brief Airsim Plugin
 *
 * This plugin interacts with Microsoft Airsim to set the camera pose to the link pose in gazebo
 *
 * @author Jaeyoung Lim <jaeyoung@ethz.ch>
 */

#include "gazebo_airsim_plugin.h"

namespace gazebo {
GZ_REGISTER_MODEL_PLUGIN(AirsimPlugin)

AirsimPlugin::AirsimPlugin() : ModelPlugin() { client_ = std::make_shared<msr::airlib::RpcLibClientBase>(); }

AirsimPlugin::~AirsimPlugin() { _updateConnection->~Connection(); }

void AirsimPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  model_ = _model;
  world_ = model_->GetWorld();

  namespace_.clear();
  if (_sdf->HasElement("robotNamespace")) {
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_catapult_plugin] Please specify a robotNamespace.\n";
  }

  if (_sdf->HasElement("link_name")) {
    sdf::ElementPtr elem = _sdf->GetElement("link_name");
    std::string linkName = elem->Get<std::string>();
    this->link_ = this->model_->GetLink(linkName);

    if (!this->link_) {
      gzerr << "[gazebo_airsim_plugin] Link with name[" << linkName << "] not found. "
            << "Airsim plugin will not be able to launch the vehicle\n";
    }
  } else {
    gzerr << "[gazebo_airsim_plugin] linkName needs to be defined in the model SDF";
  }

  _updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&AirsimPlugin::OnUpdate, this, _1));

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  client_->confirmConnection();
}

void AirsimPlugin::OnUpdate(const common::UpdateInfo &) {
  if (link_ == NULL)
    gzthrow("[gazebo_airsim_plugin] Couldn't find specified link \n");

  /// TODO: Get Camera pose from gazebo
  Eigen::Vector3d pos_ned;
  Eigen::Vector4d att_ned;
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Pose3d T_W_I = link_->WorldPose();
#else
  ignition::math::Pose3d T_W_I = ignitionFromGazeboMath(link_->GetWorldPose());
#endif
  ignition::math::Vector3d pos = T_W_I.Pos();
  ignition::math::Quaterniond att = T_W_I.Rot();

  msr::airlib::Vector3r p(pos.X(), -pos.Y(), -pos.Z());             // Convert to NED since airsim position is in NED
  msr::airlib::Quaternionr o(att.W(), att.X(), -att.Y(), -att.Z()); // Convert to NED since airsim position is in NED

  client_->simSetVehiclePose(msr::airlib::Pose(p, o), true);
}

} // namespace gazebo
