// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_mppi_controller/critic_manager.hpp"

namespace mppi
{

void CriticManager::on_configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros, ParametersHandler * param_handler)
{
  parent_ = parent;
  costmap_ros_ = costmap_ros;
  name_ = name;
  auto node = parent_.lock();
  logger_ = node->get_logger();
  parameters_handler_ = param_handler;

  getParams();
  loadCritics();
}

void CriticManager::getParams()
{
  auto node = parent_.lock();
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(critic_names_, "critics", std::vector<std::string>{}, ParameterType::Static);
  getParam(enable_debug_pub_, "publish_critics_debug", false, ParameterType::Static);
}

void CriticManager::loadCritics()
{
  if (!loader_) {
    loader_ = std::make_unique<pluginlib::ClassLoader<critics::CriticFunction>>(
      "nav2_mppi_controller", "mppi::critics::CriticFunction");
  }

  critics_.clear();
  for (auto name : critic_names_) {
    std::string fullname = getFullName(name);
    auto instance = std::unique_ptr<critics::CriticFunction>(
      loader_->createUnmanagedInstance(fullname));
    critics_.push_back(std::move(instance));
    critics_.back()->on_configure(
      parent_, name_, name_ + "." + name, costmap_ros_,
      parameters_handler_);
    RCLCPP_INFO(logger_, "Critic loaded : %s", fullname.c_str());
  }

  auto node = parent_.lock();
  if (enable_debug_pub_){
    debug_pub_ = std::make_unique<mppi::DebugPublisher>();
    debug_pub_->configure(
      node,
      "~/critics_debug",
      true,
      10.0
    );
  }
}

std::string CriticManager::getFullName(const std::string & name)
{
  return "mppi::critics::" + name;
}

void CriticManager::evalTrajectoriesScores(
  CriticData & data) const
{
  // for (const auto & critic : critics_) {
  //   if (data.fail_flag) {
  //     break;
  //   }
  //   critic->score(data);
  // }
  std::vector<bool> changed;
  std::vector<float> delta_sum;
  std::vector<float> delta_abs_sum;
  
  if (enable_debug_pub_) {
    changed.reserve(critics_.size());
    delta_sum.reserve(critics_.size());
    delta_abs_sum.reserve(critics_.size());
  }

  for (size_t i = 0; i < critics_.size(); ++i) {
    if (data.fail_flag) {
      break;
    }

    xt::xtensor<float, 1> costs_before;
    if (enable_debug_pub_) {
      costs_before = data.costs;
    }

    critics_[i]->score(data);

    if (enable_debug_pub_) {
      xt::xtensor<float, 1> cost_diff = data.costs - costs_before;
      float sum = xt::sum(cost_diff)();
      float abs_sum = xt::sum(xt::abs(cost_diff))();
      
      delta_sum.push_back(sum);
      delta_abs_sum.push_back(abs_sum);
      changed.push_back(sum != 0.0f || abs_sum != 0.0f);
    }
  }

  if (debug_pub_) {
    debug_pub_->publishIfDue(
      critic_names_,
      changed,
      delta_sum,
      delta_abs_sum,
      data.fail_flag
    );
  }

}

}  // namespace mppi
