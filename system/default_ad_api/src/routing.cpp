// Copyright 2022 TIER IV, Inc.
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

#include "routing.hpp"

#include "utils/route_conversion.hpp"

#include <memory>

namespace default_ad_api
{

RoutingNode::RoutingNode(const rclcpp::NodeOptions & options) : Node("routing", options)
{
  const auto adaptor = component_interface_utils::NodeAdaptor(this);
  group_cli_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  adaptor.init_pub(pub_state_);
  adaptor.init_pub(pub_route_);
  adaptor.init_sub(sub_state_, this, &RoutingNode::on_state);
  adaptor.init_sub(sub_route_, this, &RoutingNode::on_route);
  adaptor.init_cli(cli_set_route_, group_cli_);
  adaptor.init_srv(srv_set_route_, this, &RoutingNode::on_set_route);
  adaptor.relay_service(cli_set_route_points_, srv_set_route_points_, group_cli_);
  adaptor.relay_service(cli_clear_route_, srv_clear_route_, group_cli_);
}

void RoutingNode::on_state(const State::Message::ConstSharedPtr msg)
{
  pub_state_->publish(*msg);

  // TODO(Takagi, Isamu): Remove when the mission planner supports an empty route.
  if (msg->state == State::Message::UNSET) {
    pub_route_->publish(conversion::create_empty_route(msg->stamp));
  }
}

void RoutingNode::on_route(const Route::Message::ConstSharedPtr msg)
{
  pub_route_->publish(conversion::convert_route(*msg));
}

void RoutingNode::on_set_route(
  const autoware_ad_api::routing::SetRoute::Service::Request::SharedPtr req,
  const autoware_ad_api::routing::SetRoute::Service::Response::SharedPtr res)
{
  const auto inner_req = std::make_shared<planning_interface::SetRoute::Service::Request>();
  *inner_req = conversion::convert_set_route(*req);

  const auto inner_res = cli_set_route_->call(inner_req);
  component_interface_utils::status::copy(res, inner_res);  // NOLINT cpplint false positive
}

}  // namespace default_ad_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(default_ad_api::RoutingNode)
