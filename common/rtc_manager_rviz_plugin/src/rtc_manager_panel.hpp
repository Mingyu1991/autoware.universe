//
//  Copyright 2020 Tier IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//

#ifndef RTC_MANAGER_PANEL_HPP_
#define RTC_MANAGER_PANEL_HPP_

#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <QTableWidget>

#ifndef Q_MOC_RUN
// cpp
#include <algorithm>
#include <memory>
#include <string>
#include <vector>
// ros
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <unique_identifier_msgs/msg/uuid.hpp>
// autoware
#include <tier4_rtc_msgs/msg/command.hpp>
#include <tier4_rtc_msgs/msg/cooperate_command.hpp>
#include <tier4_rtc_msgs/msg/cooperate_status.hpp>
#include <tier4_rtc_msgs/msg/cooperate_status_array.hpp>
#include <tier4_rtc_msgs/msg/module.hpp>
#include <tier4_rtc_msgs/srv/cooperate_commands.hpp>
#endif

namespace rviz_plugins
{
using tier4_rtc_msgs::msg::Command;
using tier4_rtc_msgs::msg::CooperateCommand;
using tier4_rtc_msgs::msg::CooperateResponse;
using tier4_rtc_msgs::msg::CooperateStatus;
using tier4_rtc_msgs::msg::CooperateStatusArray;
using tier4_rtc_msgs::msg::Module;
using tier4_rtc_msgs::srv::CooperateCommands;
using unique_identifier_msgs::msg::UUID;

class RTCManagerPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit RTCManagerPanel(QWidget * parent = nullptr);
  void onInitialize() override;

protected:
  void onRTCStatus(const CooperateStatusArray::ConstSharedPtr msg);

  rclcpp::Node::SharedPtr raw_node_;
  rclcpp::Subscription<CooperateStatusArray>::SharedPtr sub_rtc_status_;
  rclcpp::Client<CooperateCommands>::SharedPtr client_rtc_commands_;
  QTableWidget * rtc_table_;
  size_t column_size_ = {5};
};

}  // namespace rviz_plugins

#endif  // RTC_MANAGER_PANEL_HPP_
