// Copyright 2023 TIER IV, Inc.
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
#ifndef PERCEPTION_UTILS__PRIME_SYNCHRONIZER_HPP_
#define PERCEPTION_UTILS__PRIME_SYNCHRONIZER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/header.hpp>

#include <message_filters/null_types.h>

#include <limits>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace perception_utils
{

template <typename PrimeMsgT, typename... SecondaryMsgT>
class PrimeSynchronizer
{
  typedef double StampT;

public:
  PrimeSynchronizer(
    rclcpp::Node * node_ptr, const std::vector<std::string> & topics,
    const std::vector<rclcpp::QoS> & qos,
    std::function<void(
      const typename PrimeMsgT::ConstSharedPtr, const typename SecondaryMsgT::ConstSharedPtr...)>
      callback,
    StampT max_delay_t = 0.2, StampT max_wait_t = 0.1)
  : node_ptr_(node_ptr), callback_(callback), max_delay_t_(max_delay_t), max_wait_t_(max_wait_t)
  {
    assert((topics.size() == sizeof...(SecondaryMsgT) + 1) && "Incorrect topic number");
    assert(topics.size() == qos.size() && "topic size not equal to qos size!");
    prime_listener_ = node_ptr_->create_subscription<PrimeMsgT>(
      topics[0], qos[0], std::bind(&PrimeSynchronizer::primeCallback, this, std::placeholders::_1));
    initSecondaryListener(
      std::vector<std::string>(topics.begin() + 1, topics.end()),
      std::vector<rclcpp::QoS>(qos.begin() + 1, qos.end()));
    std::chrono::nanoseconds wait_duration(static_cast<int>(1e9 * max_wait_t));
    timer_ = rclcpp::create_timer(
      node_ptr_, node_ptr_->get_clock(), wait_duration,
      std::bind(&PrimeSynchronizer::timerCallback, this));
    timer_->cancel();
  }

private:
  template <std::size_t Idx = 0>
  void initSecondaryListener(
    const std::vector<std::string> & topics, const std::vector<rclcpp::QoS> & qos)
  {
    if constexpr (Idx < sizeof...(SecondaryMsgT)) {
      typedef std::tuple_element_t<Idx, std::tuple<SecondaryMsgT...>> type;
      std::get<Idx>(sec_listeners_) = node_ptr_->create_subscription<type>(
        topics[Idx], qos[Idx],
        std::bind(&PrimeSynchronizer::secondaryCallback<type, Idx>, this, std::placeholders::_1));
      initSecondaryListener<Idx + 1>(topics, qos);
    }
  }
  template <std::size_t Idx = 0>
  void selectSecondaryMsg(
    std::tuple<typename PrimeMsgT::ConstSharedPtr, typename SecondaryMsgT::ConstSharedPtr...> &
      argv)
  {
    if constexpr (Idx < sizeof...(SecondaryMsgT)) {
      if (std::get<Idx>(sec_messages_).empty()) {
        std::get<Idx + 1>(argv) = nullptr;
      } else {
        StampT prime_stamp = convertStampFormat(std::get<0>(argv)->header.stamp);
        StampT min_delay = std::numeric_limits<StampT>::max();
        auto best_sec_msg = std::get<Idx>(sec_messages_).begin()->second;
        for (const auto & sec_msg_p : std::get<Idx>(sec_messages_)) {
          StampT delay = std::abs(prime_stamp - sec_msg_p.first);
          if (delay < min_delay) {
            min_delay = delay;
            best_sec_msg = sec_msg_p.second;
          }
        }
        std::get<Idx + 1>(argv) = min_delay < max_delay_t_ ? best_sec_msg : nullptr;
      }
      selectSecondaryMsg<Idx + 1>(argv);
    }
  }

  template <std::size_t Idx = 0>
  bool isArgvValid(
    const std::tuple<
      typename PrimeMsgT::ConstSharedPtr, typename SecondaryMsgT::ConstSharedPtr...> & argv)
  {
    if constexpr (Idx < sizeof...(SecondaryMsgT) + 1) {
      return (std::get<Idx>(argv) != nullptr) && isArgvValid<Idx + 1>(argv);
    }
    return true;
  }

  inline StampT convertStampFormat(const std_msgs::msg::Header::_stamp_type & stamp)
  {
    return rclcpp::Time(stamp).seconds();
  }

  void primeCallback(const typename PrimeMsgT::ConstSharedPtr msg)
  {
    timer_->cancel();
    assert(prime_messages_.size() <= 1);
    /*
    if there are old prime messages waiting for synchronization with secondary messages,
    stop waiting and call the registered callback function with prime message and synchronized
    secondary messages. For secondary topics that are not synchronized, use nullptr.
    */
    for (auto & p : prime_messages_) {
      tryCallback(p.first, true);
    }
    prime_messages_.clear();
    for (auto it = prime_messages_.begin(); it != prime_messages_.end();) {
      tryCallback(it->first, true);
      it = prime_messages_.erase(it);
    }
    /*
    update the prime messages
    */
    StampT stamp = convertStampFormat(msg->header.stamp);
    prime_messages_.insert(std::make_pair(stamp, msg));
    /*
    check if secondary messages are all ready to synchronize with prime message.
    If yes, process it immediately
    */
    if (tryCallback(stamp, false)) {
      prime_messages_.clear();
    } else {
      RCLCPP_INFO(node_ptr_->get_logger(), "start timer");
      timer_->reset();
    }
  }

  template <typename M, std::size_t Idx>
  void secondaryCallback(const typename M::ConstSharedPtr msg)
  {
    std::cout << "traffic_light_occlusion_predictor: receive message for idx: " << Idx << std::endl;
    StampT stamp = convertStampFormat(msg->header.stamp);
    auto & msg_map = std::get<Idx>(sec_messages_);
    msg_map.insert(std::make_pair(stamp, msg));

    for (auto it = prime_messages_.begin(); it != prime_messages_.end();) {
      if (tryCallback(it->first, false)) {
        timer_->cancel();
        it = prime_messages_.erase(it);
      } else {
        it++;
      }
    }

    /*
    remove old secondary messages.
    */
    if (prime_messages_.empty() && msg_map.empty()) {
      return;
    }
    StampT stamp_thres =
      prime_messages_.empty() ? msg_map.rbegin()->first : prime_messages_.begin()->first;
    for (auto it = msg_map.begin(); it != msg_map.end();) {
      if (stamp_thres - it->first > max_delay_t_) {
        it = msg_map.erase(it);
      } else {
        it++;
      }
    }
  }

  void timerCallback()
  {
    RCLCPP_INFO_STREAM(node_ptr_->get_logger(), "enter timer call back");
    timer_->cancel();
    assert(prime_messages_.size() <= 1);
    for (auto it = prime_messages_.begin(); it != prime_messages_.end();) {
      tryCallback(it->first, true);
      it = prime_messages_.erase(it);
    }
  }

  bool tryCallback(StampT stamp, bool ignoreInvalidSecMsg = true)
  {
    if (prime_messages_.count(stamp) == 0) {
      return true;
    }
    std::tuple<typename PrimeMsgT::ConstSharedPtr, typename SecondaryMsgT::ConstSharedPtr...> argv;
    std::get<0>(argv) = prime_messages_[stamp];
    selectSecondaryMsg(argv);
    if (ignoreInvalidSecMsg || isArgvValid(argv)) {
      std::apply(callback_, argv);
      return true;
    }
    return false;
  }
  /**
   * @brief node pointer
   *
   */
  rclcpp::Node * node_ptr_;
  /**
   * @brief The registered callback function that would be called when the prime message and sub
   * messages are synchronized or timeout
   *
   */
  std::function<void(
    const typename PrimeMsgT::ConstSharedPtr, const typename SecondaryMsgT::ConstSharedPtr...)>
    callback_;
  /**
   * @brief the prime message subscriber
   *
   */
  typename rclcpp::Subscription<PrimeMsgT>::SharedPtr prime_listener_;
  /**
   * @brief the secondary message subscriber tuple
   *
   */
  std::tuple<typename rclcpp::Subscription<SecondaryMsgT>::SharedPtr...> sec_listeners_;
  /**
   * @brief map to store the prime messages using timestamp of the messages as key.
   *
   */
  std::map<StampT, typename PrimeMsgT::ConstSharedPtr> prime_messages_;
  rclcpp::TimerBase::SharedPtr timer_;
  /**
   * @brief tuple of maps to store the secondary messages using timestamp of the messages as key
   *
   */
  std::tuple<typename std::map<StampT, typename SecondaryMsgT::ConstSharedPtr>...> sec_messages_;
  double max_wait_t_;
  double max_delay_t_;
};

}  // namespace perception_utils

#endif  // PERCEPTION_UTILS__PRIME_SYNCHRONIZER_HPP_
