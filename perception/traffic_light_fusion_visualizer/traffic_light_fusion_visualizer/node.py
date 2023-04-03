# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import copy

from autoware_auto_perception_msgs.msg import TrafficLight
from autoware_auto_perception_msgs.msg import TrafficSignal
from autoware_auto_perception_msgs.msg import TrafficSignalArray
import cv2
from cv_bridge import CvBridge
import message_filters
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from sensor_msgs.msg import Image

state2label_ = {
    TrafficLight.RED: "red",
    TrafficLight.AMBER: "yellow",
    TrafficLight.GREEN: "green",
    TrafficLight.WHITE: "white",
    TrafficLight.CIRCLE: "circle",
    TrafficLight.LEFT_ARROW: "left",
    TrafficLight.RIGHT_ARROW: "right",
    TrafficLight.UP_ARROW: "straight",
    TrafficLight.DOWN_ARROW: "down",
    TrafficLight.DOWN_LEFT_ARROW: "down_left",
    TrafficLight.DOWN_RIGHT_ARROW: "down_right",
    TrafficLight.CROSS: "cross",
    TrafficLight.UNKNOWN: "unknown",
}


class TrafficLightFusionVisualizer(Node):
    def __init__(self):
        super().__init__("minimal_publisher")

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.SYSTEM_DEFAULT,
            depth=1,
        )
        image_6_topic = "/perception/traffic_light_recognition/camera6/debug/rois"
        image_7_topic = "/perception/traffic_light_recognition/camera7/debug/rois"
        signal_topic = "/perception/traffic_light_recognition/traffic_signals"
        image_6_sub = message_filters.Subscriber(
            self, Image, image_6_topic, qos_profile=qos_profile
        )
        image_7_sub = message_filters.Subscriber(
            self, Image, image_7_topic, qos_profile=qos_profile
        )
        signal_sub = message_filters.Subscriber(self, TrafficSignalArray, signal_topic)
        ts_6 = message_filters.TimeSynchronizer([image_6_sub, signal_sub], 100000)
        ts_6.registerCallback(self.callback)
        ts_7 = message_filters.TimeSynchronizer([image_7_sub, signal_sub], 100000)
        ts_7.registerCallback(self.callback)
        self.bridge = CvBridge()
        self.image_6 = None
        self.image_7 = None

    def callback(self, debug_image: Image, signals: TrafficSignalArray):
        print(f"receive image: {debug_image.header.frame_id}")
        if "camera6" in debug_image.header.frame_id:
            self.image_6 = self.bridge.imgmsg_to_cv2(debug_image, "bgr8")
        else:
            self.image_7 = self.bridge.imgmsg_to_cv2(debug_image, "bgr8")

        if self.image_6 is not None and self.image_7 is not None:
            stamp = debug_image.header.stamp
            save_path = "/home/mingyuli/tmp/tl_output/fusion_output/" + str(stamp) + ".jpg"
            image_6 = copy.deepcopy(self.image_6)
            text = ""
            for signal in signals.signals:
                text += str(signal.map_primitive_id) + ": "
                signal: TrafficSignal
                for light in signal.lights:
                    text += (state2label_[light.color] + "-" + state2label_[light.shape]) + ","
                text += "   "
            cv2.putText(
                image_6,
                text,
                (10, image_6.shape[0] * 9 // 10),
                cv2.FONT_HERSHEY_COMPLEX,
                1,
                (0, 0, 255),
                2,
            )
            self.image_7 = cv2.resize(self.image_7, (image_6.shape[1], image_6.shape[0]))
            image_6 = cv2.hconcat([image_6, self.image_7])
            cv2.imwrite(save_path, image_6)
        print("saved image")


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightFusionVisualizer()
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
