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

import rclpy
from rclpy.node import Node
import message_filters
from sensor_msgs.msg import Image
from std_msgs.msg import String
from autoware_auto_perception_msgs.msg import TrafficLightRoiArray
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import cv2
from cv_bridge import CvBridge
from mmdet.apis import inference_detector, init_detector

MODELS = [
    dict(
        ckpt="./models/yolox-s/yolox-s.pth",
        cfg="./models/yolox-s/yolox_s_8x8_300e_voc_tlr.py"
    ),
    dict(
        ckpt="./models/yolox-l/yolox-l.pth",
        cfg="./models/yolox-l/yolox_l_8x8_300e_voc_tlr.py"
    ),
]

class TrafficLightFineDetector(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.SYSTEM_DEFAULT,
            depth=1,
        )
        
        image_topic = "/sensing/camera/traffic_light/image_raw"
        rough_roi_topic = "/perception/traffic_light_recognition/rough/rois"
        image_sub = message_filters.Subscriber(self, Image, image_topic, qos_profile=qos_profile)
        rough_roi_sub = message_filters.Subscriber(self, TrafficLightRoiArray, rough_roi_topic)
        ts = message_filters.TimeSynchronizer([image_sub, rough_roi_sub], 10)
        ts.registerCallback(self.callback)
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.bridge = CvBridge()
        self.models = [
            init_detector(model.cfg, model.ckpt) for model in MODELS
        ]


    def callback(self, image_msg: Image, rough_roi_msg: TrafficLightRoiArray):
        image = self.bridge.imgmsg_to_cv2(image_msg)        


def main(args=None):
    rclpy.init(args=args)

    node = TrafficLightFineDetector()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
